# MA735エンコーダシミュレータ - Arduino実装

## 概要

MA735磁気式角度センサーのエンコーダ出力波形をArduino Nanoでシミュレートするプログラムです。  
A0ピンに接続した可変抵抗で0〜6500rpmの回転数を制御し、リアルタイムでABZ + G相の信号を出力します。  
また、A4, A5ピンでキルスイッチ、スタートボタンの入力を受け取り、D6, D7ピンでそれぞれ信号を出力します。  
シリアルコマンドにより外部から各スイッチの状態や回転数を上書き制御することもできます。

## 仕様

### ハードウェア要件

- **マイコン**: Arduino Nano (ATmega328P)
- **入力**:
  - A4: キルスイッチ（内部プルアップ、GNDに接続でON）
  - A5: スタートボタン（内部プルアップ、GNDに接続でON）
  - A0: 可変抵抗（0〜5V ADC入力）
- **出力**:
  - D2: A相出力
  - D5: G相出力
  - D6: スタートボタン出力（アクティブLow）
  - D7: キルスイッチ出力（アクティブLow）
  - D8: B相出力  
  - D9: Z相出力（インデックス）

### エンコーダ設定

- **PPT（Pulses per Turn）**: 359（360パルス/回転）
- **ILIP（Index Length/Position）**: 1111
- **分解能**: 1440エッジ/回転（360パルス × 4）
- **最大回転数**: 6500rpm
- **始動時回転数**: 600rpm

### G相特殊設定

- **出力周期**: 2回転に1パルス
- **開始角度**: 450度（1.5回転目）
- **パルス幅**: 720エッジ分（4エッジ/1度）
- **設定変更**: 定数で変更可能

## プログラム構成

### 主要変数

```cpp
// エンコーダ仕様
const int PULSES_PER_REV = 360;   // 1回転あたりのパルス数
const int MAX_RPM = 6500;         // 最大回転数
const int START_RPM = 600;        // 始動時の回転数

// G相設定（変更可能）
const int G_PHASE_ROTATIONS = 2;      // G相の出力周期（回転数）
const int G_PHASE_START_DEG = 450;    // G相開始角度（度）
const int G_PHASE_EDGES = 720;        // G相出力エッジ数

// 高速計算用定数（整数演算・オーバーフロー対策済み）
const int TOTAL_EDGES_PER_REV = PULSES_PER_REV * 4;
const int TOTAL_EDGES_PER_CYCLE = TOTAL_EDGES_PER_REV * G_PHASE_ROTATIONS;
const int G_PHASE_START_EDGES = (int)((int32_t)G_PHASE_START_DEG * TOTAL_EDGES_PER_REV / 360);
const int G_PHASE_END_EDGES = G_PHASE_START_EDGES + G_PHASE_EDGES;

// 高速ADC処理用変数
volatile uint16_t adc0 = 0;              // ADC読み取り値（A0）
volatile bool adc0_read_complete = false; // ADC完了フラグ

// シリアル制御モード
bool serialControlMode = false;
bool serialKillSW = false;    // KILL SWのシリアル上書き（true=ON）
bool serialStartSW = false;   // START SWのシリアル上書き（true=ON）
uint16_t serialRPM = 0;       // シリアルモードでのRPM指定値
```

### ピン定義

```cpp
const int POT_PIN     = A0;  // 可変抵抗
const int KILL_IN_PIN = A4;  // キルスイッチ入力
const int START_IN_PIN = A5; // スタータボタン入力
const int A_PHASE_PIN = 2;   // A相出力
const int B_PHASE_PIN = 8;   // B相出力
const int Z_PHASE_PIN = 9;   // Z相出力
const int G_PHASE_PIN = 5;   // G相出力
const int KILL_OUT_PIN  = 7; // キルスイッチ出力（アクティブLow）
const int START_OUT_PIN = 6; // スタータボタン出力（アクティブLow）
```

### 主要機能

#### 1. setup()

- シリアル通信初期化（115200bps）
- ピン設定
- 高速ADC初期化（フリーランモード、A0チャンネル、DIDR0設定）
- 起動メッセージとシリアルコマンドヘルプ表示

#### 2. loop()

- シリアル入力チェック（非ブロッキング）
- SW状態読み取り（デバウンス処理付き、20ms）
- シリアルモード解除チェック（物理SW OFF→ON エッジ検出）
- シリアルモード時: `serialKillSW`/`serialStartSW` でSW状態を上書き
- 共通出力制御ロジック（物理・シリアル共通）:
  - KILL SW ON + START SW ON → D6=LOW、RPM=600rpm
  - KILL SW ON + START SW OFF → D6=HIGH、RPMは可変抵抗orシリアル指定値
  - KILL SW OFF → D7=HIGH、RPM=0
- パルス間隔計算（整数演算）
- 高精度タイマー制御（10kHz超でTimer1 CTC割り込みに自動切替）
- デバッグ情報出力（1秒間隔）

#### 3. generatePulse()

- パルスカウンター管理
- A相/B相の90度位相差生成
- Z相インデックス生成（ILIP=1111: A相とB相が両方HIGHの期間）
- G相特殊パターン生成（整数演算のみ）
- 出力ピン制御（レジスタ直接操作）

#### 4. setupHighSpeedADC()

- ADCレジスタ直接制御（A0チャンネル）
- フリーランモード設定（ADCSRB=0）
- プリスケーラー128（125KHz ADCクロック）
- 割り込み有効化
- `DIDR0`でA0のデジタル入力バッファを無効化（デジタルSWへのノイズ低減）

#### 5. checkSerialInput() / processSerialCommand()

- 非ブロッキングバッファ方式（32バイト固定長）
- 改行(`\n`/`\r`)でコマンド確定・処理
- `String`クラスを使わず`strcmp`/`strncmp`/`atoi`で処理（ヒープ断片化防止）

## シリアルコマンド

| コマンド | 動作 |
| ------- | ---- |
| `KILL:ON` | KILL SW = ON として扱う（D7=LOW、エンコーダ動作） |
| `KILL:OFF` | KILL SW = OFF として扱う（D7=HIGH、RPM=0） |
| `START:ON` | START SW = ON として扱う（D6=LOW、RPM=600） |
| `START:OFF` | START SW = OFF として扱う（D6=HIGH、RPMはADCまたはRPMコマンド値） |
| `RPM:0-6500` | KILL=ON・START=OFFを自動設定し指定RPMで動作 |
| `STATUS` | 現在の状態を表示 |
| `RESET` | シリアル制御モードを解除し物理SW制御に戻る |

### シリアル制御モードの動作

- いずれかのコマンドを受信した時点で**シリアル制御モード**に移行
- 以下の条件でシリアル制御モードを**自動解除**:
  - 物理キルスイッチ（A4）が新たに押された瞬間（OFF→ONエッジ）
  - 物理スタートボタン（A5）が新たに押された瞬間（OFF→ONエッジ）
  - Arduino Nanoがリセットされた時
- `RESET`コマンドで明示的に解除することも可能

## 動作原理

### ABZ信号生成

1. **A相**: 基本方形波（360パルス/回転、1440エッジ/回転）
2. **B相**: A相より90度遅れた方形波
3. **Z相**: 1回転に1回、A相とB相が両方HIGHの期間に出力（ILIP=1111）

### G相信号生成

1. **周期**: 2回転（2880エッジ）でカウンターリセット
2. **開始判定**: 事前計算された `G_PHASE_START_EDGES`（450度 = 1800エッジ）から
3. **終了判定**: `G_PHASE_END_EDGES`（= 開始 + 720エッジ）まで HIGH

### タイミング制御

- 10kHz未満: `micros()` ベースのソフトウェアタイミング
- 10kHz以上: Timer1 CTCモード割り込みに自動切り替え
- RPMに応じたプリスケーラー自動選択（1/8/64/256/1024）

### 高速ADC処理

1. **フリーランモード**: 連続変換で高速サンプリング
2. **割り込み駆動**: `ISR(ADC_vect)` でメインループをブロックしない
3. **読み取り順序**: `ADCL` → `ADCH` の正しい順序で10bit値を取得
4. **RPM変換**: `(uint32_t)adc0 * MAX_RPM / 1023` でオーバーフロー回避

## 使用方法

### 1. 回路接続

```
Arduino Nano
├── A4 ← キルスイッチ（GND間）（内部プルアップ使用）
├── A5 ← スタータボタン（GND間）（内部プルアップ使用）
├── A0 ← 可変抵抗（中点）
├── D2 → A相出力
├── D5 → G相出力
├── D6 → スタートボタン出力（アクティブLow）
├── D7 → キルスイッチ出力（アクティブLow）
├── D8 → B相出力
└── D9 → Z相出力（インデックス）

可変抵抗: 10kΩ推奨
接続: VCC ─ 可変抵抗 ─ GND、中点(ワイパー) → A0
```

### 2. プログラム書き込み

1. VSCode + PlatformIOでビルド・書き込み
2. シリアルモニタを115200bpsで開く
3. 起動メッセージとコマンドヘルプを確認

### 3. 物理SW操作

1. キルスイッチ（A4）をGNDに接続してON
2. 可変抵抗(A0)を回して回転数を変更（0〜6500rpm）
3. スタータボタン（A5）をONにするとRPM=600rpm固定

### 4. シリアルコマンド操作

```
RPM:3000        → KILL=ON・START=OFFで3000rpm動作
KILL:OFF        → 停止
START:ON        → 始動状態を模擬。600rpmに
RESET           → 物理SW制御に戻す
```

## デバッグ出力

シリアルモニタに1秒間隔で以下の形式で表示：

```
KILL SW: ON, START SW: OFF, RPM: 3000, Angle: 254°, Pulse: 1019/2880, G: L, Mode: SER, Timer: SW, ADC: 582
```

| フィールド | 説明 |
| --------- | ---- |
| `KILL SW` | キルスイッチの実効状態（物理SW or シリアル上書き後） |
| `START SW` | スタートボタンの実効状態 |
| `RPM` | 現在の回転数 |
| `Angle` | G相周期内での現在角度（0〜719度） |
| `Pulse` | 現在のエッジカウント / G相周期の総エッジ数 |
| `G` | G相の現在状態（H/L） |
| `Mode` | `PHY`=物理SW制御、`SER`=シリアル制御 |
| `Timer` | `SW`=ソフトウェアタイミング、`HW`=Timer1 CTC |
| `ADC` | A0のADC生値（0〜1023） |

## カスタマイズ

### G相設定の変更

```cpp
// 例：3回転周期、720度開始、8エッジ幅に変更
const int G_PHASE_ROTATIONS = 3;
const int G_PHASE_START_DEG = 720;
const int G_PHASE_EDGES = 8;
```

### PPT設定の変更

```cpp
// 例：1024パルス/回転に変更
const int PULSES_PER_REV = 1024;
```

## 技術仕様

### タイミング精度

- **基準クロック**: Arduino 16MHz
- **最小パルス間隔**: 約9.2μs（6500rpm時のエッジ間隔）
- **最大回転数**: 6500rpm対応
- **ADC変換時間**: 約104μs（125KHzクロック）

### 信号仕様

- **出力レベル**: 0V/5V（TTLレベル）
- **立ち上がり/立ち下がり時間**: <1μs（レジスタ直接操作）
- **SW入力デバウンス**: 20ms

### 処理性能・最適化

- **整数演算のみ**: 浮動小数点演算を完全排除
- **レジスタ直接操作**: `digitalWrite()`比約10倍高速なIO制御
- **ADC処理**: 割り込み駆動フリーランモード、ADCL→ADCHの正しい読み取り順序
- **RPM計算**: `uint32_t`キャストによる16bitオーバーフロー回避
- **定数の事前計算**: G相開始・終了エッジをコンパイル時に計算
- **ノイズ低減**: `DIDR0`レジスタでA0デジタルバッファを無効化

## MA735との対応

| 項目 | MA735設定 | シミュレータ設定 |
| ---- | -------- | --------------- |
| PPT | 359 | PULSES_PER_REV = 360 |
| ILIP | 1111 | A相とB相が両方HIGH時にZ出力 |
| 分解能 | 1440エッジ/回転 | 360×4エッジ |

## 注意事項

1. **電源電圧**: Arduino Nanoの5V動作を前提
2. **可変抵抗接続**: VCC（5V）〜可変抵抗〜GND の両端接続が必要（片側だけでは範囲が狭くなる）
3. **タイミング精度**: `Serial.print()` の多用はISR処理タイミングに影響する場合あり
4. **ADCとデジタルSWの干渉**: A0〜A3をADCで使用中にA4/A5のデジタル読み取りにノイズが乗る場合は、A4/A5とGND間に0.1µFのパスコンを追加
5. **Timer1の競合**: `delay()`や `tone()`はTimer1またはTimer2を使用するため競合に注意

## 拡張機能

### 改良履歴

- [x] Ardu-Stimの高速・高精度信号制御・出力処理を導入
- [x] 浮動小数点演算の完全排除による高速化
- [x] レジスタ直接操作によるIO高速化
- [x] フリーランモードADC処理の実装
- [x] ADC読み取り順序修正（ADCL→ADCH）とRPM計算オーバーフロー対策
- [x] シリアルコマンドによる外部制御機能（KILL/START/RPM/STATUS/RESET）
- [x] 物理SWのデバウンス処理（20ms）
- [x] シリアルモード解除のエッジ検出（OFF→ONの瞬間のみ解除）
- [x] `DIDR0`によるADCノイズ低減

### 将来の改良案

- [ ] SPI通信でのリアルタイム設定変更
- [ ] EEPROM設定保存機能
- [ ] エラー検出機能

---

**作成日**: 2025年7月15日  
**バージョン**: 3.0  
**対象マイコン**: Arduino Nano (ATmega328P)  
**開発環境**: VSCode & PlatformIO  
**最終更新**: 2026年3月25日（シリアル制御・デバウンス・ADC修正）
