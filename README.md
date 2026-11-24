# MA735エンコーダシミュレータ - Arduino実装

## 概要

MA735磁気式角度センサーのエンコーダ出力波形を Arduino Nano でシミュレートするプログラムです。  
A0ピンに接続した可変抵抗で 0〜6500rpm の回転数を制御し、リアルタイムで **A / Z / G相** を出力します。  
また、A4/A5ピンでキルスイッチ・スタートボタン入力を受け取り、D6/D5ピンでそれぞれアクティブLow出力します。  
シリアルコマンドで外部から状態を上書きし、物理スイッチ押下（OFF→ONエッジ）で物理制御へ戻せます。

## 仕様

### ハードウェア要件

- **マイコン**: Arduino Nano (ATmega328P)
- **入力**:
  - A4: キルスイッチ（内部プルアップ、GND接続でON）
  - A5: スタートボタン（内部プルアップ、GND接続でON）
  - A0: 可変抵抗（0〜5V ADC入力）
- **出力**:
  - D5: スタートボタン出力（アクティブLow）
  - D6: キルスイッチ出力（アクティブLow）
  - D8: A相出力
  - D9: Z相出力（インデックス）
  - D10: G相出力
  - B相: **出力なし**（ECU側で常時GND接続）

### エンコーダ設定

- **PPT（Pulses per Turn）**: 359（360パルス/回転）
- **分解能**: 1440エッジ/回転（360パルス × 4）
- **最大回転数**: 6500rpm
- **始動時回転数**: 600rpm

### G相設定

- **出力周期**: 2回転に1パルス
- **開始角度**: 450度（1.5回転目）
- **パルス幅**: 720エッジ

## プログラム構成

### 主要変数

```cpp
const int PULSES_PER_REV = 360;
const int MAX_RPM = 6500;
const int START_RPM = 600;

const int G_PHASE_ROTATIONS = 2;
const int G_PHASE_START_DEG = 450;
const int G_PHASE_EDGES = 720;

const int TOTAL_EDGES_PER_REV = PULSES_PER_REV * 4;
const int TOTAL_EDGES_PER_CYCLE = TOTAL_EDGES_PER_REV * G_PHASE_ROTATIONS;
const int G_PHASE_START_EDGES = (int)((int32_t)G_PHASE_START_DEG * TOTAL_EDGES_PER_REV / 360);
const int G_PHASE_END_EDGES = G_PHASE_START_EDGES + G_PHASE_EDGES;

volatile uint16_t adc0 = 0;

bool serialControlMode = false;
bool serialKillSW = false;
bool serialStartSW = false;
uint16_t serialRPM = 0;
```

### ピン定義

```cpp
const int POT_PIN       = A0;
const int KILL_IN_PIN   = A4;
const int START_IN_PIN  = A5;
const int A_PHASE_PIN   = 8;
const int Z_PHASE_PIN   = 9;
const int G_PHASE_PIN   = 10;
const int KILL_OUT_PIN  = 6;
const int START_OUT_PIN = 5;
```

### 主要機能

#### 1. setup()

- シリアル通信初期化（115200bps）
- ピン設定
- 起動メッセージとコマンドヘルプ表示

#### 2. loop()

- シリアル入力チェック（非ブロッキング）
- SW状態読み取り（デバウンス20ms）
- 物理SWのOFF→ONエッジでシリアルモード解除
- シリアルモード時は `serialKillSW` / `serialStartSW` でSW状態上書き
- 出力制御ロジック:
  - KILL ON + START ON  → D6=LOW, D5=LOW, RPM=600
  - KILL ON + START OFF → D6=LOW, D5=HIGH, RPMは可変抵抗またはシリアル値
  - KILL OFF            → D6=HIGH, D5=HIGH, RPM=0
- 10kHz超で Timer1 CTC へ自動切替

#### 3. generatePulse()

- 4エッジで1パルス（360PPR）
- A相生成（D8）
- Z相生成（D9、1回転に1回）
- G相生成（D10、2回転周期）
- レジスタ直接操作で高速出力

#### 4. checkSerialInput() / processSerialCommand()

- 32バイト固定長バッファ
- 改行(`\n`/`\r`)でコマンド確定
- `strcmp`/`strncmp`/`atoi`で軽量処理

## シリアルコマンド

| コマンド | 動作 |
| ------- | ---- |
| `KILL:ON` | KILL SW = ON として扱う（D6=LOW） |
| `KILL:OFF` | KILL SW = OFF として扱う（D6=HIGH、RPM=0） |
| `START:ON` | START SW = ON として扱う（D5=LOW、RPM=600） |
| `START:OFF` | START SW = OFF として扱う（D5=HIGH） |
| `RPM:0-6500` | KILL=ON・START=OFFを自動設定し指定RPMで動作 |
| `STATUS` | 現在の状態を表示 |
| `RESET` | シリアル制御モードを解除し物理SW制御に戻る |

### シリアル制御モード

- いずれかのコマンド受信で **SERモード** に移行
- 以下で **PHYモード** に戻る:
  - A4（KILL）押下のOFF→ONエッジ
  - A5（START）押下のOFF→ONエッジ
  - `RESET`コマンド
  - MCUリセット

## 動作原理

### A/Z/G信号生成

1. **A相**: 360パルス/回転（D8）
2. **Z相**: 1回転に1回のインデックス（D9）
3. **G相**: 2回転周期のゲート信号（D10）

### タイミング制御

- 低周波域: `micros()` ベース
- 高周波域: Timer1 CTC割り込み
- RPMに応じたプリスケーラー自動選択（1/8/64/256/1024）

### A0（可変抵抗）読み取り

- `analogRead(A0)` で取得
- RPM変換: `(uint32_t)adc0 * MAX_RPM / 1023`

## 使用方法

### 1. 回路接続

```
Arduino Nano
├── A4  ← キルスイッチ（GND間、内部プルアップ）
├── A5  ← スタートボタン（GND間、内部プルアップ）
├── A0  ← 可変抵抗（中点）
├── D5  → スタートボタン出力（アクティブLow）
├── D6  → キルスイッチ出力（アクティブLow）
├── D8  → A相出力
├── D9  → Z相出力
├── D10 → G相出力
└── B相はECU側でGNDへ固定接続

可変抵抗: 10kΩ推奨
接続: VCC ─ 可変抵抗 ─ GND、中点(ワイパー) → A0
```

### 2. 書き込み

1. VSCode + PlatformIOでビルド・書き込み
2. シリアルモニタを115200bpsで開く
3. 起動メッセージとコマンドヘルプを確認

### 3. 物理操作

1. A4（KILL）をGNDへ接続してON
2. A0可変抵抗で回転数を変更（0〜6500rpm）
3. A5（START）ONでRPM=600固定

### 4. シリアル操作

```
RPM:3000
KILL:OFF
START:ON
RESET
```

## デバッグ出力

1秒間隔で以下形式を出力:

```
KILL SW: ON, START SW: OFF, RPM: 3000, Angle: 254, Pulse: 1019/2880, G: L, Mode: SER, Timer: HW, ADC: 582
```

| フィールド | 説明 |
| --------- | ---- |
| `KILL SW` | キルスイッチ実効状態 |
| `START SW` | スタートボタン実効状態 |
| `RPM` | 現在回転数 |
| `Angle` | G相周期内の角度相当値 |
| `Pulse` | 現在エッジカウント / 周期総エッジ |
| `G` | G相状態（H/L） |
| `Mode` | `PHY`=物理制御、`SER`=シリアル制御 |
| `Timer` | `SW`=ソフトウェア、`HW`=Timer1 CTC |
| `ADC` | A0のADC生値（0〜1023） |

## 技術仕様

### タイミング

- 基準クロック: 16MHz
- 最大回転数: 6500rpm
- 10kHz超でハードウェアタイマーへ切替

### 信号仕様

- 出力レベル: 0V/5V（TTL）
- SW入力デバウンス: 20ms

### 実装上の最適化

- 浮動小数点演算なし（整数演算）
- レジスタ直接操作で高速IO
- 定数を事前計算（G相開始/終了エッジ）

## MA735との対応

| 項目 | シミュレータ設定 |
| ---- | --------------- |
| PPT | 359相当（360パルス/回転） |
| 分解能 | 1440エッジ/回転 |
| 出力相 | A/Z/G（B相は未出力） |

## 注意事項

1. Arduino Nanoの5V動作を前提
2. 可変抵抗はVCC-GND間に接続し、中点をA0へ接続
3. `Serial.print()` の過多は高RPM時のタイミングに影響する場合あり
4. `delay()` や `tone()` などTimer使用機能との競合に注意

---

**作成日**: 2025年7月15日  
**バージョン**: 3.1  
**対象マイコン**: Arduino Nano (ATmega328P)  
**開発環境**: VSCode & PlatformIO  
**最終更新**: 2026年3月30日（現行実装へ整合）
