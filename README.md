# MA735エンコーダシミュレータ - Arduino実装

## 概要

MA735磁気式角度センサーのエンコーダ出力波形をArduino Nanoでシミュレートするプログラムです。  
A7ピンに接続した可変抵抗で0〜6500rpmの回転数を制御し、リアルタイムでABZ + G相の信号を出力します。  
また、A4, A5ピンでキルスイッチ、スタートボタンの入力を受け取り、D6, D7ピンでそれぞれ信号を出力します。

## 仕様

### ハードウェア要件

- **マイコン**: Arduino Nano(Atmega328P)
- **入力**:
  - A4: キルスイッチ(内部プルアップ)
  - A5: スタートボタン(内部プルアップ)
  - A7ピン 可変抵抗（0〜5V ADC入力）
- **出力**:
  - D2: A相出力
  - D5: G相出力
  - D6: スタートボタン出力
  - D7: キルスイッチ出力
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
- **パルス幅**: 720エッジ分(4エッジ/1度)
- **設定変更**: 変数で動的に変更可能

## プログラム構成

### 主要変数

```cpp
// エンコーダ仕様（整数型で高速化）
const int PULSES_PER_REV = 360;   // 1回転あたりのパルス数
const int MAX_RPM = 6500;         // 最大回転数
const int START_RPM = 600;        // 始動時の回転数

// G相設定（変更可能）
const int G_PHASE_ROTATIONS = 2;      // G相の出力周期（回転数）
const int G_PHASE_START_DEG = 450;    // G相開始角度（度）
const int G_PHASE_EDGES = 720;        // G相出力エッジ数

// 高速計算用定数（浮動小数点演算を回避）
const int TOTAL_EDGES_PER_REV = PULSES_PER_REV * 4;  // 1回転あたりの総エッジ数
const int TOTAL_EDGES_PER_CYCLE = TOTAL_EDGES_PER_REV * G_PHASE_ROTATIONS; // G相周期の総エッジ数
const int G_PHASE_START_EDGES = (G_PHASE_START_DEG * TOTAL_EDGES_PER_REV) / 360; // G相開始エッジ
const int G_PHASE_END_EDGES = G_PHASE_START_EDGES + G_PHASE_EDGES; // G相終了エッジ

// 高速ADC処理用変数
volatile uint16_t adc0 = 0;              // ADC0読み取り値
volatile bool adc0_read_complete = false; // ADC完了フラグ
volatile uint8_t analog_port = 0;        // 現在のADCポート

// ADC・RPM変換定数（Ardu-Stim方式）
#define TMP_RPM_SHIFT 4    // x16倍のビットシフト
#define TMP_RPM_CAP 9000   // 最大RPM制限
```

### ピン定義

```cpp
const int POT_PIN = A7;        // 可変抵抗
const int A_PHASE_PIN = 2;     // A相出力
const int B_PHASE_PIN = 8;     // B相出力
const int Z_PHASE_PIN = 9;     // Z相出力
const int G_PHASE_PIN = 5;     // G相出力
const int KILL_OUT_PIN  = 7;   // キルスイッチ出力
const int START_OUT_PIN = 6;   // スタータボタン出力
const int KILL_IN_PIN   = A4;  // キルスイッチ入力
const int START_IN_PIN  = A5;  // スタータボタン入力
```

### 主要機能

#### 1. setup()

- シリアル通信初期化（115200bps）
- ピン設定（出力モード）
- 初期状態設定
- 設定内容の表示

#### 2. loop()

- 入力状態読み取り
- 入力状態で条件分岐
  - キルスイッチがONの場合:
    - スタートボタンがONの場合: 回転数を始動時回転数に設定
    - スタートボタンがOFFの場合: 可変抵抗から回転数読み取り
  - キルスイッチがOFFの場合: 回転数を0rpmに設定
- パルス間隔計算
- タイミング制御によるパルス生成
- デバッグ情報出力（1秒間隔）

#### 3. generatePulse()

- パルスカウンター管理
- A相/B相の90度位相差生成
- Z相インデックス生成（ILIP=1111設定）
- G相特殊パターン生成（整数演算のみ）
- 出力ピン制御（レジスタ直接操作）

#### 4. setupHighSpeedADC()

- ADCレジスタ直接制御
- フリーランモード設定
- 125KHz ADCクロック設定
- 割り込み有効化

## 動作原理

### ABZ信号生成

1. **A相**: 基本方形波（360パルス/回転）
2. **B相**: A相より90度遅れた方形波
3. **Z相**: 1回転に1回、A相とB相が両方HIGHの期間に出力（ILIP=1111）

### G相信号生成

1. **周期計算**: 2回転分（2880エッジ）でカウンターリセット
2. **エッジ位置計算**: 整数演算のみで現在のエッジ位置を算出
3. **出力判定**: 事前計算された開始・終了エッジ値を使用してHIGH出力

### タイミング制御

- マイクロ秒精度のタイミング制御
- RPMに応じたパルス間隔の動的計算（整数演算）
- 高速ADC処理による可変抵抗値の読み取り
- 10kHz以上での自動ハードウェアタイマー切り替え

### 高速ADC処理

1. **フリーランモード**: 連続サンプリングで高速化
2. **割り込み駆動**: メインループをブロックしない非同期処理
3. **ビットシフト変換**: 浮動小数点演算を回避した高速RPM変換

## 使用方法

### 1. 回路接続

```
Arduino Nano
├── A4 ← キルスイッチ入力（内部プルアップ）
├── A5 ← スタータボタン入力（内部プルアップ）
├── A7 ← 可変抵抗（中点）
├── D2 → A相出力
├── D8 → B相出力
├── D9 → Z相出力
└── D5 → G相出力

可変抵抗: 10kΩ推奨
接続: VCC - 可変抵抗 - GND
      A7 ← 可変抵抗中点
```

### 2. プログラム書き込み

1. Arduino IDEでスケッチをArduino Nanoに書き込み
2. シリアルモニタを115200bpsで開く
3. 初期化メッセージと設定確認

### 3. 動作確認

1. 可変抵抗を回して回転数を変更
2. オシロスコープでD2,D8,D9,D5の波形を観測
3. シリアルモニタでデバッグ情報を確認

## デバッグ出力

シリアルモニタに1秒間隔で以下の情報を表示：

```
RPM: 1500.0, Angle: 234.5°, Pulse: 1250/2880, G: L
```

- **RPM**: 現在の回転数
- **Angle**: G相周期内での現在角度
- **Pulse**: 現在のパルスカウント/総パルス数
- **G**: G相の現在状態（H/L）

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
- **最小パルス間隔**: 12.8μs（6500rpmのA/B相パルス）
- **最大回転数**: 6500rpm対応
- **ADC変換時間**: 約104μs（125KHzクロック）

### 信号仕様

- **出力レベル**: 0V/5V（TTLレベル）
- **立ち上がり/立ち下がり時間**: <1μs（レジスタ直接操作）
- **位相精度**: ±1エッジ以内

### 処理性能

- **計算方式**: 整数演算のみ（浮動小数点演算なし）
- **IO処理**: レジスタ直接操作（digitalWrite比約10倍高速）
- **ADC処理**: 割り込み駆動フリーランモード
- **メモリ効率**: volatile変数の適切な使用

### ハードウェアタイマーCTCモードの活用（`.\Ardu-Stim\ardustim\ardustim`を参照）

**核心技術：**

- Timer 1と2をCTC (Clear Timer on Compare) モードで使用
- CTCタイマー割り込みは、カウンターが指定された値（比較マッチレジスタに格納）に達したときにトリガーされ、次のタイマークロックティックでクリア（ゼロにリセット）される

**計算式：**

```
割り込み周波数 (Hz) = 16,000,000Hz / (プリスケーラー × (比較マッチレジスタ + 1))
比較マッチレジスタ = [16,000,000Hz / (プリスケーラー × 希望周波数)] - 1
```

**基本セットアップ（例）：**

```cpp
// Timer1 CTC モード設定例
void setupHighPrecisionTimer() {
    noInterrupts();  // 割り込み無効化
    
    // タイマーレジスタ初期化
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // 比較マッチレジスタ設定（例：1kHz）
    OCR1A = 15624;  // 16MHz/1024/1Hz
    
    // CTCモード設定
    TCCR1B |= (1 << WGM12);
    
    // プリスケーラー設定（1024）
    TCCR1B |= (1 << CS12) | (1 << CS10);
    
    // 比較マッチ割り込み有効化
    TIMSK1 |= (1 << OCIE1A);
    
    interrupts();  // 割り込み有効化
}

ISR(TIMER1_COMPA_vect) {
    // 高精度タイミングで実行される処理
}
```

### プリスケーラー選択の最適化（`.\Ardu-Stim\ardustim\ardustim`を参照）

**選択指針：**

- より大きなプリスケーラー値は、出力周波数が低くなり、消費電力がc v²f に比例するため、わずかに消費電力が減少する
- 低いプリスケーラーはターゲットに近づけるが、CTCが0に近い場合（1-10）は、タイマーが異常動作することがある

**推奨設定：**

| 目標周波数 | 推奨プリスケーラー | 推奨OCR値範囲 |
|------------|-------------------|---------------|
| 1Hz - 100Hz | 1024 | 150-15000 |
| 100Hz - 1kHz | 256 | 60-600 |
| 1kHz - 10kHz | 64 | 25-250 |
| 10kHz+ | 8-64 | 20-200 |

### 複数信号同期出力の実装（`.\Ardu-Stim\ardustim\ardustim`を参照）

**同期信号生成のコツ（例）：**

```cpp
volatile bool signal1_state = false;
volatile bool signal2_state = false;
volatile uint16_t signal_counter = 0;

ISR(TIMER1_COMPA_vect) {
    signal_counter++;
    
    // 信号1: 基本周波数
    if (signal_counter % 2 == 0) {
        signal1_state = !signal1_state;
        digitalWrite(8, signal1_state);
    }
    
    // 信号2: 1/4周波数
    if (signal_counter % 8 == 0) {
        signal2_state = !signal2_state;
        digitalWrite(9, signal2_state);
    }
}
```

### 高速動作のための最適化テクニック（`.\Ardu-Stim\ardustim\ardustim`を参照）

**レジスタ直接操作（実装済み）：**

```cpp
// digitalWrite()の代わりにレジスタ直接操作
// A相（D2 = PD2）の場合
if (aState) {
  PORTD |= (1 << PD2);   // HIGH
} else {
  PORTD &= ~(1 << PD2);  // LOW
}

// 入力読み取り（A4 = PC4）
int KILL_SW_State = (PINC & (1 << PC4)) ? swOFF : swON;
```

**整数演算による高速化（実装済み）：**

```cpp
// 浮動小数点演算を回避
uint32_t pulsesPerSecond = (currentRPM * PULSES_PER_REV) / 60;
tmp_rpm = adc0 << TMP_RPM_SHIFT;  // ビットシフト演算
currentRPM = (tmp_rpm * MAX_RPM) / 16368;  // 整数除算
```

**高速ADC処理（実装済み）：**

```cpp
// フリーランモード設定
ADCSRA |= B00100000;       // ADATE=1（オートトリガー有効）
ADCSRB &= B11111000;       // ADTS2:0=000（フリーランモード）
ADCSRA |= B00000111;       // プリスケーラー128（125KHz）
```

**メモリ効率化（実装済み）：**

- volatileキーワードの適切な使用
- 事前計算された定数の使用
- 整数型変数の使用

**タイマー競合の回避：**

- delay()、millis()、analogWrite()との競合を避ける
- 使用するタイマーを明確に管理する

**割り込み処理の原則：**

- ISRは可能な限り短く保つ
- 複雑な処理は主ループで実行
- volatile変数の適切な使用

**デバッグ時の注意：**

- Serial.print()をISR内で使用しない
- LEDやピンの状態確認でデバッグ

## MA735との対応

| 項目 | MA735設定 | シミュレータ設定 |
|------|-----------|------------------|
| PPT | 359 | PULSES_PER_REV = 360 |
| ILIP | 1111 | A&B両方HIGH時にZ出力 |
| 分解能 | 1440エッジ/回転 | 360×4エッジ |

## 注意事項

1. **電源電圧**: Arduino Nanoの5V動作を前提
2. **負荷容量**: 出力ピンの駆動能力（最大40mA）を考慮
3. **タイミング精度**: 他の割り込み処理による影響を最小化
4. **シリアル通信**: デバッグ出力が不要な場合はコメントアウト推奨
5. **メモリ使用量**: 整数演算とレジスタ直接操作により大幅削減
6. **処理速度**: 浮動小数点演算を完全に排除し高速化を実現

## 拡張機能

### 将来の改良案

- [x] Ardu-Stimの高速・高精度信号制御・出力処理を導入（完了）
- [x] 浮動小数点演算の完全排除による高速化（完了）
- [x] レジスタ直接操作によるIO高速化（完了）
- [x] フリーランモードADC処理の実装（完了）
- [ ] SPI通信でのリアルタイム設定変更
- [ ] PWM出力の追加実装
- [ ] エラー検出機能
- [ ] 温度補償機能
- [ ] EEPROM設定保存機能

---

**作成日**: 2025年7月15日  
**バージョン**: 2.0  
**対象マイコン**: Arduino Nano (ATmega328P)  
**開発環境**: VSCode & PlatformIO  
**最終更新**: 2025年7月15日（高速化実装完了）
