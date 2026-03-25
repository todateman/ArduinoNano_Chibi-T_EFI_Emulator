#include <Arduino.h>

// SW状態定義
const int swON = LOW;    // SWがONのピンの値はLOW
const int swOFF = HIGH;  // SWがOFFのピンの値はHIGH

// ピン定義
const int KILL_IN_PIN = A4;    // キルスイッチ入力（内部プルアップ）
const int START_IN_PIN  = A5;  // スタータボタン入力（内部プルアップ）
const int POT_PIN = A0;        // 可変抵抗
const int A_PHASE_PIN = 2;     // A相出力
const int B_PHASE_PIN = 8;     // B相出力
const int Z_PHASE_PIN = 9;     // Z相出力
const int G_PHASE_PIN = 5;     // G相出力
const int KILL_OUT_PIN  = 7;   // キルスイッチ出力
const int START_OUT_PIN = 6;   // スタータボタン出力

// エンコーダ仕様（MA735互換）
const int PULSES_PER_REV = 360;   // 1回転あたりのパルス数（PPT=359+1）
const int MAX_RPM = 6500;         // 最大回転数
const int START_RPM = 600;        // 始動時の回転数

// G相設定（変更可能）
const int G_PHASE_ROTATIONS = 2;  // G相の出力周期（回転数）
const int G_PHASE_START_DEG = 450; // G相開始角度（度）
const int G_PHASE_EDGES = 720;      // G相出力エッジ数(4エッジ/1度))

// 高速計算用定数
const int TOTAL_EDGES_PER_REV = PULSES_PER_REV * 4;  // 1回転あたりの総エッジ数
const int TOTAL_EDGES_PER_CYCLE = TOTAL_EDGES_PER_REV * G_PHASE_ROTATIONS; // G相周期の総エッジ数
const int G_PHASE_START_EDGES = (int)((int32_t)G_PHASE_START_DEG * TOTAL_EDGES_PER_REV / 360); // G相開始エッジ
const int G_PHASE_END_EDGES = G_PHASE_START_EDGES + G_PHASE_EDGES; // G相終了エッジ

// 高速・高精度制御用タイマー変数
volatile bool useHighPrecisionTimer = false;
volatile unsigned long timerMicros = 0;
volatile bool timerPulseFlag = false;

// 高速ADC処理用変数
volatile uint16_t adc0 = 0;              // ADC0読み取り値
volatile bool adc0_read_complete = false; // ADC完了フラグ
volatile uint8_t analog_port = 0;        // 現在のADCポート

// ADC・RPM変換定数（Ardu-Stim方式）
#define TMP_RPM_SHIFT 4    // x16倍のビットシフト
#define TMP_RPM_CAP 9000   // 最大RPM制限

// 変数
unsigned long lastMicros = 0;   // 最後のパルス生成時間
unsigned long pulseInterval;  // パルス間隔（マイクロ秒）
uint16_t currentRPM = 0;        // 現在の回転数（RPM）整数型
uint16_t tmp_rpm = 0;          // 一時的RPM値（整数演算用）
volatile int pulseCount = 0;             // パルスカウント
volatile bool aState = false;            // A相状態
volatile bool bState = false;            // B相状態
volatile bool zState = false;            // Z相状態
volatile bool gState = false;            // G相状態

// デバッグ用
unsigned long lastDebugTime = 0;    // 最後のデバッグ出力時間
const unsigned long DEBUG_INTERVAL = 1000; // 1秒間隔

// シリアル制御モード
bool serialControlMode = false;
bool serialKillSW = false;    // シリアルモードでのKILL SW上書き（true=ON）
bool serialStartSW = false;   // シリアルモードでのSTART SW上書き（true=ON）
uint16_t serialRPM = 0;       // シリアルモードでのRPM

// シリアル受信バッファ（非ブロッキング方式）
char serialBuffer[32];
uint8_t serialBufPos = 0;

// SW前回状態（シリアルモード解除のエッジ検出用）
int prevKillSWState = swOFF;
int prevStartSWState = swOFF;

// SWデバウンス用変数
const unsigned long SW_DEBOUNCE_MS = 20;  // 20msデバウンス
unsigned long killDebounceTimer = 0;
unsigned long startDebounceTimer = 0;
int rawKillSWState_prev  = swOFF;
int rawStartSWState_prev = swOFF;
int debouncedKillState  = swOFF;
int debouncedStartState = swOFF;

// プロトタイプ宣言
void generatePulse();           // パルス生成関数
void setupHighPrecisionTimer(); // 高精度タイマー設定
void setTimerFrequency(uint32_t frequency); // タイマー周波数設定
void setupHighSpeedADC();       // 高速ADC設定
void checkSerialInput();        // シリアル入力チェック（非ブロッキング）
void processSerialCommand(const char* cmd); // シリアルコマンド処理

// パルス生成関数
// この関数は、エンコーダのパルスを生成し、A相、B相、Z相、G相の状態を更新します。
// G相は指定された周期で出力され、開始角度とエッジ数に基づいて制御されます。
void generatePulse() {
  // パルスカウントを増加
  pulseCount++;
  
  // G相の周期を考慮したカウンターリセット（事前計算した定数使用）
  if (pulseCount >= TOTAL_EDGES_PER_CYCLE) {
    pulseCount = 0;
  }
  
  // 現在の位置（1回転内）
  int positionInRev = pulseCount % TOTAL_EDGES_PER_REV;
  
  // A相とB相の状態を決定（90度位相差）
  bool newAState = (positionInRev % (PULSES_PER_REV * 2)) < PULSES_PER_REV;
  
  // B相の状態（A相より90度遅れ）
  int bPosition = (positionInRev + PULSES_PER_REV) % TOTAL_EDGES_PER_REV;
  bool newBState = (bPosition % (PULSES_PER_REV * 2)) < PULSES_PER_REV;
  
  // Z相（インデックス）の状態
  // ILIP=1111の場合：A相とB相が両方HIGHの時にZ相もHIGH
  bool newZState = false;
  if (positionInRev < (PULSES_PER_REV / 2)) {  // 1回転の最初の部分
    if (newAState && newBState) {  // A相とB相が両方HIGH
      newZState = true;
    }
  }
  
  // G相の状態計算（整数演算のみ）
  bool newGState = false;
  
  // G相の周期内でのエッジ位置
  int edgeInCycle = pulseCount % TOTAL_EDGES_PER_CYCLE;
  
  // G相の出力判定（事前計算した定数使用）
  if (edgeInCycle >= G_PHASE_START_EDGES && edgeInCycle < G_PHASE_END_EDGES) {
    newGState = true;
  }
  
  // 状態を更新
  aState = newAState;
  bState = newBState;
  zState = newZState;
  gState = newGState;
  
  // 出力を更新（高速化のためレジスタ直接操作）
  // A相（D2 = PD2）
  if (aState) {
    PORTD |= (1 << PD2);
  } else {
    PORTD &= ~(1 << PD2);
  }
  
  // B相（D8 = PB0）
  if (bState) {
    PORTB |= (1 << PB0);
  } else {
    PORTB &= ~(1 << PB0);
  }
  
  // Z相（D9 = PB1）
  if (zState) {
    PORTB |= (1 << PB1);
  } else {
    PORTB &= ~(1 << PB1);
  }
  
  // G相（D5 = PD5）
  if (gState) {
    PORTD |= (1 << PD5);
  } else {
    PORTD &= ~(1 << PD5);
  }
}

void setup() {
  Serial.begin(115200);
  
  // ピン設定
  pinMode(A_PHASE_PIN, OUTPUT); // A相出力
  pinMode(B_PHASE_PIN, OUTPUT); // B相出力
  pinMode(Z_PHASE_PIN, OUTPUT); // Z相出力
  pinMode(G_PHASE_PIN, OUTPUT); // G相出力
  pinMode(KILL_OUT_PIN, OUTPUT);  // キルスイッチ出力
  pinMode(START_OUT_PIN, OUTPUT); // スタータボタン出力
  pinMode(KILL_IN_PIN, INPUT_PULLUP);  // キルスイッチ入力（プルアップ）
  pinMode(START_IN_PIN, INPUT_PULLUP); // スタータボタン入力（プルアップ）
    
  // 初期状態（レジスタ直接操作）
  // D2(A相), D5(G相) = LOW
  PORTD &= ~((1 << PD2) | (1 << PD5));
  // D8(B相), D9(Z相) = LOW
  PORTB &= ~((1 << PB0) | (1 << PB1));
  // D6(スタート出力), D7(キル出力) = LOW
  PORTD &= ~((1 << PD6) | (1 << PD7));
  
  // 高速ADC初期化
  setupHighSpeedADC();
  
  Serial.println("MA735 Encoder Simulator Started");
  Serial.println("PPT=359 (360 pulses/turn), ILIP=1111");
  Serial.print("G Phase: ");
  Serial.print(G_PHASE_ROTATIONS);
  Serial.print(" rotations cycle, starts at ");
  Serial.print(G_PHASE_START_DEG);
  Serial.print("°, ");
  Serial.print(G_PHASE_EDGES);
  Serial.println(" edges");
  Serial.println("A0: Speed Control (0-6500 RPM) - High Speed ADC");
  Serial.println("D2: A Phase, D8: B Phase, D9: Z Phase, D5: G Phase");
  Serial.println("A4: Kill Switch Input, A5: Start Button Input");
  Serial.println("D6: Start Button Output, D7: Kill Switch Output");
  Serial.println("ADC: Free-running mode, 125KHz clock, interrupt-driven");
  Serial.println("--- Serial Commands ---");
  Serial.println("KILL:ON/OFF  - Kill switch output");
  Serial.println("START:ON/OFF - Start button output");
  Serial.println("RPM:0-6500   - Set RPM");
  Serial.println("STATUS       - Show current state");
  Serial.println("RESET        - Exit serial control mode");
  Serial.println("Serial mode is active until physical switch pressed or MCU reset");
}

void loop() {
  // シリアル入力処理（非ブロッキング）
  checkSerialInput();

  // SWの状態を確認（デバウンス処理付き）
  // A4(KILL_IN) = PC4, A5(START_IN) = PC5
  int rawKill  = (PINC & (1 << PC4)) ? swOFF : swON;
  int rawStart = (PINC & (1 << PC5)) ? swOFF : swON;
  unsigned long nowMs = millis();

  if (rawKill != rawKillSWState_prev) {
    rawKillSWState_prev = rawKill;
    killDebounceTimer = nowMs;
  }
  if (nowMs - killDebounceTimer >= SW_DEBOUNCE_MS) {
    debouncedKillState = rawKill;
  }
  int KILL_SW_State = debouncedKillState;

  if (rawStart != rawStartSWState_prev) {
    rawStartSWState_prev = rawStart;
    startDebounceTimer = nowMs;
  }
  if (nowMs - startDebounceTimer >= SW_DEBOUNCE_MS) {
    debouncedStartState = rawStart;
  }
  int START_SW_State = debouncedStartState;

  // 物理SWが新たに押された瞬間（OFF→ONのエッジ）でシリアルモードを解除
  if (serialControlMode) {
    if ((prevKillSWState == swOFF && KILL_SW_State == swON) ||
        (prevStartSWState == swOFF && START_SW_State == swON)) {
      serialControlMode = false;
      Serial.println("[Serial mode exited by physical switch]");
    }
  }
  prevKillSWState = KILL_SW_State;
  prevStartSWState = START_SW_State;

  if (serialControlMode) {
    // シリアルモードの場合、SW入力を上書き
    KILL_SW_State  = serialKillSW  ? swON : swOFF;
    START_SW_State = serialStartSW ? swON : swOFF;
  }

  // SW状態に基づく出力制御（シリアル・物理共通ロジック）
  if (KILL_SW_State == swON) {  // キルスイッチがON
    // D7(キル出力) = LOW
    PORTD &= ~(1 << PD7);

    if (START_SW_State == swON) {  // スタータボタンがON
      // D6(スタート出力) = LOW
      PORTD &= ~(1 << PD6);
      currentRPM = START_RPM; // 始動時回転数
    } else {                        // スタータボタンがOFF
      // D6(スタート出力) = HIGH
      PORTD |= (1 << PD6);
      if (serialControlMode) {
        currentRPM = serialRPM;   // シリアル指定RPM
      } else if (adc0_read_complete) {
        adc0_read_complete = false;
        // 直接RPMに変換（ADC 0-1023 → RPM 0-6500）uint32_tでオーバーフロー回避
        currentRPM = (uint16_t)(((uint32_t)adc0 * MAX_RPM) / 1023);
      }
    }
  } else {                      // キルスイッチがOFF
    // D7(キル出力) = HIGH
    PORTD |= (1 << PD7);

    // パルス生成を停止
    currentRPM = 0; // 停止状態
  }
  
  // パルス間隔を計算（マイクロ秒）整数演算
  if (currentRPM > 0) {
    // pulsesPerSecond = (currentRPM / 60) * PULSES_PER_REV
    // 整数演算：pulsesPerSecond = (currentRPM * PULSES_PER_REV) / 60
    uint32_t pulsesPerSecond = (currentRPM * PULSES_PER_REV) / 60;
    pulseInterval = 1000000UL / pulsesPerSecond; // マイクロ秒
    
    // 高速動作時（10kHz以上）はハードウェアタイマーを使用
    if (pulsesPerSecond > 10000) {
      if (!useHighPrecisionTimer) {
        setupHighPrecisionTimer();
        useHighPrecisionTimer = true;
      }
      setTimerFrequency(pulsesPerSecond);
    } else {
      if (useHighPrecisionTimer) {
        // タイマーを無効化
        TIMSK1 &= ~(1 << OCIE1A);
        useHighPrecisionTimer = false;
      }
    }
  } else {
    pulseInterval = 0; // 停止
    if (useHighPrecisionTimer) {
      TIMSK1 &= ~(1 << OCIE1A);
      useHighPrecisionTimer = false;
    }
  }

  // パルス生成（タイマー使用時は割り込みで処理）
  if (useHighPrecisionTimer) {
    if (timerPulseFlag) {
      generatePulse();
      timerPulseFlag = false;
    }
  } else {
    unsigned long currentMicros = micros();
    if (pulseInterval > 0 && (currentMicros - lastMicros >= pulseInterval)) {
      generatePulse();
      lastMicros = currentMicros;
    }
  }
  
  // デバッグ情報出力
  if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
    // 整数演算で角度計算
    int angleInCycle = (pulseCount % TOTAL_EDGES_PER_CYCLE * 360) / TOTAL_EDGES_PER_REV;
    
    Serial.print("KILL SW: ");
    Serial.print(KILL_SW_State == swON ? "ON" : "OFF");
    Serial.print(", START SW: ");
    Serial.print(START_SW_State == swON ? "ON" : "OFF");
    Serial.print(", RPM: ");
    Serial.print(currentRPM);
    Serial.print(", Angle: ");
    Serial.print(angleInCycle);
    Serial.print("°, Pulse: ");
    Serial.print(pulseCount);
    Serial.print("/");
    Serial.print(TOTAL_EDGES_PER_CYCLE);
    Serial.print(", G: ");
    Serial.print(gState ? "H" : "L");
    Serial.print(", Mode: ");
    Serial.print(serialControlMode ? "SER" : "PHY");
    Serial.print(", Timer: ");
    Serial.print(useHighPrecisionTimer ? "HW" : "SW");
    Serial.print(", ADC: ");
    Serial.print(adc0);
    Serial.println();
    lastDebugTime = millis();
  }
}

// 高精度タイマー設定関数
void setupHighPrecisionTimer() {
  noInterrupts();
  
  // タイマーレジスタ初期化
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // CTCモード設定
  TCCR1B |= (1 << WGM12);
  
  // 初期プリスケーラー設定（64）
  TCCR1B |= (1 << CS11) | (1 << CS10);
  
  // 比較マッチ割り込み有効化
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();
}

// タイマー周波数設定関数（整数演算版）
void setTimerFrequency(uint32_t frequency) {
  noInterrupts();
  
  // 16MHz基準クロック
  uint32_t clockFreq = 16000000UL;
  uint32_t targetCompare;
  
  // 最適なプリスケーラーを選択
  if (frequency > 62500) {
    // プリスケーラー 1
    TCCR1B = (TCCR1B & 0xF8) | (1 << CS10);
    targetCompare = clockFreq / frequency - 1;
  } else if (frequency > 7812) {
    // プリスケーラー 8
    TCCR1B = (TCCR1B & 0xF8) | (1 << CS11);
    targetCompare = clockFreq / (8 * frequency) - 1;
  } else if (frequency > 976) {
    // プリスケーラー 64
    TCCR1B = (TCCR1B & 0xF8) | (1 << CS11) | (1 << CS10);
    targetCompare = clockFreq / (64 * frequency) - 1;
  } else if (frequency > 244) {
    // プリスケーラー 256
    TCCR1B = (TCCR1B & 0xF8) | (1 << CS12);
    targetCompare = clockFreq / (256 * frequency) - 1;
  } else {
    // プリスケーラー 1024
    TCCR1B = (TCCR1B & 0xF8) | (1 << CS12) | (1 << CS10);
    targetCompare = clockFreq / (1024 * frequency) - 1;
  }
  
  // 範囲チェック
  if (targetCompare > 65535) targetCompare = 65535;
  if (targetCompare < 1) targetCompare = 1;
  
  OCR1A = (uint16_t)targetCompare;
  
  interrupts();
}

// Timer1 比較マッチ割り込み
ISR(TIMER1_COMPA_vect) {
  timerPulseFlag = true;
}

// 高速ADC設定関数（修正版）
void setupHighSpeedADC() {
  // ADC設定レジスタの初期化
  ADMUX = 0;                 // 全てクリア
  ADMUX |= (1 << REFS0);     // AVCC基準電圧
  ADMUX |= 0;                // A0ピン（アナログ0チャンネル）を選択
  
  ADCSRA = 0;                // 全てクリア
  ADCSRA |= (1 << ADEN);     // ADC有効化
  ADCSRA |= (1 << ADATE);    // オートトリガー有効
  ADCSRA |= (1 << ADIE);     // ADC割り込み有効
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // プリスケーラー128
  
  ADCSRB = 0;                // フリーランモード（ADTS=000）
  
  DIDR0 |= (1 << ADC0D);    // A0のデジタル入力バッファ無効化（ADCノイズ低減）
  
  // 初期化完了フラグ
  adc0_read_complete = false;
  analog_port = 0;
  
  // ADC開始
  ADCSRA |= (1 << ADSC);
}

// ADC割り込みサービスルーチン
ISR(ADC_vect) {
  if (analog_port == 0) {
    uint8_t low  = ADCL;                        // ADCLを先に読む（ADCHがロックされる）
    uint8_t high = ADCH;                        // 次にADCHを読む（ロック解除）
    adc0 = ((uint16_t)high << 8) | low;         // 10bit ADC値
    adc0_read_complete = true;                  // 完了フラグ設定
    return;
  }
}

// シリアルコマンド処理
// コマンド形式: KILL:ON/OFF, START:ON/OFF, RPM:0-6500, STATUS, RESET
void processSerialCommand(const char* cmd) {
  if (strncmp(cmd, "KILL:", 5) == 0) {
    const char* val = cmd + 5;
    if (strcmp(val, "ON") == 0) {
      serialKillSW = true;
    } else if (strcmp(val, "OFF") == 0) {
      serialKillSW = false;
    } else {
      Serial.println("ERR: KILL:ON or KILL:OFF");
      return;
    }
    serialControlMode = true;
    Serial.print("OK KILL:");
    Serial.println(serialKillSW ? "ON" : "OFF");
  } else if (strncmp(cmd, "START:", 6) == 0) {
    const char* val = cmd + 6;
    if (strcmp(val, "ON") == 0) {
      serialStartSW = true;
    } else if (strcmp(val, "OFF") == 0) {
      serialStartSW = false;
    } else {
      Serial.println("ERR: START:ON or START:OFF");
      return;
    }
    serialControlMode = true;
    Serial.print("OK START:");
    Serial.println(serialStartSW ? "ON" : "OFF");
  } else if (strncmp(cmd, "RPM:", 4) == 0) {
    int rpm = atoi(cmd + 4);
    if (rpm < 0 || rpm > MAX_RPM) {
      Serial.print("ERR: RPM 0-");
      Serial.println(MAX_RPM);
      return;
    }
    serialRPM = (uint16_t)rpm;
    serialKillSW  = true;   // RPMコマンドはKILL ON前提
    serialStartSW = false;  // RPMコマンドはSTART OFF前提
    serialControlMode = true;
    Serial.print("OK RPM:");
    Serial.println(serialRPM);
  } else if (strcmp(cmd, "STATUS") == 0) {
    Serial.print("Mode:");
    Serial.println(serialControlMode ? "SERIAL" : "PHYSICAL");
    Serial.print("RPM:");
    Serial.println(currentRPM);
    Serial.print("KILL_SW:");
    Serial.println(serialKillSW ? "ON" : "OFF");
    Serial.print("START_SW:");
    Serial.println(serialStartSW ? "ON" : "OFF");
  } else if (strcmp(cmd, "RESET") == 0) {
    serialControlMode = false;
    serialKillSW = false;
    serialStartSW = false;
    serialRPM = 0;
    Serial.println("OK RESET");
  } else {
    Serial.print("ERR: Unknown cmd: ");
    Serial.println(cmd);
  }
}

// シリアル入力チェック（非ブロッキング・バッファ方式）
void checkSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufPos > 0) {
        serialBuffer[serialBufPos] = '\0';
        processSerialCommand(serialBuffer);
        serialBufPos = 0;
      }
    } else if (serialBufPos < 31) {
      serialBuffer[serialBufPos++] = c;
    }
  }
}