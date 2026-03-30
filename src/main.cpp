#include <Arduino.h>

// SW状態定義
const int swON = LOW;
const int swOFF = HIGH;

// ピン定義
const int KILL_IN_PIN  = A4;
const int START_IN_PIN = A5;
const int POT_PIN      = A0;
const int A_PHASE_PIN  = 8;   // D8=PB0
const int Z_PHASE_PIN  = 9;   // D9=PB1
const int G_PHASE_PIN  = 10;  // D10=PB2
const int KILL_OUT_PIN  = 6;  // D6=PD6（アクティブLow）
const int START_OUT_PIN = 5;  // D5=PD5（アクティブLow）

// エンコーダ仕様（MA735互換）
const int PULSES_PER_REV = 360;
const int MAX_RPM = 6500;
const int START_RPM = 600;

// G相設定（変更可能）
const int G_PHASE_ROTATIONS = 2;
const int G_PHASE_START_DEG = 450;
const int G_PHASE_EDGES = 720;

// 高速計算用定数（コンパイル時計算）
const int TOTAL_EDGES_PER_REV   = PULSES_PER_REV * 4;           // 1440
const int TOTAL_EDGES_PER_CYCLE = TOTAL_EDGES_PER_REV * G_PHASE_ROTATIONS; // 2880
const int G_PHASE_START_EDGES   = (int)((int32_t)G_PHASE_START_DEG * TOTAL_EDGES_PER_REV / 360); // 1800
const int G_PHASE_END_EDGES     = G_PHASE_START_EDGES + G_PHASE_EDGES; // 2520

// タイマー制御
volatile bool useHighPrecisionTimer = false;

// ADC値
volatile uint16_t adc0 = 0;

// 変数
unsigned long lastMicros = 0;
unsigned long pulseInterval = 0;
uint16_t currentRPM = 0;

// エンコーダカウンタ（除算不要なインクリメント方式）
volatile int pulseCount = 0;  // G相周期内カウント（0〜TOTAL_EDGES_PER_CYCLE-1）
volatile int posInRev   = 0;  // 回転内カウント    （0〜TOTAL_EDGES_PER_REV-1）

// 状態変数（デバッグ用）
volatile bool aState = false;
volatile bool zState = false;
volatile bool gState = false;

// デバッグ
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000;

// シリアル制御
bool serialControlMode = false;
bool serialKillSW  = false;
bool serialStartSW = false;
uint16_t serialRPM = 0;

// シリアルバッファ
char serialBuffer[32];
uint8_t serialBufPos = 0;

// SW前回状態
int prevKillSWState  = swOFF;
int prevStartSWState = swOFF;

// デバウンス
const unsigned long SW_DEBOUNCE_MS = 20;
unsigned long killDebounceTimer  = 0;
unsigned long startDebounceTimer = 0;
int rawKillSWState_prev  = swOFF;
int rawStartSWState_prev = swOFF;
int debouncedKillState   = swOFF;
int debouncedStartState  = swOFF;

// プロトタイプ宣言
void generatePulse();
void setupHighPrecisionTimer();
void setTimerFrequency(uint32_t frequency);
void setupHighSpeedADC();
void checkSerialInput();
void processSerialCommand(const char* cmd);

// ─────────────────────────────────────────────────────────────────────────────
// パルス生成関数（ISRから直接呼び出し）
// ・除算不要なカウンタ方式で ISR 実行時間を最小化（% 1440 / % 2880 を排除）
// ・4エッジで1エンコーダパルス → 360 PPR の正しい出力
// ・出力：A相(D8/PB0), Z相(D9/PB1), G相(D10/PB2)
// ─────────────────────────────────────────────────────────────────────────────
void generatePulse() {
  // G相周期カウンタ
  if (++pulseCount >= TOTAL_EDGES_PER_CYCLE) pulseCount = 0;

  // 回転内エッジ位置カウンタ（除算不要）
  if (++posInRev >= TOTAL_EDGES_PER_REV) posInRev = 0;

  // エッジフェーズ: & 3 (= % 4, 2の累乗なのでビットマスクで高速)
  // 0,1 → HIGH / 2,3 → LOW の4エッジで1パルス → 360 PPR
  int ep = posInRev & 3;

  bool newA = (ep < 2);                   // A相：edgePhase 0,1 でHIGH
  bool newZ = (posInRev < 4 && ep == 1);  // Z相：1回転に1回、A&&B HIGH期間のみ出力
  bool newG = (pulseCount >= G_PHASE_START_EDGES && pulseCount < G_PHASE_END_EDGES); // G相

  // 状態更新（デバッグ用）
  aState = newA; zState = newZ; gState = newG;

  // レジスタ直接操作（高速IO）
  if (newA) PORTB |= (1 << PB0); else PORTB &= ~(1 << PB0);  // A相 D8=PB0
  if (newZ) PORTB |= (1 << PB1); else PORTB &= ~(1 << PB1);  // Z相 D9=PB1
  if (newG) PORTB |= (1 << PB2); else PORTB &= ~(1 << PB2);  // G相 D10=PB2
}

void setup() {
  Serial.begin(115200);

  // ピン設定
  pinMode(A_PHASE_PIN, OUTPUT);    // D8
  pinMode(Z_PHASE_PIN, OUTPUT);    // D9
  pinMode(G_PHASE_PIN, OUTPUT);    // D10
  pinMode(KILL_OUT_PIN, OUTPUT);   // D6
  pinMode(START_OUT_PIN, OUTPUT);  // D5
  pinMode(KILL_IN_PIN, INPUT_PULLUP);
  pinMode(START_IN_PIN, INPUT_PULLUP);

  // 初期状態（KILL/START出力をHIGHに=非アクティブ）
  PORTD |= ((1 << PD5) | (1 << PD6));
  PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));
  PORTB &= ~((1 << PB0) | (1 << PB1));

  // ADC初期化（簡易版）
  pinMode(POT_PIN, INPUT);

  Serial.println("MA735 Encoder Simulator Started");
  Serial.println("PPT=359 (360 pulses/turn), ILIP=1111");
  Serial.print("G Phase: "); Serial.print(G_PHASE_ROTATIONS);
  Serial.print(" rot cycle, start "); Serial.print(G_PHASE_START_DEG);
  Serial.print("deg, "); Serial.print(G_PHASE_EDGES); Serial.println(" edges");
  Serial.println("D8=A, D9=Z, D10=G | A4=KillIn, A5=StartIn | D6=KillOut, D5=StartOut");
  Serial.println("--- Commands: KILL:ON/OFF | START:ON/OFF | RPM:0-6500 | STATUS | RESET ---");

  // 念のためグローバル割り込みを明示的に有効化
  interrupts();
}

void loop() {
  interrupts();

  checkSerialInput();

  // SWデバウンス（A4=PC4, A5=PC5）
  int rawKill  = (PINC & (1 << PC4)) ? swOFF : swON;
  int rawStart = (PINC & (1 << PC5)) ? swOFF : swON;
  unsigned long nowMs = millis();

  if (rawKill != rawKillSWState_prev) { rawKillSWState_prev = rawKill; killDebounceTimer = nowMs; }
  if (nowMs - killDebounceTimer >= SW_DEBOUNCE_MS) debouncedKillState = rawKill;
  int KILL_SW_State = debouncedKillState;

  if (rawStart != rawStartSWState_prev) { rawStartSWState_prev = rawStart; startDebounceTimer = nowMs; }
  if (nowMs - startDebounceTimer >= SW_DEBOUNCE_MS) debouncedStartState = rawStart;
  int START_SW_State = debouncedStartState;

  // 物理SWのOFF→ONエッジでシリアル制御モードを解除
  bool killPressedEdge  = (prevKillSWState  == swOFF && KILL_SW_State  == swON);
  bool startPressedEdge = (prevStartSWState == swOFF && START_SW_State == swON);
  if (serialControlMode && (killPressedEdge || startPressedEdge)) {
    serialControlMode = false;
    serialKillSW = false;
    serialStartSW = false;
    serialRPM = 0;
    Serial.println("INFO: Physical switch edge detected. Back to PHY mode");
  }

  prevKillSWState  = KILL_SW_State;
  prevStartSWState = START_SW_State;

  // シリアルモード時はSW状態を上書き
  if (serialControlMode) {
    KILL_SW_State  = serialKillSW  ? swON : swOFF;
    START_SW_State = serialStartSW ? swON : swOFF;
  }

  // SW状態→出力制御・RPM決定
  if (KILL_SW_State == swON) {
    uint8_t sreg = SREG;
    noInterrupts();
    PORTD &= ~(1 << PD6);  // KILL出力 D6=LOW（アクティブ）
    SREG = sreg;
    if (START_SW_State == swON) {
      sreg = SREG;
      noInterrupts();
      PORTD &= ~(1 << PD5);  // START出力 D5=LOW（アクティブ）
      SREG = sreg;
      currentRPM = START_RPM;
    } else {
      sreg = SREG;
      noInterrupts();
      PORTD |= (1 << PD5);  // START出力 D5=HIGH（非アクティブ）
      SREG = sreg;
      if (serialControlMode) {
        currentRPM = serialRPM;
      } else {
        adc0 = analogRead(POT_PIN);
        currentRPM = (uint16_t)(((uint32_t)adc0 * MAX_RPM) / 1023);
      }
    }
  } else {
    uint8_t sreg = SREG;
    noInterrupts();
    PORTD |= (1 << PD6);  // KILL出力 D6=HIGH（非アクティブ）
    PORTD |= (1 << PD5);  // START出力 D5=HIGH（非アクティブ）
    SREG = sreg;
    currentRPM = 0;
  }

  // パルス間隔計算（generatePulse をTOTAL_EDGES_PER_REV回/回転で呼び出す）
  if (currentRPM > 0) {
    uint32_t pulsesPerSecond = ((uint32_t)currentRPM * TOTAL_EDGES_PER_REV) / 60;
    pulseInterval = 1000000UL / pulsesPerSecond;

    if (pulsesPerSecond > 10000) {
      if (!useHighPrecisionTimer) { setupHighPrecisionTimer(); useHighPrecisionTimer = true; }
      setTimerFrequency(pulsesPerSecond);
    } else {
      if (useHighPrecisionTimer) { TIMSK1 &= ~(1 << OCIE1A); useHighPrecisionTimer = false; }
    }
  } else {
    pulseInterval = 0;
    if (useHighPrecisionTimer) { TIMSK1 &= ~(1 << OCIE1A); useHighPrecisionTimer = false; }
  }

  // SWタイマー（HWタイマー時はISRから自動呼び出し）
  if (!useHighPrecisionTimer) {
    unsigned long cm = micros();
    if (pulseInterval > 0 && (cm - lastMicros >= pulseInterval)) {
      generatePulse();
      lastMicros = cm;
    }
  }

  // デバッグ出力（1秒間隔）
  if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
    int angleDeg = ((long)pulseCount * 360) / TOTAL_EDGES_PER_REV;
    Serial.print("KILL SW: "); Serial.print(KILL_SW_State == swON ? "ON" : "OFF");
    Serial.print(", START SW: "); Serial.print(START_SW_State == swON ? "ON" : "OFF");
    Serial.print(", RPM: "); Serial.print(currentRPM);
    Serial.print(", Angle: "); Serial.print(angleDeg);
    Serial.print(", Pulse: "); Serial.print(pulseCount);
    Serial.print("/"); Serial.print(TOTAL_EDGES_PER_CYCLE);
    Serial.print(", G: "); Serial.print(gState ? "H" : "L");
    Serial.print(", Mode: "); Serial.print(serialControlMode ? "SER" : "PHY");
    Serial.print(", Timer: "); Serial.print(useHighPrecisionTimer ? "HW" : "SW");
    Serial.print(", ADC: "); Serial.print(adc0);
    Serial.println();
    lastDebugTime = millis();
  }
}

// 高精度タイマー設定
void setupHighPrecisionTimer() {
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  TCCR1B |= (1 << WGM12);              // CTCモード
  TCCR1B |= (1 << CS11) | (1 << CS10); // プリスケーラー64（初期）
  TIMSK1 |= (1 << OCIE1A);             // 割り込み有効
  interrupts();
}

// タイマー周波数設定
void setTimerFrequency(uint32_t frequency) {
  noInterrupts();
  const uint32_t F = 16000000UL;
  uint32_t cmp;
  if      (frequency > 62500) { TCCR1B = (TCCR1B & 0xF8) | (1 << CS10);            cmp = F / frequency - 1; }
  else if (frequency >  7812) { TCCR1B = (TCCR1B & 0xF8) | (1 << CS11);            cmp = F / (    8 * frequency) - 1; }
  else if (frequency >   976) { TCCR1B = (TCCR1B & 0xF8) | (1 << CS11) | (1 << CS10); cmp = F / (   64 * frequency) - 1; }
  else if (frequency >   244) { TCCR1B = (TCCR1B & 0xF8) | (1 << CS12);            cmp = F / (  256 * frequency) - 1; }
  else                        { TCCR1B = (TCCR1B & 0xF8) | (1 << CS12) | (1 << CS10); cmp = F / ( 1024 * frequency) - 1; }
  if (cmp > 65535) cmp = 65535;
  if (cmp < 1)     cmp = 1;
  OCR1A = (uint16_t)cmp;
  interrupts();
}

// Timer1 比較マッチ割り込み → generatePulse() を直接呼び出し
ISR(TIMER1_COMPA_vect) {
  generatePulse();
}

// ADC設定（未使用。互換のため残置）
void setupHighSpeedADC() {
}

// シリアルコマンド処理
void processSerialCommand(const char* cmd) {
  if (strncmp(cmd, "KILL:", 5) == 0) {
    const char* v = cmd + 5;
    if      (strcmp(v, "ON")  == 0) serialKillSW = true;
    else if (strcmp(v, "OFF") == 0) serialKillSW = false;
    else { Serial.println("ERR: KILL:ON or KILL:OFF"); return; }
    serialControlMode = true;
    Serial.print("OK KILL:"); Serial.println(serialKillSW ? "ON" : "OFF");

  } else if (strncmp(cmd, "START:", 6) == 0) {
    const char* v = cmd + 6;
    if      (strcmp(v, "ON")  == 0) serialStartSW = true;
    else if (strcmp(v, "OFF") == 0) serialStartSW = false;
    else { Serial.println("ERR: START:ON or START:OFF"); return; }
    serialControlMode = true;
    Serial.print("OK START:"); Serial.println(serialStartSW ? "ON" : "OFF");

  } else if (strncmp(cmd, "RPM:", 4) == 0) {
    int rpm = atoi(cmd + 4);
    if (rpm < 0 || rpm > MAX_RPM) { Serial.print("ERR: RPM 0-"); Serial.println(MAX_RPM); return; }
    serialRPM = (uint16_t)rpm;
    serialKillSW = true; serialStartSW = false; serialControlMode = true;
    Serial.print("OK RPM:"); Serial.println(serialRPM);

  } else if (strcmp(cmd, "STATUS") == 0) {
    Serial.print("Mode:");     Serial.println(serialControlMode ? "SERIAL" : "PHYSICAL");
    Serial.print("RPM:");      Serial.println(currentRPM);
    Serial.print("KILL_SW:");  Serial.println(serialKillSW  ? "ON" : "OFF");
    Serial.print("START_SW:"); Serial.println(serialStartSW ? "ON" : "OFF");

  } else if (strcmp(cmd, "RESET") == 0) {
    serialControlMode = false; serialKillSW = false; serialStartSW = false; serialRPM = 0;
    Serial.println("OK RESET");

  } else {
    Serial.print("ERR: Unknown cmd: "); Serial.println(cmd);
  }
}

// シリアル入力チェック（非ブロッキング）
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
