#include <Arduino.h>

// SW状態定義
const int swON = LOW;    // SWがONのピンの値はLOW
const int swOFF = HIGH;  // SWがOFFのピンの値はHIGH

// ピン定義
const int KILL_IN_PIN = A4;    // キルスイッチ入力
const int START_IN_PIN  = A5;  // スタータボタン入力
const int POT_PIN = A7;        // 可変抵抗
const int A_PHASE_PIN = 2;     // A相出力
const int B_PHASE_PIN = 8;     // B相出力
const int Z_PHASE_PIN = 9;     // Z相出力
const int G_PHASE_PIN = 5;     // G相出力
const int KILL_OUT_PIN  = 7;   // キルスイッチ出力
const int START_OUT_PIN = 6;   // スタータボタン出力


// エンコーダ仕様（MA735互換）
const int PULSES_PER_REV = 360;   // 1回転あたりのパルス数（PPT=359+1）
const float MAX_RPM = 6500.0;     // 最大回転数
const float START_RPM = 600.0;   // 始動時の回転数

// G相設定（変更可能）
const int G_PHASE_ROTATIONS = 2;  // G相の出力周期（回転数）
const float G_PHASE_START_DEG = 450.0;  // G相開始角度（度）
const int G_PHASE_EDGES = 720;      // G相出力エッジ数(4エッジ/1度))

// 変数
unsigned long lastMicros = 0;   // 最後のパルス生成時間
unsigned long pulseInterval;  // パルス間隔（マイクロ秒）
float currentRPM = 0;           // 現在の回転数（RPM）
int pulseCount = 0;             // パルスカウント
bool aState = false;            // A相状態
bool bState = false;            // B相状態
bool zState = false;            // Z相状態
bool gState = false;            // G相状態

// デバッグ用
unsigned long lastDebugTime = 0;    // 最後のデバッグ出力時間
const unsigned long DEBUG_INTERVAL = 1000; // 1秒間隔

// プロトタイプ宣言
void generatePulse();           // パルス生成関数

// パルス生成関数
// この関数は、エンコーダのパルスを生成し、A相、B相、Z相、G相の状態を更新します。
// G相は指定された周期で出力され、開始角度とエッジ数に基づいて制御されます。
void generatePulse() {
  // パルスカウントを増加
  pulseCount++;
  
  // G相の周期を考慮したカウンターリセット
  int totalEdgesPerCycle = PULSES_PER_REV * 4 * G_PHASE_ROTATIONS;
  if (pulseCount >= totalEdgesPerCycle) {
    pulseCount = 0;
  }
  
  // 現在の位置（1回転内）
  int positionInRev = pulseCount % (PULSES_PER_REV * 4);
  
  // A相とB相の状態を決定（90度位相差）
  bool newAState = (positionInRev % (PULSES_PER_REV * 2)) < PULSES_PER_REV;
  
  // B相の状態（A相より90度遅れ）
  int bPosition = (positionInRev + PULSES_PER_REV) % (PULSES_PER_REV * 4);
  bool newBState = (bPosition % (PULSES_PER_REV * 2)) < PULSES_PER_REV;
  
  // Z相（インデックス）の状態
  // ILIP=1111の場合：A相とB相が両方HIGHの時にZ相もHIGH
  bool newZState = false;
  if (positionInRev < (PULSES_PER_REV / 2)) {  // 1回転の最初の部分
    if (newAState && newBState) {  // A相とB相が両方HIGH
      newZState = true;
    }
  }
  
  // G相の状態計算
  bool newGState = false;
  
  // 現在の角度を計算（度）
  float currentAngle = (float)pulseCount * 360.0 / (PULSES_PER_REV * 4);
  
  // G相の周期内での角度
  float angleInCycle = fmod(currentAngle, 360.0 * G_PHASE_ROTATIONS);
  
  // G相の開始と終了角度を計算
  float gStartAngle = G_PHASE_START_DEG;
  float gEndAngle = gStartAngle + (float)G_PHASE_EDGES * 360.0 / (PULSES_PER_REV * 4);
  
  // G相の出力判定
  if (angleInCycle >= gStartAngle && angleInCycle < gEndAngle) {
    newGState = true;
  }
  
  // エッジ検出のため前の状態を保存
  bool prevAState = aState;
  bool prevBState = bState;
  
  // 状態を更新
  aState = newAState;
  bState = newBState;
  zState = newZState;
  gState = newGState;
  
  // 出力を更新
  digitalWrite(A_PHASE_PIN, aState ? HIGH : LOW);
  digitalWrite(B_PHASE_PIN, bState ? HIGH : LOW);
  digitalWrite(Z_PHASE_PIN, zState ? HIGH : LOW);
  digitalWrite(G_PHASE_PIN, gState ? HIGH : LOW);
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
    
  // 初期状態
  digitalWrite(A_PHASE_PIN, LOW);
  digitalWrite(B_PHASE_PIN, LOW);
  digitalWrite(Z_PHASE_PIN, LOW);
  digitalWrite(G_PHASE_PIN, LOW);
  digitalWrite(KILL_OUT_PIN, LOW);
  digitalWrite(START_OUT_PIN, LOW);
  
  Serial.println("MA735 Encoder Simulator Started");
  Serial.println("PPT=359 (360 pulses/turn), ILIP=1111");
  Serial.print("G Phase: ");
  Serial.print(G_PHASE_ROTATIONS);
  Serial.print(" rotations cycle, starts at ");
  Serial.print(G_PHASE_START_DEG);
  Serial.print("°, ");
  Serial.print(G_PHASE_EDGES);
  Serial.println(" edges");
  Serial.println("A0: Speed Control (0-6500 RPM)");
  Serial.println("D2: A Phase, D3: B Phase, D4: Z Phase, D5: G Phase");
}

void loop() {
  // SWの状態を確認
  int KILL_SW_State = digitalRead(KILL_IN_PIN);
  int START_SW_State = digitalRead(START_IN_PIN);

  // キルスイッチの状態に応じて出力を制御
  if (KILL_SW_State == swON) {  // キルスイッチがON
    digitalWrite(KILL_OUT_PIN, LOW);

    // スタータボタンの状態に応じて出力を制御
    if (START_SW_State == swON) {  // スタータボタンがON
      digitalWrite(START_OUT_PIN, LOW);
      currentRPM = START_RPM; // 始動時回転数
    } else {                        // スタータボタンがOFF
      digitalWrite(START_OUT_PIN, HIGH);
    
      // 可変抵抗から回転数を読み取り
      int potValue = analogRead(POT_PIN);
      currentRPM = map(potValue, 0, 1023, 0, MAX_RPM * 100) / 100.0;
    }
  } else {                      // キルスイッチがOFF
    digitalWrite(KILL_OUT_PIN, HIGH);

    // パルス生成を停止
    currentRPM = 0; // 停止状態
  }
  
  // パルス間隔を計算（マイクロ秒）
  if (currentRPM > 0) {
    float pulsesPerSecond = (currentRPM / 60.0) * PULSES_PER_REV;
    pulseInterval = 1000000.0 / pulsesPerSecond; // マイクロ秒
  } else {
    pulseInterval = 0; // 停止
  }

  // パルス生成
  unsigned long currentMicros = micros();
  if (pulseInterval > 0 && (currentMicros - lastMicros >= pulseInterval)) {
    generatePulse();
    lastMicros = currentMicros;
  }
  
  // デバッグ情報出力
  if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
    float currentAngle = (float)pulseCount * 360.0 / (PULSES_PER_REV * 4);
    float angleInCycle = fmod(currentAngle, 360.0 * G_PHASE_ROTATIONS);
    
    Serial.print("KILL SW: ");
    Serial.print(KILL_SW_State == swON ? "ON" : "OFF");
    Serial.print(", START SW: ");
    Serial.print(START_SW_State == swON ? "ON" : "OFF");
    Serial.print(", RPM: ");
    Serial.print(currentRPM, 1);
    Serial.print(", Angle: ");
    Serial.print(angleInCycle, 1);
    Serial.print("°, Pulse: ");
    Serial.print(pulseCount);
    Serial.print("/");
    Serial.print(PULSES_PER_REV * 4 * G_PHASE_ROTATIONS);
    Serial.print(", G: ");
    Serial.print(gState ? "H" : "L");
    Serial.println();
    lastDebugTime = millis();
  }
}