#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =================== LCD CONFIG ===================
#define LCD_ADDRESS 0x27          // Đổi thành 0x3F nếu LCD không hiện
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// =================== PIN CONFIG ===================
// Stepper driver (DM542)
const int STEP_PIN = 25;
const int DIR_PIN  = 26;
// KHÔNG dùng enable pin (ENA+ / ENA- nên được nối để luôn enable driver)

// Buttons (active LOW, dùng INPUT_PULLUP)
const int BTN_START = 13;
const int BTN_STOP  = 14;
const int BTN_UP    = 15;
const int BTN_DOWN  = 16;

// =================== SYRINGE & MECHANICS ===================
// THÔNG SỐ CƠ KHÍ – chỉnh cho đúng với hệ của bạn
const float SYRINGE_INNER_DIAMETER_MM = 26.6;  // đường kính trong ống 50mL (xấp xỉ)
const float LEADSCREW_PITCH_MM        = 8.0;   // bước vít me (mm/vòng)
const int   STEPS_PER_REV             = 200;   // 1.8° stepper
const int   MICROSTEPS                = 16;    // hệ số microstep đang set trên DM542 (vd: 16 microstep/step)
static int volume_value = 5, speed_value = 10, diameter = 20, radius = 10 ;
static unsigned int syringe_type = 0;
// Tính từ trên ra:
const int   MICROSTEP_PER_REV         = STEPS_PER_REV * MICROSTEPS;

float stepsPerMl = 0.0;  // số microstep cần cho 1 mL

// =================== PUMP PARAMETERS ===================
float targetVolumeMl   = 1.0;   // thể tích cần bơm (mL) – mặc định ban đầu
float targetFlowMlPerH = 10.0;   // lưu lượng (mL/h) – mặc định ban đầu

const float MIN_VOLUME_ML   = 0.0;
const float MAX_VOLUME_ML   = 10.0;

const float MIN_FLOW_ML_H   = 10.0;
const float MAX_FLOW_ML_H   = 500.0;   // tăng lên để test cho motor quay nhanh dễ thấy

// =================== PUMP RUNTIME VARIABLES ===================
enum PumpState {
  SET_VOLUME,   // chỉnh volume
  SET_FLOW,     // chỉnh flowrate
  RUNNING,      // đang bơm
  DONE          // hoàn thành
};

PumpState pumpState = SET_VOLUME;

unsigned long lastStepMicros     = 0;
unsigned long stepIntervalMicros = 0;

long totalSteps     = 0;
long completedSteps = 0;

// =================== BUTTON HANDLING ===================
bool lastStartState = HIGH;
bool lastStopState  = HIGH;
bool lastUpState    = HIGH;
bool lastDownState  = HIGH;

// debounce cho START/STOP
const long stepsPerML = 3840;
const unsigned long DEBOUNCE_MS = 200;
unsigned long lastStartPressTime = 0;
unsigned long lastStopPressTime  = 0;

// Đọc trạng thái cạnh xuống (HIGH -> LOW)
  bool readButtonEdge(int pin, bool &lastState) {
  bool current = digitalRead(pin);
  bool pressed = (lastState == HIGH && current == LOW); // phát hiện nhấn (HIGH->LOW)
  lastState = current;
  return pressed;
}

// =================== HELPER FUNCTIONS ===================

// Tính số microstep cần cho 1 mL dựa trên đường kính ống & bước vít
void computeStepsPerMl() {
  float area   = 3.14159265f * radius * radius;            // mm^2
  // Mỗi 1 mL = 1000 mm^3
  // mm dịch chuyển để bơm 1 mL:
  float mmPerMl = 1000.0f / area;                          // mm/mL

  // Số bước/vòng * (mmPerMl / pitch)
  stepsPerMl = MICROSTEP_PER_REV * (mmPerMl /  LEADSCREW_PITCH_MM );

  Serial.print("stepsPerMl: ");
  Serial.println(stepsPerMl);
}

// Tính khoảng thời gian giữa 2 step dựa trên flowrate (mL/h)
void updateStepInterval() {
  if (targetFlowMlPerH <= 0.0f) {
    stepIntervalMicros = 0;
    return;
  }

  float flowMlPerS = targetFlowMlPerH / 3600.0f;          // mL/giây
  float stepsPerSecond = flowMlPerS * stepsPerMl;         // step/giây

  if (stepsPerSecond <= 0.0f) {
    stepIntervalMicros = 0;
    return;
  }

  stepIntervalMicros = (unsigned long)(1000000.0f / stepsPerSecond);

  // Optional: giới hạn cho stepIntervalMicros không quá nhỏ (quá nhanh)
  if (stepIntervalMicros < 500) { // ~2 kHz max
    stepIntervalMicros = 500;
  }

  Serial.print("stepIntervalMicros: ");
  Serial.println(stepIntervalMicros);
}

// Bắt đầu bơm theo volume & flow đã chọn
void startPump() {
  totalSteps     = (long)(targetVolumeMl * stepsPerMl);
  completedSteps = 0;

  updateStepInterval();
  if (stepIntervalMicros == 0 || totalSteps <= 0) {
    // Không có gì để bơm (volume=0 hoặc flow=0) → quay lại set volume
    pumpState = SET_VOLUME;
    return;
  }

  // DEBUG: in ra thông số để kiểm tra
  float pumpTimeSeconds = (float)totalSteps * (float)stepIntervalMicros / 1000000.0f;
  Serial.println("=== START PUMP ===");
  Serial.print("Volume (mL): ");
  Serial.println(targetVolumeMl);
  Serial.print("Flow (mL/h): ");
  Serial.println(targetFlowMlPerH);
  Serial.print("stepsPerMl: ");
  Serial.println(stepsPerMl);
  Serial.print("totalSteps: ");
  Serial.println(totalSteps);
  Serial.print("stepIntervalMicros: ");
  Serial.println(stepIntervalMicros);
  Serial.print("Estimated pump time (s): ");
  Serial.println(pumpTimeSeconds);
  Serial.println("==================");

  // Chọn chiều bơm (HIGH/LOW tùy cách gắn syringe)
  digitalWrite(DIR_PIN, HIGH);   // nếu bơm ngược thì đổi thành LOW

  lastStepMicros = micros();
  pumpState = RUNNING;
}

// Dừng bơm theo yêu cầu người dùng (STOP)
void stopPump() {
  completedSteps = 0;
  totalSteps     = 0;
  pumpState      = SET_VOLUME;
}

// Hoàn thành đủ volume → sang trạng thái DONE
void finishPump() {
  pumpState = DONE;
}

// Phát một xung STEP (1 step đầy đủ)
void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);     // độ rộng xung
  digitalWrite(STEP_PIN, LOW);
}

// =================== LCD DISPLAY ===================
void showSetVolume() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Volume:");
  lcd.setCursor(0, 1);
  lcd.print(targetVolumeMl, 1);
  lcd.print(" mL    ");
}

void showSetFlow() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Flow:");
  lcd.setCursor(0, 1);
  lcd.print(targetFlowMlPerH, 1);
  lcd.print(" mL/h   ");
}

void showRunning() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RUN");

  lcd.setCursor(0, 1);
  lcd.print(targetVolumeMl, 1);
  lcd.print("mL ");
  lcd.print(targetFlowMlPerH, 1);
  lcd.print("mL/h");
}

void showDone() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Done!");
  lcd.setCursor(0, 1);
  lcd.print(targetVolumeMl, 1);
  lcd.print("mL ");
  lcd.print(targetFlowMlPerH, 1);
  lcd.print("mL/h");
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);

  Wire.begin(21, 22);   // SDA=21, SCL=22 cho ESP32
  lcd.init();
  lcd.backlight();

  computeStepsPerMl();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Syringe Pump");
  lcd.setCursor(0, 1);
  lcd.print("ESP32 + NEMA17");
  delay(2000);

  showSetVolume();
}

// =================== LOOP ===================
void loop() {
  unsigned long nowMs = millis();

  // Đọc cạnh nhấn nút (edge)
  bool rawStartEdge = readButtonEdge(BTN_START, lastStartState);
  bool rawStopEdge  = readButtonEdge(BTN_STOP,  lastStopState);
  bool upPressed    = readButtonEdge(BTN_UP,    lastUpState);
  bool downPressed  = readButtonEdge(BTN_DOWN,  lastDownState);

  // Debounce thời gian cho START/STOP
  bool startPressed = false;
  bool stopPressed  = false;

  if (rawStartEdge && (nowMs - lastStartPressTime >= DEBOUNCE_MS)) {
    startPressed = true;
    lastStartPressTime = nowMs;
  }
  if (rawStopEdge && (nowMs - lastStopPressTime >= DEBOUNCE_MS)) {
    stopPressed = true;
    lastStopPressTime = nowMs;
  }

  // STOP luôn ưu tiên – dừng bơm ngay khi đang RUNNING
  if (stopPressed && pumpState == RUNNING) {
    stopPump();
    showSetVolume();
  }

  switch (pumpState) {
    case SET_VOLUME:
      // Chỉnh Volume bằng UP/DOWN (unit: mL)
      if (upPressed) {
        targetVolumeMl += 1.0f;
        if (targetVolumeMl > MAX_VOLUME_ML) targetVolumeMl = MAX_VOLUME_ML;
        showSetVolume();
      }
      if (downPressed) {
        targetVolumeMl -= 1.0f;
        if (targetVolumeMl < MIN_VOLUME_ML) targetVolumeMl = MIN_VOLUME_ML;
        showSetVolume();
      }
      // Nhấn START → sang set Flow
      if (startPressed) {
        pumpState = SET_FLOW;
        showSetFlow();
      }
      break;

    case SET_FLOW:
      // Chỉnh Flow bằng UP/DOWN (unit: mL/h)
      if (upPressed) {
        targetFlowMlPerH += 10.0f;
        if (targetFlowMlPerH > MAX_FLOW_ML_H) targetFlowMlPerH = MAX_FLOW_ML_H;
        showSetFlow();
      }
      if (downPressed) {
        targetFlowMlPerH -= 10.0f;
        if (targetFlowMlPerH < MIN_FLOW_ML_H) targetFlowMlPerH = MIN_FLOW_ML_H;
        showSetFlow();
      }
      // Nhấn START → bắt đầu bơm theo Volume + Flow đã set
      if (startPressed) {
        startPump();
        if (pumpState == RUNNING) {
          showRunning();
        } else {
          // trường hợp volume = 0 hoặc flow = 0 -> không chạy, quay lại set volume
          showSetVolume();
        }
      }
      break;

    case RUNNING:
      // Bơm theo stepIntervalMicros dựa trên flowrate
      if (completedSteps < totalSteps && stepIntervalMicros > 0) {
        unsigned long nowUs = micros();
        if (nowUs - lastStepMicros >= stepIntervalMicros) {
          lastStepMicros = nowUs;
          stepMotor();
          completedSteps++;

          // Thỉnh thoảng refresh LCD (mỗi 100 step)
          if (completedSteps % 100 == 0) {
            showRunning();
          }
        }
      } else if (completedSteps >= totalSteps) {
        // Hoàn thành đủ volume
        finishPump();
        showDone();
      }
      break;

    case DONE:
      // Sau khi bơm xong, nhấn START để set lại từ đầu
      if (startPressed) {
        pumpState = SET_VOLUME;
        showSetVolume();
      }
      break;

    default:
      pumpState = SET_VOLUME;
      showSetVolume();
      break;
  }
}
