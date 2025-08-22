/*
  QuadControl_ArduinoUno.ino
  Controle básico de quadricóptero para Arduino Uno (ATmega328P)

  Funcionalidades:
  - Leitura MPU6050 via I2C (accel + gyro)
  - Filtro complementar (opção de ativar Kalman 1D)
  - Parsing manual do NMEA $GPGGA via SoftwareSerial (GY-GPS6MV2)
  - PID para Roll, Pitch, Yaw
  - Saída PWM com Servo.writeMicroseconds para ESCs (1000..2000us)

  Notas:
  - Substitua throttleInput pela leitura do seu receptor RC (PWM/PPM/SBUS)
  - Teste com hélices removidas inicialmente!
*/

#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>

// ============================ CONFIGURAÇÃO ============================
// MPU6050 I2C address
#define MPU_ADDR 0x68

// GPS via SoftwareSerial (conecte TX do GPS ao RX_PIN)
const uint8_t GPS_RX_PIN = 0;
const uint8_t GPS_TX_PIN = 1;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

// ESC / motors pins (usar pinos PWM virtuais via Servo lib)
const uint8_t PIN_M1 = 3;  // Motor 1 (ex: front-left)
const uint8_t PIN_M2 = 5;  // Motor 2 (ex: front-right)
const uint8_t PIN_M3 = 6;  // Motor 3 (ex: rear-right)
const uint8_t PIN_M4 = 9; // Motor 4 (ex: rear-left)
// OBS: pinos escolhidos suportam Servo/Timer. Ajuste conforme seu hardware.

// ESC signal limits (microseconds)
const int ESC_MIN = 1000;
const int ESC_MAX = 2000;

// Sensor scaling (MPU6050 default assumed: accel ±2g, gyro ±250 dps)
const float ACCEL_SCALE = 16384.0; // LSB/g
const float GYRO_SCALE  = 131.0;   // LSB/(deg/s)

// Filtro complementar
const float ALPHA = 0.98; // peso do giroscópio na fusão (0..1). Ajustar entre 0.95-0.995

// Opcional: habilitar Kalman 1D por eixo (descomente para usar)
// #define USE_KALMAN

// PID parâmetros iniciais (tune estes valores)
struct PIDParams {
  float Kp;
  float Ki;
  float Kd;
  float outputMin;
  float outputMax;
};
PIDParams pidRoll  = {4.0, 0.01, 0.6, -400, 400};
PIDParams pidPitch = {4.0, 0.01, 0.6, -400, 400};
PIDParams pidYaw   = {3.0, 0.005, 0.2, -200, 200};

// Desired angles (setpoint). Para nível: 0.0.
float setRoll  = 0.0;
float setPitch = 0.0;
float setYaw   = 0.0; // se usar yaw hold

// Throttle (1000..2000). Substituir pela leitura do rádio.
int throttleInput = 1200; // valor inicial para teste (sempre ajustar com cuidado)

// ============================ VARIÁVEIS GLOBAIS ============================
Servo esc1, esc2, esc3, esc4;

unsigned long lastMicros = 0;

// MPU raw
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

// Angulos estimados (graus)
float rollAcc, pitchAcc;      // ângulos a partir do acelerômetro
float rollEstimate = 0.0;     // estimativa final (complementar/kalman)
float pitchEstimate = 0.0;
float yawEstimate = 0.0;      // integração do gyro Z (deriva)

// ============================ KALMAN 1D (opcional) ============================
#ifdef USE_KALMAN
// Simples Kalman 1D por eixo (angle, bias)
class Kalman {
public:
  float Q_angle, Q_bias, R_measure;
  float angle; // estimativa do angulo
  float bias;
  float P[2][2];

  Kalman() {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;
    angle = 0.0f; bias = 0.0f;
    P[0][0] = 0; P[0][1] = 0; P[1][0] = 0; P[1][1] = 0;
  }
  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;
    float P00 = P[0][0], P01 = P[0][1], P10 = P[1][0], P11 = P[1][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;
    return angle;
  }
};
Kalman kalmanRoll, kalmanPitch;
#endif

// ============================ PID CONTROL ============================
class PID {
public:
  PIDParams params;
  float integral;
  float lastError;
  PID(PIDParams p) : params(p), integral(0.0), lastError(0.0) {}
  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;
    // anti-windup
    if (integral * params.Ki > params.outputMax) integral = params.outputMax / params.Ki;
    if (integral * params.Ki < params.outputMin) integral = params.outputMin / params.Ki;
    float derivative = (error - lastError) / dt;
    lastError = error;
    float out = params.Kp * error + params.Ki * integral + params.Kd * derivative;
    if (out > params.outputMax) out = params.outputMax;
    if (out < params.outputMin) out = params.outputMin;
    return out;
  }
  void reset() { integral = 0; lastError = 0; }
};

PID pid_controller_roll(pidRoll);
PID pid_controller_pitch(pidPitch);
PID pid_controller_yaw(pidYaw);

// ============================ GPS PARSING $GPGGA ============================
char gpsLine[120];
uint8_t gpsLineIndex = 0;
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
bool gpsFix = false;

void parseGPGGA(const char* line) {
  // expected format: $GPGGA,time,lat,NS,lon,EW,fix,... e.g.:
  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  // Tokenize by commas
  char buf[120];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;

  char* tokens[20];
  uint8_t nt = 0;
  char* p = strtok(buf, ",");
  while (p && nt < 20) {
    tokens[nt++] = p;
    p = strtok(NULL, ",");
  }
  if (nt < 7) return;
  if (strcmp(tokens[0], "$GPGGA") != 0) return;

  const char* latStr = tokens[2];
  const char* ns = tokens[3];
  const char* lonStr = tokens[4];
  const char* ew = tokens[5];
  const char* fixStr = tokens[6];

  int fix = atoi(fixStr);
  if (fix == 0) {
    gpsFix = false;
    return;
  }
  gpsFix = true;

  // Convert lat ddmm.mmmm to decimal degrees
  if (strlen(latStr) > 3 && strlen(lonStr) > 4) {
    float latVal = atof(latStr);
    int latDeg = int(latVal / 100);
    float latMin = latVal - latDeg * 100.0;
    float latDecimal = latDeg + latMin / 60.0;
    if (ns && ns[0] == 'S') latDecimal = -latDecimal;

    float lonVal = atof(lonStr);
    int lonDeg = int(lonVal / 100);
    float lonMin = lonVal - lonDeg * 100.0;
    float lonDecimal = lonDeg + lonMin / 60.0;
    if (ew && ew[0] == 'W') lonDecimal = -lonDecimal;

    gpsLatitude = latDecimal;
    gpsLongitude = lonDecimal;
  }
}

// Read GPS line (non-blocking)
void gpsReadLoop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gpsLine[gpsLineIndex] = 0;
      if (gpsLineIndex > 0) {
        // Process line
        if (strncmp(gpsLine, "$GPGGA", 6) == 0) {
          parseGPGGA(gpsLine);
        }
      }
      gpsLineIndex = 0;
    } else {
      if (gpsLineIndex < sizeof(gpsLine)-1) {
        gpsLine[gpsLineIndex++] = c;
      } else {
        // overflow
        gpsLineIndex = 0;
      }
    }
  }
}

// ============================ MPU6050 ===================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  Wire.begin();
  delay(100);
  // Wake up
  mpuWrite(0x6B, 0x00); // PWR_MGMT_1 = 0 (wake)
  // Set accel range to ±2g (0) and gyro to ±250 dps (0) (default)
  mpuWrite(0x1C, 0x00); // ACCEL_CONFIG
  mpuWrite(0x1B, 0x00); // GYRO_CONFIG
  delay(50);
}

bool readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register for ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return false;
  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();
  return true;
}

// ============================ SETUP & LOOP ===============================
void setup() {
  Serial.begin(115200);
  while (!Serial) ; // esperar Serial
  Serial.println(F("QuadControl - Arduino Uno"));
  Serial.println(F("Iniciando MPU6050 e GPS..."));

  // Initialize MPU
  mpuInit();

  // Initialize GPS serial (default GPS baud 9600)
  gpsSerial.begin(9600);

  // Initialize ESCs
  esc1.attach(PIN_M1);
  esc2.attach(PIN_M2);
  esc3.attach(PIN_M3);
  esc4.attach(PIN_M4);

  // Armar ESCs com sinal minimo por 1s (sem hélices)
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
  delay(1000);

  // read initial MPU to initialize angles
  if (readMPU6050()) {
    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;
    // compute angles from accelerometer (degrees)
    rollAcc = atan2(ay, az) * 180.0 / M_PI;
    pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
    rollEstimate = rollAcc;
    pitchEstimate = pitchAcc;
    yawEstimate = 0.0;
#ifdef USE_KALMAN
    kalmanRoll.angle = rollAcc;
    kalmanPitch.angle = pitchAcc;
#endif
  }

  lastMicros = micros();
  Serial.println(F("Setup completo. Tune os PID conforme necessário."));
}

// Main loop runs at variable rate; dt computed from micros()
void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) / 1000000.0f;
  if (dt <= 0) dt = 0.0001;
  lastMicros = nowMicros;

  // Read sensors
  if (!readMPU6050()) {
    // failed to read MPU this cycle
  } else {
    // Convert raw to physical units
    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;
    float gx = gx_raw / GYRO_SCALE; // deg/s
    float gy = gy_raw / GYRO_SCALE;
    float gz = gz_raw / GYRO_SCALE;

    // Compute accelerometer angles (degrees)
    rollAcc  = atan2(ay, az) * 180.0 / M_PI;
    pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;

    // Complementary filter for roll and pitch
#ifdef USE_KALMAN
    // Kalman uses accel angle and gyro rate
    rollEstimate  = kalmanRoll.getAngle(rollAcc,  gx, dt);
    pitchEstimate = kalmanPitch.getAngle(pitchAcc, gy, dt);
#else
    // Integrate gyro to angle
    rollEstimate  = ALPHA * (rollEstimate  + gx * dt) + (1.0f - ALPHA) * rollAcc;
    pitchEstimate = ALPHA * (pitchEstimate + gy * dt) + (1.0f - ALPHA) * pitchAcc;
#endif

    // Yaw - only gyro integration (drifts)
    yawEstimate += gz * dt;
    // normalize yaw to [-180,180]
    if (yawEstimate > 180.0) yawEstimate -= 360.0;
    if (yawEstimate < -180.0) yawEstimate += 360.0;
  }

  // Read GPS (non-blocking)
  gpsReadLoop();

  // === Control: compute PID outputs (units: degrees -> mix to microseconds later) ===
  float rollOut  = pid_controller_roll.compute(setRoll, rollEstimate, dt);
  float pitchOut = pid_controller_pitch.compute(setPitch, pitchEstimate, dt);
  float yawOut   = pid_controller_yaw.compute(setYaw, yawEstimate, dt);

  // Motor mixing (X configuration). Mix outputs into motor microsecond adjustments.
  // Define mixing signs carefully depending on motor layout & rotation directions.
  // A common X-mix (assuming motors: 1=FrontLeft, 2=FrontRight, 3=RearRight, 4=RearLeft):
  // m1 = throttle + pitch + roll + yaw
  // m2 = throttle + pitch - roll - yaw
  // m3 = throttle - pitch - roll + yaw
  // m4 = throttle - pitch + roll - yaw
  // Here rollOut/pitchOut/yawOut are in some units; we'll add them to throttle (center 1500)
  float mix1 = throttleInput + pitchOut + rollOut + yawOut;
  float mix2 = throttleInput + pitchOut - rollOut - yawOut;
  float mix3 = throttleInput - pitchOut - rollOut + yawOut;
  float mix4 = throttleInput - pitchOut + rollOut - yawOut;

  // Constrain mixes and send to ESCs
  int out1 = constrain((int)mix1, ESC_MIN, ESC_MAX);
  int out2 = constrain((int)mix2, ESC_MIN, ESC_MAX);
  int out3 = constrain((int)mix3, ESC_MIN, ESC_MAX);
  int out4 = constrain((int)mix4, ESC_MIN, ESC_MAX);

  esc1.writeMicroseconds(out1);
  esc2.writeMicroseconds(out2);
  esc3.writeMicroseconds(out3);
  esc4.writeMicroseconds(out4);

  // Telemetry via Serial (print at lower rate to avoid flooding)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) { // 10 Hz
    lastPrint = millis();
    Serial.print("R:");
    Serial.print(rollEstimate, 2);
    Serial.print(" P:");
    Serial.print(pitchEstimate, 2);
    Serial.print(" Y:");
    Serial.print(yawEstimate, 2);
    Serial.print(" | Th:");
    Serial.print(throttleInput);
    Serial.print(" M1:");
    Serial.print(out1);
    Serial.print(" M2:");
    Serial.print(out2);
    Serial.print(" M3:");
    Serial.print(out3);
    Serial.print(" M4:");
    Serial.print(out4);
    if (gpsFix) {
      Serial.print(" | GPS: ");
      Serial.print(gpsLatitude, 6);
      Serial.print(", ");
      Serial.print(gpsLongitude, 6);
    } else {
      Serial.print(" | GPS: NO FIX");
    }
    Serial.println();
  }

  // short delay to let other tasks run; overall loop dt is driven by micros()
  // but avoid busy spinning
  delay(2);
}

// ============================ UTILITÁRIOS PARA TUNING ============================
// Funções para ajustar PID e filtros em tempo de execução podem ser adicionadas,
// por exemplo lendo comandos via Serial e atualizando pid_controller_roll.params.Kp etc.
// Recomendo o seguinte fluxo para tunagem:
// 1) Ajustar Kp para resposta rápida (pequenas oscilações).
// 2) Adicionar Kd para amortecer.
// 3) Ajustar Ki lentamente (corrige offset).
// 4) Testar sem hélices e depois com hélices removidas até ter confiança.

