#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// === GPS ===
static const int RXPin = 8, TXPin = 7; 
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// === MPU6050 + Kalman ===
#define MPU6050_ADDR 0x68
#define ACCEL_SENS 16384.0f
#define GYRO_SENS 131.0f

// offsets calibrados
int16_t ax_off=0, ay_off=0, az_off=0;
int16_t gx_off=0, gy_off=0, gz_off=0;

struct Kalman1D {
  float angle = 0, bias = 0;
  float P00=1, P01=0, P10=0, P11=1;
  float Q_angle=0.001f, Q_bias=0.003f, R_measure=0.03f;

  float update(float newAngle, float newRate, float dt) {
    Serial.println("[Kalman] Update iniciado");
    float rate = newRate - bias;
    angle += dt * rate;
    P00 += dt*(dt*P11 - P01 - P10 + Q_angle);
    P01 -= dt*P11;
    P10 -= dt*P11;
    P11 += Q_bias * dt;
    float y = newAngle - angle;
    float S = P00 + R_measure;
    float K0 = P00 / S;
    float K1 = P10 / S;
    angle += K0 * y;
    bias += K1 * y;
    float P00_temp = P00;
    float P01_temp = P01;
    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;
    Serial.println("[Kalman] Update concluído");
    Serial.println(angle);
    return angle;
  }

  void setAngle(float a) { 
    Serial.println("[Kalman] SetAngle chamado");
    angle = a; 
  }
};

Kalman1D kfRoll, kfPitch, kfLat, kfLon;

unsigned long lastMicros = 0;

// --- Funções MPU ---
void setupMPU() {
  Serial.println("[MPU] Setup iniciado");
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0); 
  Wire.endTransmission(true);
  Serial.println("[MPU] Setup concluído");
}

void readMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Serial.println("[MPU] Leitura iniciada");
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14,true);
  ax=(Wire.read()<<8)|Wire.read();
  ay=(Wire.read()<<8)|Wire.read();
  az=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); // ignora temp
  gx=(Wire.read()<<8)|Wire.read();
  gy=(Wire.read()<<8)|Wire.read();
  gz=(Wire.read()<<8)|Wire.read();

  // aplica offsets
  ax -= ax_off;
  ay -= ay_off;
  az -= az_off;
  gx -= gx_off;
  gy -= gy_off;
  gz -= gz_off;

  Serial.println("acell");
  Serial.println(ax);Serial.println(ay);Serial.println(az);
  Serial.println("gyro");
  Serial.println(gx);Serial.println(gy);Serial.println(gz);

  Serial.println("[MPU] Leitura concluída");
}

// --- Calibração ---
void calibrateMPU() {
  Serial.println("[MPU] Calibração iniciada");
  long ax_sum=0, ay_sum=0, az_sum=0;
  long gx_sum=0, gy_sum=0, gz_sum=0;

  const int N = 2000; // nº de amostras
  Serial.println("Calibrando MPU6050... mantenha o drone parado na mesa");

  for(int i=0; i<N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(ax, ay, az, gx, gy, gz);

    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;

    delay(3);
  }

  ax_off = ax_sum / N;
  ay_off = ay_sum / N;
  az_off = (az_sum / N) - ACCEL_SENS; // ajusta Z para ~1g
  gx_off = gx_sum / N;
  gy_off = gy_sum / N;
  gz_off = gz_sum / N;

  

  Serial.println("acell off");
  Serial.println(ax_off);Serial.println(ay_off);Serial.println(az_off);
  Serial.println("gyro off");
  Serial.println(gx_off);Serial.println(gy_off);Serial.println(gz_off);
  Serial.println("[MPU] Calibração concluída!");
}

// --- PID ---
float Kp = 1.5, Ki = 0.0, Kd = 0.8;
float Kp_yaw = 2.0, Ki_yaw = 0.0, Kd_yaw = 1.0;

float errSumRoll=0, lastErrRoll=0;
float errSumPitch=0, lastErrPitch=0;
float errSumYaw=0, lastErrYaw=0;

float computePID(float setpoint, float input, float &errSum, float &lastErr, float dt, float Kp, float Ki, float Kd) {
  Serial.println("[PID] Cálculo iniciado");
  float error = setpoint - input;
  errSum += error * dt;
  float dErr = (error - lastErr) / dt;
  lastErr = error;
  Serial.println("[PID] Cálculo concluído");
  return Kp * error + Ki * errSum + Kd * dErr;
}

// --- Motores ---
#define M1 10
#define M2 5
#define M3 6
#define M4 9

Servo motor1, motor2, motor3, motor4;
int throttle = 1200; // valor inicial

// --- Setup ---
void setup(){
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println("[Setup] Iniciando...");

  setupMPU();
  calibrateMPU(); // <<=== CALIBRAÇÃO AUTOMÁTICA

  motor1.attach(M1);
  motor2.attach(M2);
  motor3.attach(M3);
  motor4.attach(M4);

  Serial.println("[Setup] Iniciando motores...");

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  delay(5000); // arm ESCs
  Serial.println("[Setup] IMU + GPS + Kalman + PID iniciado");
  lastMicros = micros();
}

// --- Loop ---
void loop() {
  Serial.println("===== LOOP =====");
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;

  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax,ay,az,gx,gy,gz);

  float axg = (float)ax/ACCEL_SENS;
  float ayg = (float)ay/ACCEL_SENS;
  float azg = (float)az/ACCEL_SENS;

  float gxds=(float)gx/GYRO_SENS;
  float gyds=(float)gy/GYRO_SENS;

  float rollAcc  = atan2f(ayg, azg) * 180/PI;
  float pitchAcc = atan2f(-axg, sqrtf(ayg*ayg+azg*azg)) * 180/PI;

  float roll  = kfRoll.update(rollAcc,gxds,dt);
  float pitch = kfPitch.update(pitchAcc,gyds,dt);

  Serial.print("[Loop] RollAcc: "); Serial.print(rollAcc);
  Serial.print(" PitchAcc: "); Serial.println(pitchAcc);

  // GPS leitura
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read()) && gps.location.isUpdated()) {
        double lat = gps.location.lat();
        double lon = gps.location.lng();
        double latF = kfLat.update(lat, 0, dt);
        double lonF = kfLon.update(lon, 0, dt);
        Serial.print("[GPS] Lat:"); Serial.print(latF,6);
        Serial.print("\tLon:"); Serial.println(lonF,6);
    }
  }

  // === PID controle ===
  float setRoll = 0, setPitch = 0, setYaw = 0;
  float rollPID  = computePID(setRoll, roll,  errSumRoll,  lastErrRoll,  dt, Kp, Ki, Kd);
  float pitchPID = computePID(setPitch, pitch, errSumPitch, lastErrPitch, dt, Kp, Ki, Kd);
  float yawPID   = computePID(setYaw, 0, errSumYaw, lastErrYaw, dt, Kp_yaw, Ki_yaw, Kd_yaw);

  Serial.println("[Loop] PID calculado");

  int m1Signal = throttle + pitchPID + rollPID - yawPID;
  int m2Signal = throttle + pitchPID - rollPID + yawPID;
  int m3Signal = throttle - pitchPID - rollPID - yawPID;
  int m4Signal = throttle - pitchPID + rollPID + yawPID;

  m1Signal = constrain(m1Signal, 1000, 2000);
  m2Signal = constrain(m2Signal, 1000, 2000);
  m3Signal = constrain(m3Signal, 1000, 2000);
  m4Signal = constrain(m4Signal, 1000, 2000);

  motor1.writeMicroseconds(m1Signal);
  motor2.writeMicroseconds(m2Signal);
  motor3.writeMicroseconds(m3Signal);
  motor4.writeMicroseconds(m4Signal);

  // Debug
  Serial.print("\tRoll:"); Serial.print(roll,2);
  Serial.print("\tPitch:"); Serial.print(pitch,2);
  Serial.print("\tM1:"); Serial.print(m1Signal);
  Serial.print("\tM2:"); Serial.print(m2Signal);
  Serial.print("\tM3:"); Serial.print(m3Signal);
  Serial.print("\tM4:"); Serial.println(m4Signal);

  Serial.println("===== FIM LOOP =====");
  delay(20);
}
