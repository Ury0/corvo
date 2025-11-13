#include <Servo.h>
#include <Wire.h>

#define motor1 5
#define motor2 6
#define motor3 9
#define motor4 10

#define MPU_ADDR 0x68  // Endereço padrão do MPU6050

// ======= Classe PID =======
class PID {
  public:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;

    PID(float p, float i, float d, float sp = 0.0f) {
      Kp = p; Ki = i; Kd = d; setpoint = sp;
      integral = 0; prev_error = 0;
    }

    float compute(float input) {
      float error = setpoint - input;
      integral += error;
      float derivative = error - prev_error;
      prev_error = error;
      return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }
};

// ======= Instâncias PID =======
PID pidPitch(3.0, 0.04, 1.0);
PID pidRoll(3.0, 0.04, 1.0);

// ======= Servos (ESCs) =======
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

// ======= Variáveis de controle =======
int baseSpeed = 1600;
int minSpeed = 1000;
int maxSpeed = 2000;

// ======= Variáveis MPU =======
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float Ax, Ay, Az, Gx, Gy, Gz;
float roll_acc, pitch_acc;
float roll_cf = 0.0, pitch_cf = 0.0;

float Ax_offset = 0, Ay_offset = 0, Az_offset = 0;
float Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

unsigned long tempo_anterior = 0;
float dt;
const float alpha = 0.9; // Peso do giroscópio

// ======= Função de calibração =======
void calibrarMPU() {
  Serial.println("Iniciando calibração... mantenha o drone parado.");
  delay(2000);

  long Ax_sum = 0, Ay_sum = 0, Az_sum = 0;
  long Gx_sum = 0, Gy_sum = 0, Gz_sum = 0;

  const int n = 200; // número de amostras
  for (int i = 0; i < n; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Ax_sum += AcX;
    Ay_sum += AcY;
    Az_sum += AcZ;
    Gx_sum += GyX;
    Gy_sum += GyY;
    Gz_sum += GyZ;

    delay(5);
  }

  // Calcula offsets médios
  Ax_offset = Ax_sum / (float)n;
  Ay_offset = Ay_sum / (float)n;
  Az_offset = (Az_sum / (float)n) - 16384.0;  // ajusta gravidade (1g)
  Gx_offset = Gx_sum / (float)n;
  Gy_offset = Gy_sum / (float)n;
  Gz_offset = Gz_sum / (float)n;

  Serial.println("=== Calibração concluída ===");
  Serial.print("Ax_offset: "); Serial.println(Ax_offset);
  Serial.print("Ay_offset: "); Serial.println(Ay_offset);
  Serial.print("Az_offset: "); Serial.println(Az_offset);
  Serial.print("Gx_offset: "); Serial.println(Gx_offset);
  Serial.print("Gy_offset: "); Serial.println(Gy_offset);
  Serial.print("Gz_offset: "); Serial.println(Gz_offset);
  Serial.println("============================");
}

// ======= Setup =======
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializa MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0b00011000); Wire.endTransmission(); // ±2000°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0b00011000); Wire.endTransmission(); // ±16g

  //calibrando
  calibrarMPU();

  // Conecta os ESCs
  motorFL.attach(motor1);
  motorFR.attach(motor2);
  motorBL.attach(motor3);
  motorBR.attach(motor4);

  tempo_anterior = micros();

  // Inicializa motor
  motorFL.writeMicroseconds(minSpeed);
  motorFR.writeMicroseconds(minSpeed);
  motorBL.writeMicroseconds(minSpeed);
  motorBR.writeMicroseconds(minSpeed);
  Serial.print("1000");
  delay(7000);

  motorFL.writeMicroseconds(maxSpeed);
  motorFR.writeMicroseconds(maxSpeed);
  motorBL.writeMicroseconds(maxSpeed);
  motorBR.writeMicroseconds(maxSpeed);
  Serial.print("2000");

  delay(7000);

  motorFL.writeMicroseconds(baseSpeed);
  motorFR.writeMicroseconds(baseSpeed);
  motorBL.writeMicroseconds(baseSpeed);
  motorBR.writeMicroseconds(baseSpeed);
  Serial.print("1500");
  delay(7000);

}

// ======= Loop =======
void loop() {
  // Lê MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Aplica calibração
  Ax = (AcX - Ax_offset) / 2048.0;
  Ay = (AcY - Ay_offset) / 2048.0;
  Az = (AcZ - Az_offset) / 2048.0;
  Gx = (GyX - Gx_offset) / 16.4;
  Gy = (GyY - Gy_offset) / 16.4;
  Gz = (GyZ - Gz_offset) / 16.4;

  // Cálculo do tempo
  unsigned long tempo_atual = micros();
  dt = (tempo_atual - tempo_anterior) / 1e6;
  if (dt <= 0) dt = 0.01;
  tempo_anterior = tempo_atual;

  // Ângulos do acelerômetro
  roll_acc  = atan2(Ay, Az) * 180.0 / PI;
  pitch_acc = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

  // Integração giroscópio
  float roll_from_gyro  = roll_cf + Gx * dt;
  float pitch_from_gyro = pitch_cf + Gy * dt;

  // Filtro complementar
  roll_cf  = alpha * roll_from_gyro  + (1 - alpha) * roll_acc;
  pitch_cf = alpha * pitch_from_gyro + (1 - alpha) * pitch_acc;

  // PID
  float pitchOut = pidPitch.compute(pitch_cf);
  float rollOut  = pidRoll.compute(roll_cf);

  //Serial.print("pitch: "); Serial.print(pitchOut);
  //Serial.print("roll: "); Serial.print(rollOut);
  //Serial.print("\n");


  // Mistura nos motores
  int motorFL_out = constrain(baseSpeed + pitchOut + rollOut, minSpeed, maxSpeed);
  int motorFR_out = constrain(baseSpeed + pitchOut - rollOut, minSpeed, maxSpeed);
  int motorBL_out = constrain(baseSpeed - pitchOut + rollOut, minSpeed, maxSpeed);
  int motorBR_out = constrain(baseSpeed - pitchOut - rollOut, minSpeed, maxSpeed);

  // Envia aos ESCs
  motorFL.writeMicroseconds(motorFL_out);
  motorFR.writeMicroseconds(motorFR_out);
  motorBL.writeMicroseconds(motorBL_out);
  motorBR.writeMicroseconds(motorBR_out);

  // Serial debug
  //Serial.print("roll_cf: "); Serial.print(roll_cf, 2);
  //Serial.print(" | pitch_cf: "); Serial.print(pitch_cf, 2);
 // Serial.print("\n");
  Serial.print(" | motorFL: "); Serial.print(motorFL_out);
  Serial.print(" motorFR: "); Serial.print(motorFR_out);
  Serial.print(" motorBL: "); Serial.print(motorBL_out);
  Serial.print(" motorBR: "); Serial.println(motorBR_out);

  delay(20);
}
