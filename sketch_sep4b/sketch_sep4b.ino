#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// =========================
// Objetos
// =========================
Adafruit_MPU6050 mpu;
Servo motor1;  // Frente Esquerda
Servo motor2;  // Frente Direita
Servo motor3;  // Traseira Esquerda
Servo motor4;  // Traseira Direita

// =========================
// PID - Constantes de ajuste
// =========================
float Kp = 2.0;   // Proporcional
float Ki = 0.5;   // Integral
float Kd = 1.2;   // Derivativo

// =========================
// PID - Variáveis internas
// =========================
float erroPitch, erroRoll;
float somaErroPitch = 0, somaErroRoll = 0;
float erroAnteriorPitch = 0, erroAnteriorRoll = 0;

// =========================
// Configurações
// =========================
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar o MPU6050. Verifique as conexões!");
    while (1);
  }

  Serial.println("MPU6050 OK!");

  // Inicialização dos ESCs
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  motor4.attach(12);

  // Calibração inicial dos ESCs
  Serial.println("Calibrando ESCs - Máximo...");
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
  delay(3000);

  Serial.println("Enviando Mínimo...");
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(3000);

  Serial.println("Calibração concluída!");

  // Configurações do MPU
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// =========================
// Loop Principal
// =========================
void loop() {
  static unsigned long tempoAnterior = 0;
  unsigned long tempoAtual = millis();
  float deltaTempo = (tempoAtual - tempoAnterior) / 1000.0; // em segundos
  tempoAnterior = tempoAtual;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcula ângulo (em graus)
  float anguloPitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;  // Inclinação frontal/traseira
  float anguloRoll  = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI; // Inclinação lateral

  // =========================
  // Controle PID - PITCH
  // =========================
  float setPointPitch = 0; // Drone nivelado
  erroPitch = setPointPitch - anguloPitch;

  somaErroPitch += erroPitch * deltaTempo; // Integral
  float derivadaPitch = (erroPitch - erroAnteriorPitch) / deltaTempo; // Derivativo

  float PID_Pitch = (Kp * erroPitch) + (Ki * somaErroPitch) + (Kd * derivadaPitch);
  erroAnteriorPitch = erroPitch;

  // =========================
  // Controle PID - ROLL
  // =========================
  float setPointRoll = 0; // Drone nivelado
  erroRoll = setPointRoll - anguloRoll;

  somaErroRoll += erroRoll * deltaTempo; // Integral
  float derivadaRoll = (erroRoll - erroAnteriorRoll) / deltaTempo; // Derivativo

  float PID_Roll = (Kp * erroRoll) + (Ki * somaErroRoll) + (Kd * derivadaRoll);
  erroAnteriorRoll = erroRoll;

  // =========================
  // Potência Base
  // =========================
  int basePower = 1200; // Idle para manter motores girando

  // Ajuste individual por motor
  int m1 = basePower + PID_Pitch + PID_Roll;   // Frente Esquerda
  int m2 = basePower + PID_Pitch - PID_Roll;   // Frente Direita
  int m3 = basePower - PID_Pitch + PID_Roll;   // Traseira Esquerda
  int m4 = basePower - PID_Pitch - PID_Roll;   // Traseira Direita

  // Limita valores entre 1000 e 2000 µs
  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  // Envia sinal para motores
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);

  // =========================
  // Debug
  // =========================
  Serial.print("Pitch: ");
  Serial.print(anguloPitch);
  Serial.print("  PID_Pitch: ");
  Serial.print(PID_Pitch);

  Serial.print(" | Roll: ");
  Serial.print(anguloRoll);
  Serial.print("  PID_Roll: ");
  Serial.println(PID_Roll);

  Serial.print("Motores: ");
  Serial.print(m1); Serial.print(" ");
  Serial.print(m2); Serial.print(" ");
  Serial.print(m3); Serial.print(" ");
  Serial.println(m4);

  delay(20); // 50Hz
}
