#include <Servo.h>

#include <Wire.h>

#define m 9
#define MPU_ADDR 0x68  // Endereço padrão do MPU6050
unsigned long tempo_atual;
// Variáveis para armazenar valores crus do MPU6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// ==== Variáveis do PID ====
float Kp = 1.5;   // Ganho Proporcional
float Ki = 0.05;  // Ganho Integral
float Kd = 0.8;   // Ganho Derivativo

float setpoint = 0.0; // Ângulo desejado (ex.: 0 graus)
float entrada;        // Ângulo atual do MPU6050
float saida;          // Saída do PID

float erro, erro_anterior = 0;
float integral = 0;
float derivativo = 0;

unsigned long tempo_anterior = 0;

// ==== Pino de saída para motor (PWM) ====
Servo motor; // Saída PWM para controlar motor

void setup() {
  Serial.begin(9600);
  Wire.begin();

  
  // ==== Inicializa o MPU6050 ====
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // Registrador PWR_MGMT_1
  Wire.write(0);      // Define tudo como 0 (ativa o sensor)
  Wire.endTransmission(true);

  // Configuração do giroscópio (±2000°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission();

  // Configuração do acelerômetro (±16g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0b00011000);
  Wire.endTransmission();

  // Inicializa pino do motor
  motor.attach(m);
  tempo_atual = millis();
  
  motor.writeMicroseconds(1000);
  delay(7000);
}

void loop() {
  // ==== Leitura do MPU6050 ====
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Endereço inicial: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Lê 14 bytes de dados

  // Acelerômetro
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Temperatura
  Tmp = Wire.read() << 8 | Wire.read();

  // Giroscópio
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // ==== Conversão para valores físicos ====
  float Ax = AcX / 2048.0; // Aceleração em g
  float Ay = AcY / 2048.0;
  float Az = AcZ / 2048.0;

  float Gx = GyX / 16.4;   // Velocidade angular em °/s
  float Gy = GyY / 16.4;
  float Gz = GyZ / 16.4;

  float Temp = Tmp / 340.0 + 36.53; // Temperatura em °C

  // ==== Cálculo do ângulo usando acelerômetro (Pitch) ====
  entrada = atan2(Ay, Az) * 180 / PI;

  // ==== PID ====
  tempo_atual = millis();
  float dt = (tempo_atual - tempo_anterior) / 1000.0; // tempo em segundos
  tempo_anterior = tempo_atual;

  erro = setpoint - entrada;
  integral += erro * dt;
  derivativo = (erro - erro_anterior) / dt;

  saida = (Kp * erro) + (Ki * integral) + (Kd * derivativo);
  
  erro_anterior = erro;

  // Limita a saída do PID para faixa de PWM (0 a 255)
  int pwm = map(saida, -50, 50, 1000, 2000);

  // Aplica PWM no motor
  motor.writeMicroseconds(pwm);

  // ==== Serial Monitor ====
  Serial.print("Angulo (Pitch): "); Serial.print(entrada);
  Serial.print(" | Erro: "); Serial.print(erro);
  Serial.print(" | Saida PID: "); Serial.print(saida);
  Serial.print(" | PWM: "); Serial.println(pwm);

  delay(20); // Pequeno atraso para estabilidade
}
