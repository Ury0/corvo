# 🚁 Projeto Quadricóptero com Arduino UNO  

Este projeto implementa o controle de um **quadricóptero** utilizando **Arduino UNO**, **sensor MPU6050 (acelerômetro + giroscópio)**, **GPS**, **filtro de Kalman** e **controladores PID** para estabilização de voo.  

---

## 📌 Objetivo  
- Construir um sistema embarcado capaz de **estabilizar um drone em voo**.  
- Utilizar sensores **IMU (MPU6050)** para leitura da orientação (roll, pitch, yaw).  
- Integrar um **GPS** para monitoramento da posição.  
- Aplicar **filtro de Kalman** para fusão dos sensores e redução de ruído.  
- Implementar **controladores PID** para corrigir a inclinação do drone ajustando a velocidade dos motores.  

---

## 🛠️ Componentes Utilizados  
- **Arduino UNO**  
- **MPU6050 (Acelerômetro + Giroscópio via I²C)**  
- **Módulo GPS (GY-NEO6MV2 ou similar)**  
- **4 Motores Brushless + 4 ESCs (Controladores Eletrônicos de Velocidade)**  
- **Frame Quadricóptero**  
- **Bateria LiPo**  
- **Jumpers e Protoboard (para testes)**  

---

## 🔌 Ligações Principais  

### **MPU6050 (I²C)**  
- VCC → 5V (ou 3.3V, dependendo do módulo)  
- GND → GND  
- SDA → A4 (Arduino UNO)  
- SCL → A5 (Arduino UNO)  

### **GPS (Serial)**  
- TX → RX (pino 0 do Arduino UNO)  
- RX → TX (pino 1 do Arduino UNO)  
- VCC → 5V  
- GND → GND  

### **Motores / ESCs**  
- Sinais PWM nos pinos: **3, 5, 6 e 9**  

---

## ⚙️ Funcionalidades do Código  
- Leitura da IMU (aceleração e giroscópio).  
- Aplicação do **Filtro de Kalman** para fusão dos dados.  
- Cálculo de **Roll** e **Pitch** em tempo real.  
- Leitura de posição via GPS (Lat/Lon filtrados por Kalman).  
- Controle **PID** de estabilização (Roll, Pitch e Yaw).  
- Envio de sinais PWM aos ESCs para manter o equilíbrio.  

---

## 📈 Calibração dos Sensores  
1. Com o drone parado e nivelado:  
   - Medir offsets do **giroscópio** (valores médios quando está parado).  
   - Medir offsets do **acelerômetro** (ajustar para que em repouso indique corretamente a gravidade).  
2. Inserir os valores no código antes do loop principal.  

---

## 🚀 Próximos Passos  
- Adicionar controle remoto (RC ou via rádio 2.4GHz).  
- Implementar **altitude hold** com sensor barométrico (BMP180/BMP280).  
- Testes práticos em campo aberto.  

---

## 📷 Demonstração  
*(Adicionar fotos do protótipo ou esquemático de ligação aqui)*  

---

## 👤 Autor  
Projeto desenvolvido por **[Seu Nome]** como parte de estudos em **sistemas embarcados, controle PID e robótica aérea**.  
