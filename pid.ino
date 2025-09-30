#include <QTRSensors.h>

// ===================================
// 1. CONFIGURAÇÃO DE HARDWARE (PINAGEM)
// ===================================

// ---------- Sensores QTR (Linha) ----------
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];

// ---------- Motores (L298D) ----------
// Motor Direito
const int PIN_IN1_DIR = 4; // Sentido 1
const int PIN_IN2_DIR = 2; // Sentido 2
const int PIN_PWM_DIR = 5; // MA (PWM)

// Motor Esquerdo
const int PIN_IN3_ESQ = 8; // Sentido 1
const int PIN_IN4_ESQ = 7; // Sentido 2
const int PIN_PWM_ESQ = 6; // MB (PWM)

// ---------- Buzzer ----------
const int PIN_BUZZER = 13; 

// ---------- Sensor de Parada/Fim de Pista ----------
const int PIN_SENSOR_FIM = 12; 

// ===================================
// 2. CONFIGURAÇÃO DE CONTROLE (PID e VELOCIDADE)
// ===================================

// ---------- Velocidade Base ----------
const int VEL_BASE   = 40;  // Velocidade base para avanço
// Velocidade mínima para vencer a inércia
const int VEL_MIN    = 30; 

// ---------- Ganhos PID ----------
// Kp ajustado para 5.0 (Valor de teste inicial)
const float KP = 1.6;
; // <<< AJUSTE ESTE VALOR PRIMEIRO
const float KI = 0.5; 
const float KD = 7.0; 

// ---------- Parâmetros de Ajuste de Erro/PID ----------
const float POSICAO_IDEAL = 3500.0;
const float FATOR_ESCALA_ERRO = 35.0; 
const float FATOR_ESCALA_PID  = 3.0; 

// ===================================
// 3. VARIÁVEIS DE ESTADO (PID)
// ===================================

float PID_VALOR = 0;
float erroAtual = 0;
float somatorioErro = 0;
float deltaErro = 0;
float erroAnterior = 0;

// ===================================
// 4. SETUP
// ===================================

void setup() {
  Serial.begin(9600);

  // Configuração dos Pinos de Saída
  pinMode(PIN_IN1_DIR, OUTPUT);
  pinMode(PIN_IN2_DIR, OUTPUT);
  pinMode(PIN_IN3_ESQ, OUTPUT);
  pinMode(PIN_IN4_ESQ, OUTPUT);
  pinMode(PIN_PWM_DIR, OUTPUT);
  pinMode(PIN_PWM_ESQ, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); 

  // Configuração e Calibração dos Sensores QTR
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 9, 11}, SENSOR_COUNT); 
  
  // AÇÃO DO BUZZER ANTES DA CALIBRAÇÃO
  tocaBuzzer(200, 1000); 
  
  // Calibração
  Serial.println("Calibrando... Passe o robo sobre a linha.");
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  Serial.println("Calibracao concluida. Pronto para iniciar.");
  
  // AÇÃO DO BUZZER E ESPERA DE 2s APÓS A CALIBRAÇÃO
  tocaBuzzer(200, 1000); 
  delay(2000); 
}

// ===================================
// 5. FUNÇÕES DE CONTROLE
// ===================================

// ---------- Função: Buzzer ----------
void tocaBuzzer(int duracao_ms, int frequencia_hz) {
    tone(PIN_BUZZER, frequencia_hz, duracao_ms);
    delay(duracao_ms);
    noTone(PIN_BUZZER); // Garante que o som pare
}

// ---------- Função: calcular PID ----------
void calculaPID() {
  qtr.readCalibrated(sensorValues);
  uint16_t position = qtr.readLineWhite(sensorValues);

  // 1. Cálculo do Erro
  erroAtual = ((float)position - POSICAO_IDEAL) / FATOR_ESCALA_ERRO;

  // 2. Cálculo Integral (somatório limitado)
  somatorioErro += erroAtual;
  somatorioErro = constrain(somatorioErro, -100, 100);

  // 3. Cálculo Derivativo
  deltaErro = erroAtual - erroAnterior;
  erroAnterior = erroAtual;

  // 4. Cálculo do PID (P*erro + I*somatorio + D*delta)
  PID_VALOR = KP * erroAtual + KI * somatorioErro + KD * deltaErro;

  // Impressão de dados (Serial)
  // ... (Serial.print para monitoramento) ...
}


// ---------- Função: controle dos motores ----------
void controleMotor(int pwmEsq, int pwmDir) {
  // Limita o PWM entre -255 e 255
  pwmEsq = constrain(pwmEsq, -255, 255);
  pwmDir = constrain(pwmDir, -255, 255);

  // VERIFICAÇÃO DE VELOCIDADE MÍNIMA:
  // Garante que o motor avance no mínimo com VEL_MIN (50) se o PWM for positivo.
  if (pwmEsq > 0 && pwmEsq < VEL_MIN) {
      pwmEsq = VEL_MIN;
  }
  if (pwmDir > 0 && pwmDir < VEL_MIN) {
      pwmDir = VEL_MIN;
  }
  
  // --- Motor Esquerdo ---
  if (pwmEsq > 0) { // Avançar
    digitalWrite(PIN_IN3_ESQ, LOW);
    digitalWrite(PIN_IN4_ESQ, HIGH);
    analogWrite(PIN_PWM_ESQ, pwmEsq);
  } else if (pwmEsq < 0) { // Ré
    digitalWrite(PIN_IN3_ESQ, HIGH);
    digitalWrite(PIN_IN4_ESQ, LOW);
    analogWrite(PIN_PWM_ESQ, abs(pwmEsq));
  } else { // Parar (PWM = 0)
    digitalWrite(PIN_IN3_ESQ, LOW);
    digitalWrite(PIN_IN4_ESQ, LOW);
    analogWrite(PIN_PWM_ESQ, 0);
  }

  // --- Motor Direito ---
  if (pwmDir > 0) { // Avançar
    digitalWrite(PIN_IN1_DIR, LOW);
    digitalWrite(PIN_IN2_DIR, HIGH);
    analogWrite(PIN_PWM_DIR, pwmDir);
  } else if (pwmDir < 0) { // Ré
    digitalWrite(PIN_IN1_DIR, HIGH);
    digitalWrite(PIN_IN2_DIR, LOW);
    analogWrite(PIN_PWM_DIR, abs(pwmDir));
  } else { // Parar (PWM = 0)
    digitalWrite(PIN_IN1_DIR, LOW);
    digitalWrite(PIN_IN2_DIR, LOW);
    analogWrite(PIN_PWM_DIR, 0);
  }
}

// ---------- Função: parar motores (Função de referência, não usada no loop) ----------
void paraMotores() {
  digitalWrite(PIN_IN1_DIR, LOW);
  digitalWrite(PIN_IN2_DIR, LOW);
  digitalWrite(PIN_IN3_ESQ, LOW);
  digitalWrite(PIN_IN4_ESQ, LOW);
  analogWrite(PIN_PWM_DIR, 0);
  analogWrite(PIN_PWM_ESQ, 0);
  Serial.println("Motores PARADOS.");
}

// ===================================
// 6. LOOP PRINCIPAL
// ===================================

void loop() {
  calculaPID();

  // Aplicação do PID:
  float ajustePID = PID_VALOR / FATOR_ESCALA_PID; 

  int pwmEsq = VEL_BASE - ajustePID; 
  int pwmDir = VEL_BASE + ajustePID; 

  controleMotor(pwmEsq, pwmDir);

  // CORREÇÃO: Delay reduzido drasticamente para garantir movimento contínuo
  // e correções rápidas.
  delay(5); 
}
