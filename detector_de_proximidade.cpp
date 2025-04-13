#include <Arduino.h>
#define PINO_LED_VERMELHO 5
#define PINO_LED_VERDE 4
#define PINO_LED_AMARELO 2
#define BUZZER 33
#define TRIG 14
#define ECHO 13
void setup() {
  Serial.begin(115200);
  pinMode(PINO_LED_VERDE, OUTPUT);
  pinMode(PINO_LED_AMARELO, OUTPUT);
  pinMode(PINO_LED_VERMELHO, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duracao = pulseIn(ECHO, HIGH);
  float distancia = duracao * 0.034 / 2;

  Serial.println("Distância: " + String(distancia) + " cm");
  if(distancia <= 45.00)
  {
    digitalWrite(PINO_LED_VERMELHO, HIGH);
    tone(BUZZER,262);
    delay(500);
    digitalWrite(PINO_LED_VERMELHO, LOW);
    
  }
  if(distancia <= 90.00 && distancia > 45.00)
  {
    digitalWrite(PINO_LED_AMARELO, HIGH);
    delay(500);
    digitalWrite(PINO_LED_AMARELO, LOW);
    noTone(BUZZER);

  }
  if(distancia > 90.00)
  {
    digitalWrite(PINO_LED_VERDE, HIGH);
    delay(500);
    digitalWrite(PINO_LED_VERDE, LOW);
    noTone(BUZZER);
  }
}
