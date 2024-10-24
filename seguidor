//Componentes: Ponte H L298D e sensor InfraRed

const int BRANCO = 0;
const int PRETO = 1;
int contar = 8;
bool ponto = false;
//Veriáveis para sensores de linha e velocidade
int     sensorEsq, sensorLinha, sensorDir;
int     velMin = 100, velMed = 180, velMax = 200; //Velocidades dos motores
int     limiar = 400; //limiar do sensor de luz

//Variáveis para sensores de início e fim de pista
int     sensorFim;
int     contFim = 4; //Conta marcas de início, fim e cruzamentos
boolean flagFim = false; 
long    tempoTotal; //Tempo total da volta
long    tempoExtra = 2000; //Tempo extra para seguir linha


//Definicoes pinos Arduino ligados a entrada da Ponte H
int IN1 = 2;
int IN2 = 4;
int IN3 = 7;
int IN4 = 8;
int  MA = 5; //Motor direito
int  MB = 6; //Motor esquerdo

void setup(){
  
  //Define os pinos como saida  
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);  
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  pinMode(MA,  OUTPUT);  
  pinMode(MB,  OUTPUT);  
  
}

void para_frente(int vel) {
  //Configura velocidade dos motores
  analogWrite(MA, 130);   
  analogWrite(MB, 130);   
  //Aciona o motores 
  digitalWrite(IN1, LOW);  //A 
  digitalWrite(IN2, HIGH); //A
  digitalWrite(IN3, LOW);  //B
  digitalWrite(IN4, HIGH); //B   
}

void vira_direita(int vel) {
  //MotorA_frente
  analogWrite(MA, vel);
  digitalWrite(IN1, HIGH);  //A 
  digitalWrite(IN2, LOW); //A  
  
  //MotorB_tras 
  analogWrite(MB, 120);   
  digitalWrite(IN3, LOW); //B
  digitalWrite(IN4, HIGH);  //B   
}

void vira_esquerda(int vel) {
  //MotorA_tras
  analogWrite(MA, 120);
  digitalWrite(IN1, LOW);  //A 
  digitalWrite(IN2, HIGH); //A
   
  
  //MotorB_frente 
  analogWrite(MB, vel);   
  digitalWrite(IN3, HIGH); //B
  digitalWrite(IN4, LOW);  //B   
}

void para_motores() {
  //MotorA_para
  analogWrite(MA, 0);   
  analogWrite(MB, 0);
  digitalWrite(IN1, LOW);  //A 
  digitalWrite(IN2, LOW);  //A  
  //MotorB_para 
  digitalWrite(IN3, LOW);  //B
  digitalWrite(IN4, LOW);  //B   
}

void segueLinha(){      

  //le os sensores
  sensorDir   = digitalRead(11); 
  sensorLinha = digitalRead(12);
  sensorEsq   = digitalRead(13);


  //Se estiver na linha segue em frente 
  if ((sensorDir == PRETO && sensorEsq == PRETO) || (sensorDir == BRANCO && sensorEsq == BRANCO)){
    para_frente(velMin);    
  }
 
  //Se sensorDir acha linha vire para a direita
  if (sensorDir == BRANCO && sensorEsq == PRETO){
    vira_esquerda(velMin);
  }
 
  //Se sensorEsq acha linha vire para a esquerda
  if (sensorEsq == BRANCO && sensorDir == PRETO){
    vira_direita(velMin);
  }
  if(sensorLinha == BRANCO)
  {
    ponto = true;
  }
  if(sensorLinha == PRETO && ponto == true)
  {
    ponto = false;
    contar = contar-1;
  }

}
 
void loop(){
  if(contar > 0)
  {
    segueLinha();
  }
  if(contar == 0)
  {
    segueLinha();
    delay(1300);
    contar = contar-1;
    para_motores();
  }
 }
