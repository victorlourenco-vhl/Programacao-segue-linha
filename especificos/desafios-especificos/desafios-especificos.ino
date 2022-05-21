/* Finalizacao: 20/05/2022 21:28
 * Revisao final: 21/05/2022
 * Nome: Victor Hugo Lourenco
 */

// Obs: Estou simulando a utilzacao de um Arduino Mega com Ponte H
// Obs: As explicacoes se encontram na forma de comentario em suas respectivas funcoes.

/* 
 
PRINCIPAIS FUNCOES REFERENTE A CADA DESAFIO

-> DESAFIO 1
  * void leituraSensores();
  * void verificacaoErros();
  * void calculoPID();
  * void controleVelocidade();

-> DESAFIO 2
  * void pararRobo();

-> DESAFIO 3
  * void contador();
  * void calculoRPM();
  * void distaciaPercorrida();
  
-> DESAFIO 4
  * void deboucing();

*/


/* DEFININDO PINOS DO ARDUINO */

// Sensores de controle
#define S1 2
#define S2 3
#define S3 4
#define S4 5
#define S5 6
// Sensores de parada
#define S6 6
#define S7 7

// Pinos dos motores - Obs: Cada motor utiliza 2 portas para inverter a polaridade
#define motor1A  8   
#define motor1B  9  
#define motor2A  10
#define motor2B  11
#define ENA      12 // PWM
#define ENB      13 // PWM

// Definindo pino do encoder
#define encoder 14

// Pino botao
#define botao 15

/* VARIAVEIS AUXILIARES */

// PID - Não sei quais seriam os valores apropriados para o segue linha
float Ki = 0, Kp = 0, Kd = 0;
float P = 0, I = 0, D = 0, PID = 0;
float ERRO, erroAnterior;

// Velocidade dos motores com base no PWM
unsigned int velBaseE = 230;
unsigned int velBaseD = 230;
unsigned int velMotorE = 0;
unsigned int velMotorD = 0;

// Array de sensores
short int sensor[7];

// Contador para definir a parada do robo
unsigned int contadorParada = 0;

/* VARIAVEIS PARA CONTROLE DO ENCODER */

unsigned int RPM = 0;
/* Obs: Utiliza volatile quando a 
        variavel pode ser mudada por algo 
        além do controle de codigo(ex: interrupcoes). */
volatile byte pulsos = 0;
unsigned long timeold = 0;
// Varias para calculo de distancia
unsigned int PPM = 0; // Pulsos por metro
// O PPR varia de encoder para encoder
unsigned int PPR = 20; // Pulso por rotacao
const float pi = 3.14;
unsigned int R = 0.03; // Raio da roda em metros

// Correcao do efeito boucing
int estadoBotao;         // variável para vericar o estado do botão
int contadorClicks = 0;     // variável para contar o número de clicks no botão
unsigned long ultimoTempoDebouce = 0; // última vez que o botão foi pressionado
unsigned long debounceDelay = 70;   // O intervalo, igual ao delay do código anterio
      
/* PROTOTIPOS DE FUNCAO */

void leituraSensores();
void verificacaoErros();
void calculoPID();
void controleVelocidade();
void pararRobo();
void contador();
void calculoRPM();
void distaciaPercorrida();
void deboucing();


void setup(){
  // Iniciando o monitor serial
  Serial.begin(9600);
  
  // Definido estado dos sensores
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  
  // Definido estado dos motores
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Definindo estado do encoder
  pinMode(encoder, INPUT);

  // Definindo estado do botao
  pinMode(botao, INPUT_PULLUP); // 0 = HIGH e 1 = LOW

  // Iniciando os motores parados
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Contabiliza os pulsos por meio de interrupcoes
  attachInterrupt(0, contador, RISING);

}
void loop(){
  void leituraSensores();
  void verificacaoErros();
  void calculoPID();
  void controleVelocidade();
  void pararRobo();
  void contador();
  void calculoRPM();
  void distaciaPercorrida();
  void deboucing();
}

/**** TABELA DE ERROS **** 
    S1 S2 S3 S4 S5 ERRO
    0  0  0  0  1   4
    0  0  0  1  1   3
    0  0  0  1  0   2
    0  0  1  1  0   1
    0  0  1  0  0   0
    0  1  1  0  0  -1
    0  1  0  0  0  -2
    1  1  0  0  0  -3
    1  0  0  0  0  -4
**************************/


void leituraSensores(){
  sensor[0] = digitalRead(S1);
  sensor[1] = digitalRead(S2);
  sensor[2] = digitalRead(S3);
  sensor[3] = digitalRead(S4);
  sensor[4] = digitalRead(S5);
  sensor[5] = digitalRead(S6);
  sensor[6] = digitalRead(S7);
}

// Funcao para verificao de errros
void verificacaoErros(){

  leituraSensores();

  // Verificando os erros com base na tabela da apostila de treinamento
  
  if(sensor[0] == 0  && sensor[1] == 0  && sensor[2] == 0  && sensor[3] == 0  && sensor[4] == 1){
    ERRO = 4;
  }
  else if(sensor[0] == 0  && sensor[1] == 0  && sensor[2] == 0  && sensor[3] == 1  && sensor[4] == 1){
    ERRO = 3;
  }
  else if(sensor[0] == 0  && sensor[1] == 0  && sensor[2] == 0  && sensor[3] == 1  && sensor[4] == 0){
    ERRO = 2;
  }
  else if(sensor[0] == 0  && sensor[1] == 0  && sensor[2] == 1  && sensor[3] == 1  && sensor[4] == 0){
    ERRO = 1;
  }
  else if(sensor[0] == 0  && sensor[1] == 0  && sensor[2] == 1  && sensor[3] == 0  && sensor[4] == 0){
    ERRO = 0;
  }
  else if(sensor[0] == 0  && sensor[1] == 1  && sensor[2] == 1  && sensor[3] == 0  && sensor[4] == 0){
    ERRO = -1;
  }
  else if(sensor[0] == 0  && sensor[1] == 1  && sensor[2] == 0  && sensor[3] == 0  && sensor[4] == 0){
    ERRO = -2;
  }
  else if(sensor[0] == 1  && sensor[1] == 1  && sensor[2] == 0  && sensor[3] == 0  && sensor[4] == 0){
    ERRO = -3;
  }
  else if(sensor[0] == 1  && sensor[1] == 0  && sensor[2] == 0  && sensor[3] == 0  && sensor[4] == 0){
    ERRO = -4;
  }
}


void calculoPID(){
  // Limites escolhidos com base no PWM
  if(ERRO = 0){
    I = 0;
  }
  else if(I > 255){
    I = 255;
  }
  else if(I < -255){
    I = -255;
  }
  P = ERRO;
  I = I + ERRO;
  D = ERRO - erroAnterior;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = ERRO; 
  
}

void controleVelocidade(){
  // Corrigi a velocidade no motor direitor
  if(PID >= 0){
    velMotorE = velBaseE;
    velMotorD = velBaseD - PID;
  }
  // Corrigi a velocidade no motor esquerdo
  else{
    velMotorE = velBaseE - PID;
    velMotorD = velBaseD;
  }
  // Define o sentido do robo e passa a velocidade com base no PID
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(ENA, velMotorE);
  analogWrite(ENB, velMotorD);  
}

// Funcao para parar o robo com base na contagem das marcações da pista.
void pararRobo(){

   leituraSensores();
  
  // Utilizando apenas o sensor do lado direito
  if(sensor[6] == 1){
    contadorParada += 1;
    
    // Parada apos a leitura de duas marcações
    if (contadorParada == 2){
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, LOW);
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
    }
  }
}

void contador(){
  pulsos++;
}

void calculoRPM(){
  /* EXPLICACAO
   * Descricao:
     Atraves da interrupcao externa do arduino
     o codigo vai calcular o RPM a cada um minuto.
   
   * Sobre a forlmula:
     O primeiro parenteses e responsavel por calcular os pulsos por minuto.
     Ja o segundo pararentes e divisor do primeiro e serve para calcular
     de forma indireta o tempo decorrido e ao multiplicar pela quantidade de pulsos
     descobre o RPM. 
   */
   
  if (millis() - timeold >= 1000)
  {
    //Desabilita interrupcao ao fazer as contas
    detachInterrupt(0);
    RPM = (60 * 1000 / PPR) / (millis() - timeold) * pulsos;
    timeold = millis();
    pulsos = 0;

    //Mostra o valor de RPM no serial monitor
    Serial.print("RPM = ");
    Serial.println(RPM, DEC);
    attachInterrupt(0, contador, RISING);
  }
}

void distaciaPercorrida(){
  /* EXPLICACAO
   * Formula para calcular a distancia percorrida em metros
   * Fómula: PPM = PPR/π.R.2
   * Onde:  
   * PPM: Pulsos por metro
   * PPR: Pulsos por rotacao
   * π: Contante de pi(3.14...)
   * R: Raio da roda
   * Obs: Atraves do PPM e possivel descobrir a distacia percorrida em metros
   */
   
   PPM = PPR / (pi * R * 2);
   Serial.print("DP: "); // Distacia percorrida
   Serial.println(PPM + "m");
     
}

void deboucing(){
  /* EXPLICACAO
   * Efeito boucing:
     O efeito boucing e uma deformacao mecanica que costuma ocorrer
     nas laminas internas de push buttons. Esse efeito decorre 
     de sinais aleatorios que sao gerados pelas laminas ao soltar
     o botao pressionado.
   */
   int estadoBotao = digitalRead(botao);
 
   if (estadoBotao == 0) {
    if ((millis() - ultimoTempoDebouce) > debounceDelay) {
      contadorClicks++;
    }
    ultimoTempoDebouce = millis();
  }

  Serial.print("Clicks: ");
  Serial.println(contadorClicks);
  
}
