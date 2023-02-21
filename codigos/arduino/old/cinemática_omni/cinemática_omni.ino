//Controle Robô Omnidirecional
//Versão 1.0
//Criado por Clístenes Grizafis Bento

//-------------------------------------
//- Bibliotecas --------
//-------------------------------------

#include<math.h>

//-------------------------------------
//- Parâmetros e configurações --------
//-------------------------------------

// Características do robô
const float RAIO_RODA = 0.04;
const int PASSOS_REVOLUCAO = 800;
const float CIRCUNFERENCIA_RODA = (2 * M_PI * RAIO_RODA);

//--- Pinos motores ---
// Motor frontal esquerdo
#define PASSO_FE 3 //Passo eixo Y
#define DIR_FE 6 // Direção eixo Y

// Motor frontal direito
#define PASSO_FD 12 //Passo eixo A
#define DIR_FD 13 // Direção eixo A

// Motor traseiro esquerdo 
#define PASSO_TE 2 //Passo eixo X
#define DIR_TE 5 // Direção eixo X

// Motor traseiro esquerdo 
#define PASSO_TD 4 //Passo eixo Z
#define DIR_TD 7 // Direção eixo Z

//-------------------------------------
//- VARIAVEIS GLOBAIS --------
//-------------------------------------

//velocidade teste
 float vel = 0.016666;


//-------------------------------------
//- FUNÇÕES RELEVANTES --------
//-------------------------------------

//função de define o delay de cada passo dos moteores convertendo a velocidade recebida em milissegundos
float delayDosPassos(float velocidade){
  float rpm = (60/CIRCUNFERENCIA_RODA)*velocidade;
  Serial.print(rpm);
  Serial.print((60000/(float)PASSOS_REVOLUCAO)/rpm);
  return ((60000/(float)PASSOS_REVOLUCAO)/rpm); //60.000 é a quantidade de milisegundos que tem um minuto

};

 void setup() {

 Serial.begin(600);

 // Definindo os pinos dos motores como saída
 pinMode(PASSO_FE,OUTPUT); 
 pinMode(DIR_FE,OUTPUT);
 
 pinMode(PASSO_FD,OUTPUT); 
 pinMode(DIR_FD,OUTPUT);
 
 pinMode(PASSO_TE,OUTPUT); 
 pinMode(DIR_TE,OUTPUT);
 
 pinMode(PASSO_TD,OUTPUT); 
 pinMode(DIR_TD,OUTPUT);

 
 
 }
 void loop() {
// Habilita o motor para que se movimente em um sentido particular
 digitalWrite(DIR_FE,HIGH);
 digitalWrite(DIR_FD,HIGH);
 digitalWrite(DIR_TE,HIGH);
 digitalWrite(DIR_TD,HIGH);


digitalWrite(PASSO_FE,HIGH); 
delayMicroseconds(delayDosPassos(vel)); 
digitalWrite(PASSO_FE,LOW); 
delayMicroseconds(delayDosPassos(vel));
digitalWrite(PASSO_FD,HIGH); 
delayMicroseconds(delayDosPassos(vel)); 
digitalWrite(PASSO_FD,LOW); 
delayMicroseconds(delayDosPassos(vel));
digitalWrite(PASSO_TE,HIGH); 
delayMicroseconds(delayDosPassos(vel)); 
digitalWrite(PASSO_TE,LOW); 
delayMicroseconds(delayDosPassos(vel));
digitalWrite(PASSO_TD,HIGH); 
delayMicroseconds(delayDosPassos(vel)); 
digitalWrite(PASSO_TD,LOW); 
delayMicroseconds(delayDosPassos(vel));
}