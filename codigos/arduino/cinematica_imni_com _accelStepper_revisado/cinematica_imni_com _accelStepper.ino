//Controle Robô Omnidirecional
//Versão 1.0
//Criado por Clístenes Grizafis Bento

//-------------------------------------
//- Bibliotecas --------
//-------------------------------------
#include <AccelStepper.h>
#include <math.h>
#include<ros.h>
#include <geometry_msgs/Twist.h>

//-------------------------------------
//- Parâmetros e configurações --------
//-------------------------------------

// Características do robô
const float RAIO_RODA = 0.04;
const int PASSOS_REVOLUCAO = 800;
const float CIRCUNFERENCIA_RODA = (2 * M_PI * RAIO_RODA);
const float VEL_MAX = 3200.0; //velocidade máxima em passos por segundo
const float MAX_ACEL = 1000.0; // aceleração máxima em passos por segundos^2
const float SEPARACAO_RODAS_LARGURA = 0.25;
const float SEPARACAO_RODAS_COMPRIMENTO = 0.16;
const float GEOMETRIA_RODAS = SEPARACAO_RODAS_LARGURA + SEPARACAO_RODAS_COMPRIMENTO;

//--- Pinos motores ---
#define TIPOINTERFACEMOTOR 1 //tipo de interface do motor
//#define PINOATIVACAO 8 //pino que deixa ativado os motores de passo

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

// instanciando motores como objetos da biblioteca AccelStepper
AccelStepper motor_fe = AccelStepper(TIPOINTERFACEMOTOR, PASSO_FE, DIR_FE);
AccelStepper motor_fd = AccelStepper(TIPOINTERFACEMOTOR, PASSO_FD, DIR_FD);
AccelStepper motor_te = AccelStepper(TIPOINTERFACEMOTOR, PASSO_TE, DIR_TE);
AccelStepper motor_td = AccelStepper(TIPOINTERFACEMOTOR, PASSO_TD, DIR_TD);

//-------------------------------------
//- FUNÇÕES CALLBACK --------
//-------------------------------------

void callbackVelocidade(const geometry_msgs::Twist& msg){
  float vel_linear_x = msg.linear.x;
  float vel_linear_y = msg.linear.y;
  float vel_angular_z = msg.angular.z;
  convertROSVelParaMotor(vel_linear_x, vel_linear_y, vel_angular_z);
}

//-------------------------------------
//- VARIAVEIS GLOBAIS --------
//-------------------------------------

float velocidade_fe = 0.0;
float velocidade_fd = 0.0;
float velocidade_te = 0.0;
float velocidade_td = 0.0;


ros::NodeHandle nh; // manipulador de nó ROS
ros::Subscriber<geometry_msgs::Twist> sub_velocity("/cmd_vel", callbackVelocidade); // subscriber do tópico ROS cmd_vel



void setup() {
  
  // ativando motores
  //pinMode(PINOATIVACAO, OUTPUT);
  //digitalWrite(PINOATIVACAO, LOW);

  // comunicação serial para debug
  Serial.begin(57600);

  // Ajustando velocidade máxima de cada motor
  motor_fe.setMaxSpeed(VEL_MAX);
  motor_fd.setMaxSpeed(VEL_MAX);
  motor_te.setMaxSpeed(VEL_MAX);
  motor_td.setMaxSpeed(VEL_MAX);

  // Set the maximum acceleration
  motor_fe.setAcceleration(MAX_ACEL);
  motor_fd.setAcceleration(MAX_ACEL);
  motor_te.setAcceleration(MAX_ACEL);
  motor_td.setAcceleration(MAX_ACEL);

  nh.initNode(); //inicializa o nó arduino
  nh.subscribe(sub_velocity); //inicia o assinante do tópico cmd_vel
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //convertROSVelParaMotor(0.0, 0.0, 0.0);
  nh.spinOnce(); // mantém o nó ROS em execussão
   moveRobo(); // move_robo
}

float metrosParaPassos(float velocidade){
  return((velocidade/CIRCUNFERENCIA_RODA)*(float)PASSOS_REVOLUCAO);
}
void convertROSVelParaMotor(float linear_x, float linear_y, float angular_z){    

  velocidade_fe = (linear_x - linear_y - angular_z * GEOMETRIA_RODAS);  
  velocidade_fd = (linear_x + linear_y + angular_z * GEOMETRIA_RODAS);
  velocidade_te = (linear_x + linear_y - angular_z * GEOMETRIA_RODAS);
  velocidade_td = (linear_x - linear_y + angular_z * GEOMETRIA_RODAS);  
}
void moveRobo(){
  // Set the speed in steps per second:
  motor_fe.setSpeed(metrosParaPassos(velocidade_fe));
  motor_fd.setSpeed(metrosParaPassos(velocidade_fd));
  motor_te.setSpeed(metrosParaPassos(velocidade_te));
  motor_td.setSpeed(metrosParaPassos(velocidade_td));
  // Step the motor with a constant speed as set by setSpeed():
  motor_fe.runSpeed();
  motor_fd.runSpeed();
  motor_te.runSpeed();
  motor_td.runSpeed();
}