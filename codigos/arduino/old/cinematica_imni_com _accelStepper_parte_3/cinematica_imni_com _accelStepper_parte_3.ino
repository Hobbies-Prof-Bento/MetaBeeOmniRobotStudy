// Controle robô omnidirecional
// Versão 1.0.0
// Criado por Clístenes Grizafis Bento

//---------------------------------
//--Bibliotecas -------------------
//---------------------------------
#include <AccelStepper.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
//---------------------------------
//--Parâmetros e configurações ----
//---------------------------------
const float PASSOS_REVOLUCAO = 800.0;
const float VEL_MAX = 3200.0; //velocidade máxima em passos por segundos
const float ACEL_MAX = 1000.0; //aceleração máxima em passos por segundos^2

const float RAIO_RODA = 0.04;
const float CIRCUNFERENCIA_RODA = (2*M_PI*RAIO_RODA);
const float SEPARACAO_RODAS_LARGURA = 0.25; //distância em metros da roda direita até a esquerda
const float SEPARACAO_RODAS_COMPRIMENTO = 0.16; //distância em metros da roda da frente para a roda de tras
const float GEOMETRIA_RODAS = SEPARACAO_RODAS_LARGURA + SEPARACAO_RODAS_COMPRIMENTO;

//--- Pinos motores
#define TIPOINTERFACEMOTOR 1

// Motor frontal Esquerdo
#define PASSO_FE 3 //passo eixo Y
#define DIR_FE 6 //direção eixo Y

// Motor frontal direito
#define PASSO_FD 12 //passo eixo A
#define DIR_FD 13 //direção eixo A

// Motor traseiro Esquerdo
#define PASSO_TE 2 //passo eixo X
#define DIR_TE 4 //direção eixo X

// Motor traseiro direito
#define PASSO_TD 4 //passo eixo Z
#define DIR_TD 7 //direção eixo Z

// instanciando motores como objetos da biblioteca AccelStepper
AccelStepper motor_fe = AccelStepper(TIPOINTERFACEMOTOR, PASSO_FE, DIR_FE);
AccelStepper motor_fd = AccelStepper(TIPOINTERFACEMOTOR,PASSO_FD,DIR_FD);
AccelStepper motor_te = AccelStepper(TIPOINTERFACEMOTOR,PASSO_TE,DIR_TE);
AccelStepper motor_td = AccelStepper(TIPOINTERFACEMOTOR,PASSO_TD,DIR_TD);

//---------------------------------
//--Funções callback ----
//---------------------------------
void callbackVelocidade(const geometry_msgs::Twist& msg){

  /* Twist
  {
    linear {
      x,y,z
    },
    angular{
      x,y,z      
    }
  }
   */

  float vel_linear_x = msg.linear.x;
  float vel_linear_y = msg.linear.y;
  float vel_angular_z = msg.angular.z;

  converteROSVelParaMotor(vel_linear_x, vel_linear_y, vel_angular_z);
}

//---------------------------------
//--Variáveis globais ----
//---------------------------------

float velocidade_fe = 0.0;
float velocidade_fd = 0.0;
float velocidade_te = 0.0;
float velocidade_td = 0.0;

ros::NodeHandle nh; //manipulador de nó ROS
ros::Subscriber<geometry_msgs::Twist> sub_velocidade("/cmd_vel", callbackVelocidade); // assinante do tópico ROS cmd_vel

void setup() {
  Serial.begin(57600);

  // Ajustando velocidade máxima de cada motor
  motor_fe.setMaxSpeed(VEL_MAX);
  motor_fd.setMaxSpeed(VEL_MAX);
  motor_te.setMaxSpeed(VEL_MAX);
  motor_td.setMaxSpeed(VEL_MAX);

  // Ajustando aceleração máxima
  motor_fe.setAcceleration(ACEL_MAX);
  motor_fd.setAcceleration(ACEL_MAX);
  motor_te.setAcceleration(ACEL_MAX);
  motor_td.setAcceleration(ACEL_MAX);

  nh.initNode(); // inicializa o nó ROS no arduino
  nh.subscribe(sub_velocidade); //inicia o assinante do tópico cmd_vel
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();  
  moveRobo();

}
float metrosParaPassos(float velocidade){
  return ((velocidade/CIRCUNFERENCIA_RODA)*PASSOS_REVOLUCAO);
}
void converteROSVelParaMotor(float linear_x, float linear_y, float angular_z){
  velocidade_fe = (linear_x - linear_y - angular_z*GEOMETRIA_RODAS);
  velocidade_fd = (linear_x + linear_y + angular_z*GEOMETRIA_RODAS);
  velocidade_te = (linear_x + linear_y - angular_z*GEOMETRIA_RODAS);
  velocidade_td = (linear_x - linear_y + angular_z*GEOMETRIA_RODAS);
}
void moveRobo(){

  // ajustando velocidade do robô
  motor_fe.setSpeed(metrosParaPassos(velocidade_fe));
  motor_fd.setSpeed(metrosParaPassos(velocidade_fd));
  motor_te.setSpeed(metrosParaPassos(velocidade_te));
  motor_td.setSpeed(metrosParaPassos(velocidade_td));

  // mandando comando para robô mover motor na velocidade ajustada
  motor_fe.runSpeed();
  motor_fd.runSpeed();
  motor_te.runSpeed();
  motor_td.runSpeed();

}
