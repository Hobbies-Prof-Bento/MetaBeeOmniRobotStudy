// Controle robô omnidirecional
// Versão 1.0.0
// Criado por Clístenes Grizafis Bento

//---------------------------------
//--Bibliotecas -------------------
//---------------------------------
#include <AccelStepper.h>

//---------------------------------
//--Parâmetros e configurações ----
//---------------------------------
const float PASSOS_REVOLUCAO = 800.0;
const float VEL_MAX = 3200.0; //velocidade máxima em passos por segundos
const float ACEL_MAX = 1000.0; //aceleração máxima em passos por segundos^2

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

}

void loop() {
  // put your main code here, to run repeatedly:
  moveRobo();

}
void moveRobo(){

  // ajustando velocidade do robô
  motor_fe.setSpeed(1000);
  motor_fd.setSpeed(1000);
  motor_te.setSpeed(1000);
  motor_td.setSpeed(1000);

  // mandando comando para robô mover motor na velocidade ajustada
  motor_fe.runSpeed();
  motor_fd.runSpeed();
  motor_te.runSpeed();
  motor_td.runSpeed();

}
