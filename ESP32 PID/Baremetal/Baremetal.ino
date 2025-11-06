#include <Wire.h>

#define I2C_FREQ 400000
#define SDA_1 21
#define SCL_1 22
#define SDA_2 27
#define SCL_2 26
#define RXD2 18
#define TXD2 17
#define DISPLAY_BAUD 9600
#define FrecPWM 20000
#define ResolucionPWM 10
#define MAXPWM 1023


//VARIABLES SENSOR MAGNETICO ANGULO Y
int lowbyteY; //raw angle 7:0
word highbyteY; //raw angle 7:0 and 11:8
int rawAngleY; //final raw angle 
float degAngleY; //raw angle in degrees (360/4096 * [value between 0-4095])
float lastdegAngleY;

//VARIABLES SENSOR MAGNETICO ANGULO X
int lowbyteX; //raw angle 7:0
word highbyteX; //raw angle 7:0 and 11:8
int rawAngleX; //final raw angle 
float degAngleX; //raw angle in degrees (360/4096 * [value between 0-4095])

float angulox;
float anguloy;

//VARRIABLES MOTOR X
const int PWMPinx = 0;
int PWMValuex = 0;
const int directionPin1x = 33; 
const int directionPin2x = 32; 
int motorDirectionx = 0;

//VARRIABLES MOTOR Y
const int PWMPiny = 4;
int PWMValuey = 0;
const int directionPin1y = 14;
const int directionPin2y = 12;
int motorDirectiony = 0;

//PARAMETROS PID EJE X

float setpointx = 80;
float kpx = 225; 
float kix = 117.5; 
float kdx = 0; 
float controlSignalx = 0; 

//PARAMETROS PID EJE Y

float setpointy = 80;
float kpy = 50; 
float kiy = 45; 
float kdy = 0.1; 
float controlSignaly = 0;

//PID CALCULO TIEMPO
float previousTime = 0; //para calcular delta t
float currentTime = 0; //tiempó actual
float deltaTime = 0; //diferencia de tiempo

float errorValuex = 0; //error x
float edotx = 0; //derivativo x (de/dt)
float previousErrorx = 0; //para calcular el derivativo x (edotx)
float errorIntegralx = 0; //error integral x
float accionIntegralx = 0; //accion integral x

float errorValuey = 0; //error y
float edoty = 0; //derivativo y (de/dt)
float previousErrory = 0; //para calcular el derivativo y (edoty)
float errorIntegraly = 0; //error integral y
float accionIntegraly = 0; //accion integral y

//VARIABLES APLICACION TRACKING
float ejex;
float ejey;
int valorx;
int valory;

//
unsigned long previousTimedisplay = 0; 
unsigned long currentTimedisplay = 0; 

//Variables Debuging
int a=0;
int hab=0;
float salang;


TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1); 
HardwareSerial mySerial(2);



void setup() {
 
  Serial.begin(115200);
 

  mySerial.begin(DISPLAY_BAUD, SERIAL_8N1, RXD2, TXD2, 1);

  I2C_1.begin(SDA_1, SCL_1, I2C_FREQ);
  I2C_2.begin(SDA_2, SCL_2, I2C_FREQ);

  ledcAttach(PWMPinx, FrecPWM, ResolucionPWM);
  ledcAttach(PWMPiny, FrecPWM, ResolucionPWM);
  pinMode(directionPin1y, OUTPUT);
  pinMode(directionPin2y, OUTPUT);
  pinMode(directionPin1x, OUTPUT);
  pinMode(directionPin2x, OUTPUT);


  previousTimedisplay=millis();
}

void loop() 
{
SensorMagneticoY();
SensorMagneticoX();
CalculoPID();
DriveMotorX();
DriveMotorY();
ComunicacionPC();
//ComunicacionPCpruebaPID();
}


void SensorMagneticoY()
{
  I2C_1.beginTransmission(0x36); //connect to the sensor
  I2C_1.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  I2C_1.endTransmission(); //end transmission
  I2C_1.requestFrom(0x36, 1); //request from the sensor
  while(I2C_1.available() == 0); //wait until it becomes available 
  lowbyteY = I2C_1.read(); //Reading the data after the request
  I2C_1.beginTransmission(0x36);
  I2C_1.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  I2C_1.endTransmission();
  I2C_1.requestFrom(0x36, 1);
  while(I2C_1.available() == 0);  
  highbyteY = I2C_1.read();
  highbyteY = highbyteY << 8; //shifting to left
  rawAngleY = highbyteY | lowbyteY; //int is 16 bits (as well as the word)
  degAngleY = rawAngleY * 0.087890625; 
  if (degAngleY<30 || degAngleY>200)
  {degAngleY=30;}
  
}

void SensorMagneticoX()
{
  I2C_2.beginTransmission(0x36); //connect to the sensor
  I2C_2.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  I2C_2.endTransmission(); //end transmission
  I2C_2.requestFrom(0x36, 1); //request from the sensor
  while(I2C_2.available() == 0); //wait until it becomes available 
  lowbyteX = I2C_2.read(); //Reading the data after the request
  //11:8 - 4 bits
  I2C_2.beginTransmission(0x36);
  I2C_2.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  I2C_2.endTransmission();
  I2C_2.requestFrom(0x36, 1);
  while(I2C_2.available() == 0);  
  highbyteX = I2C_2.read();
  highbyteX = highbyteX << 8; 
  rawAngleX = highbyteX | lowbyteX;
  degAngleX = rawAngleX * 0.087890625; 
  if (degAngleX<30 || degAngleX>200)
  {degAngleX=30;}
}

void CalculoPID()
{
  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  //deltaTime = (currentTime - previousTime);
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //Error en X
  errorValuex = degAngleX - setpointx; //Current position - target position (or setpoint)
  //Serial.println(errorValuex);
  if (fabs(errorValuex)<0.1)
  {errorValuex=0;}
  //Serial.println(errorValuex);
  //Error en Y
  errorValuey = degAngleY - setpointy; //Current position - target position (or setpoint)
  if (fabs(errorValuey)<0.1)
  {errorValuey=0;}
  //Serial.println(errorValuey);
  //Error Der X
  edotx = (errorValuex - previousErrorx) / deltaTime; //edotx = de/dt - derivative term

  //Error Der Y
  edoty = (errorValuey - previousErrory) / deltaTime; //edotx = de/dt - derivative term

  //Error Int X
  errorIntegralx = errorIntegralx + (errorValuex * deltaTime); //integral term x - Newton-Leibniz, notice, this is a running sum!

  //Error Int Y
  errorIntegraly = errorIntegraly + (errorValuey * deltaTime); //integral term x - Newton-Leibniz, notice, this is a running sum!

  //Accion Integral X
  accionIntegralx=(errorIntegralx*kix);

  //Accion Integral Y
  accionIntegraly=(errorIntegraly*kiy);

  //Señal Control X
  controlSignalx = (kpx * errorValuex) + (kdx * edotx) + (accionIntegralx); //final sum, proportional term also calculated here
  
  //Señal Control Y
  controlSignaly = (kpy * errorValuey) + (kdy * edoty) + (accionIntegraly); //final sum, proportional term also calculated here
  
  //Se guarda el error anterior X
  previousErrorx = errorValuex; //save the error for the next iteration to get the difference (for edot)

  //Se guarda el error anterior Y
  previousErrory = errorValuey; //save the error for the next iteration to get the difference (for edot)

  //Anticlamping X
  if (controlSignalx > MAXPWM) //fabs() = floating point absolute value
  {
    controlSignalx = MAXPWM; //capping the PWM signal - 8 bit
  }

  if (controlSignalx < -MAXPWM)
  {
    controlSignalx = -MAXPWM;
  }

  //Anticlamping Y
  if (controlSignaly > MAXPWM) //fabs() = floating point absolute value
  {
    controlSignaly = MAXPWM; //capping the PWM signal - 8 bit
  }

  if (controlSignaly < -MAXPWM)
  {
    controlSignaly = -MAXPWM;
  }
  //Serial.println(controlSignalx);
  //Serial.println(controlSignaly);
    
}

void DriveMotorX() 
{
    //esp_task_wdt_reset();
 
  if (controlSignalx < 0) //negative value: CCW
  {
    motorDirectionx = -1;
  }
  if (controlSignalx > 0) //positive: CW
  {
    motorDirectionx = 1;
  }
  if (controlSignalx == 0) //0: STOP - this might be a bad practice when you overshoot the setpoint
  {
    motorDirectionx = 0;
  }
  
  PWMValuex = (int)fabs(controlSignalx); //PWM values cannot be negative and have to be integers
  
  if (PWMValuex > MAXPWM) //fabs() = floating point absolute value
  {
    PWMValuex = MAXPWM; //capping the PWM signal - 8 bit
  }

  if (PWMValuex < MINPWMXI && errorValuex != 0 && motorDirectionx == -1)
  {
    PWMValuex = MINPWMXI;
  }
  
  if (PWMValuex < MINPWMXD && errorValuex != 0 && motorDirectionx == 1)
  {
    PWMValuex = MINPWMXD;
  }

  if (motorDirectionx == 1) //-1 == CCW
  {
    digitalWrite(directionPin1x, LOW);
    digitalWrite(directionPin2x, HIGH);
  }
  if (motorDirectionx == -1) // == 1, CW
  {
    digitalWrite(directionPin1x, HIGH);
    digitalWrite(directionPin2x, LOW);
  }
  if (motorDirectionx == 0) // == 0, stop/break
  {
    digitalWrite(directionPin1x, LOW);
    digitalWrite(directionPin2x, LOW);
    PWMValuex = 0;   
  }
  //Serial.println(motorDirectionx);
  ledcWrite(PWMPinx, PWMValuex); 
}

void DriveMotorY() 
{
  int valorpwm;
   
 // esp_task_wdt_reset();
  if (controlSignaly < 0) //negative value: CCW
  {
    motorDirectiony = -1;
  }
  if (controlSignaly > 0) //positive: CW
  {
    motorDirectiony = 1;
  }
  if (controlSignaly == 0) //0: STOP - this might be a bad practice when you overshoot the setpoint
  {
    motorDirectiony = 0;
  }
  //---------------------------------------------------------------------------
  //Speed
  PWMValuey = (int)fabs(controlSignaly); //PWM values cannot be negative and have to be integers
  
  if (PWMValuey > MAXPWM) //fabs() = floating point absolute value
  {
    PWMValuey = MAXPWM; //capping the PWM signal - 8 bit
  }

  //Calculo PWM MIN PARA EL SETPOINT ELEGIDO

  //MINPWMYI=453+((1.4)*setpointy);
  //MINPWMYD=842.5-((2.64)*setpointy);

  if (PWMValuey < MINPWMYI && errorValuey != 0 && motorDirectiony == -1)
  {
    PWMValuey = MINPWMYI;
  }

  if (PWMValuey < MINPWMYD && errorValuey != 0 && motorDirectiony == 1)
  {
    PWMValuey = MINPWMYD;
  }
  
  if (motorDirectiony == 1) //-1 == CCW
  {
    digitalWrite(directionPin1y, LOW);
    digitalWrite(directionPin2y, HIGH);
  }
  if (motorDirectiony == -1) // == 1, CW
  {
    digitalWrite(directionPin1y, HIGH);
    digitalWrite(directionPin2y, LOW);
  }
  if (motorDirectiony == 0) // == 0, stop/break
  {
    digitalWrite(directionPin1y, LOW);
    digitalWrite(directionPin2y, LOW);
    PWMValuey = 0;
  }
  if (PWMValuey != valorpwm)
  //Serial.println(motorDirectiony);
  {ledcWrite(PWMPiny, PWMValuey);
  valorpwm=PWMValuey;}
  
}

void ComunicacionPCpruebaPID() 
{
    if(a==1)
    {
      if (Serial.available() > 1)
      {
      setpointy=Serial.parseInt();
      hab=1;
      }
    }

     //Serial.println("A");

  if(a==0)
    {
    if (Serial.available() > 1)
    {
      a=1;
      //kpx=Serial.parseInt();
      //kix=Serial.parseInt();
      //kdx=Serial.parseInt();
      kpy=Serial.parseFloat();
      kiy=Serial.parseFloat();
      kdy=Serial.parseFloat();
      Serial.println(kpy);
      Serial.println(kiy);
      Serial.println(kdy);
      //ejey=Serial.parseInt();
    }
    /*
    ejex=Serial.parseInt();
    ejey=Serial.parseInt();
    if(ejex<=125 && ejex>=35)
    {setpointx=ejex;}
    if(ejey<=125 && ejey>=35)
    {setpointy=ejey;}
    */
    //setpointx=setpointx+ejex;
    //setpointy=setpointy+ejey;
    }

  if (hab=1 && salang!=degAngleY)
  {
    Serial.println(degAngleY);
    salang=degAngleY;
  }

  

  //Serial.println(degAngleY);
  //Serial.println(degAngleX);

}

void ComunicacionPC() 
{
    if (Serial.available() > 1)
    {
    ejex=Serial.parseFloat();
    ejey=Serial.parseFloat();
    if(ejex<=109 && ejex>=51)
    {setpointx=ejex;}
    if(ejey<=102 && ejey>=58)
    {setpointy=ejey;}
    
    //setpointx=setpointx+ejex;
    //setpointy=setpointy+ejey;
    }
  currentTimedisplay=millis();
  if (currentTimedisplay>previousTimedisplay+250)
  {
  previousTimedisplay=currentTimedisplay;
  Serial.print(degAngleX);
  Serial.print(" ");
  Serial.println(degAngleY);
  }

}


