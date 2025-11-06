#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11
#define DISPLAY_BAUD 9600


float ejex,ejey;
float ejex1,ejey1;

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin, true);


void setup() {

delay(1000);
  // put your setup code here, to run once:
Serial.begin(115200);
  // Set up two tasks to run independently.
pinMode(rxPin, INPUT);
pinMode(txPin, OUTPUT);

    
    // Set the baud rate for the SoftwareSerial object


mySerial.begin(9600);

mySerial.write(0x0E); //Borra display
mySerial.write(0x0C); //Cursor home
mySerial.print("  UTN FRA - LEIFRA  ");
  //Posicion Caracter Angulo X
mySerial.write(0x1B);
mySerial.write(0x48);
mySerial.write(20);

mySerial.write(142);
mySerial.print("x:");
  //Posicion Caracter Angulo Y
mySerial.write(0x1B);
mySerial.write(0x48);
mySerial.write(30);

mySerial.write(142);
mySerial.print("y:");
}

void loop() {
  // put your main code here, to run repeatedly:
 if (Serial.available() > 1)
  {
    ejex=Serial.parseFloat();
    ejey=Serial.parseFloat();
    ejex=ejex-80;
    ejey=ejey-80;
    
    if (ejex!=ejex1)
    {
    ejex1=ejex;
    mySerial.write(0x1B);
    mySerial.write(0x48);
    mySerial.write(23);
    mySerial.print("     ");
    mySerial.write(0x1B);
    mySerial.write(0x48);
    mySerial.write(23);
    mySerial.print(ejex,1);
    }
    if (ejey!=ejey1)
    {
    ejey1=ejey;
    mySerial.write(0x1B);
    mySerial.write(0x48);
    mySerial.write(33);
    mySerial.print("     ");
    mySerial.write(0x1B);
    mySerial.write(0x48);
    mySerial.write(33);
    mySerial.print(ejey,1);
    }
    
  }


  

    


}
