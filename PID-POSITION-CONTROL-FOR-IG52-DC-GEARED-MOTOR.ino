#include<EEPROM.h>
#define ENCA 2 
#define ENCB 3 

#define PWMA 5
#define PWMB 6
#define ENA_A 8
#define ENA_B 9

#define CPR 93132  //Counts Per Revolution
#define FACTOR 87776*2  //FACTOR which is used to convert position from pulses to cm
#define JOYSTICK A0

long deg;
long pos;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

// PID constants
float kp = -0.2;
float ki = 0;
float kd = 0;



void setup() {
  Serial.begin(9600);

  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(ENA_A,OUTPUT);
  pinMode(ENA_B,OUTPUT);
  digitalWrite(ENA_A,HIGH);
  digitalWrite(ENA_B,HIGH);

  pos=map(EEPROM.read(0),127,255,0,526656); //Read the previous position of the motor shaft from the EEPROM memory so that the reference position do not change (Absolute Servo)


  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {

  /* 
  if(analogRead(JOYSTICK)>800){
    Serial.println("CW");
    deg+=10;    
  }
  else if(analogRead(A0)<300){
    Serial.println("CCW");
    deg-=10;        
  }
  if(deg>360)
    deg=360;
  if(deg<0)
    deg=0;
  */

  PID(FACTOR);
 

}

void setMotor(int dir, int pwmVal){
  if(dir == 1){
    analogWrite(PWMA,pwmVal);
    digitalWrite(PWMB,LOW);
  }
  else if(dir == -1){
    digitalWrite(PWMA,LOW);
    analogWrite(PWMB,pwmVal);
  }
  else{
    digitalWrite(PWMA,LOW);
    digitalWrite(PWMB,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

int PID(long deg){
  long target = deg;  
  

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  long e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;


  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }


  // signal the motor
  setMotor(dir,pwr);

  // store previous error
  eprev = e;
  //Serial.print("Target: ");

  Serial.print(target);
  Serial.print(" ");
  //Serial.print("Position: ");
  Serial.print(pos);
  Serial.println();
  //Storing current position in eeprom
  EEPROM.update(0, map(pos,0,526656,127,255));


}