
#include <Wire.h>              //including libraries of I2C
#include "SR04.h"
#include <IRremote.h>          //including libraries of remote control
#define RECV_PIN  12        //pin 12 of IR remoter control receiver
#include <Servo.h>
IRrecv irrecv(RECV_PIN);      //defining pin 12 of IR remoter control
Servo myservo;
decode_results res;
decode_results results;         //cache of decode of IR remoter control
#define IR_Go       0x00511DBB //going forward
#define IR_Back     0x00A3C8EDDB  //going backward
#define IR_Left     0x0052A3D41F  //turning left
#define IR_Right    0x0020FE4DBB  //turning right
#define IR_Stop     0x00D7E84B1B  //stop 
#define IR_Servo_L  0x00C101E57B  //motor turning left
#define IR_Servo_R  0x0097483BFB  //motor turning right
#define IR_Speed_UP     0x00F0C41643 //increasing speed
#define IR_Speed_DOWN   0x009716BE3F //decreasing speed
#define IR_XunJi_Mode   0x003D9AE3F7
#define IR_Self_Control  0x006182021B  //ultrasonic distance detecting 
#define IR_IR_Control  0x008C22657B
#define IR_Bluetooth_Control  0x00488F3CBB
#define IR_ESC      0x003EC3FC1B  //quitting from remote control
//////////////////////////////////////////////////
#define SensorLeft    9   //sensor left pin of line tracking module 
#define SensorMiddle  10   //sensor middle pin of line tracking module
#define SensorRight   11  //sensor right pin of line tracking module
unsigned char SL;        //state of left sensor of line tracking module
unsigned char SM;        //state of middle sensor of line tracking module
unsigned char SR;        //state of right sensor of line tracking module
int inputPin=A0;  // ultrasonic module   ECHO to A0
int outputPin=A1;  // ultrasonic module  TRIG to A1
unsigned char Bluetooth_val;       // ultrasonic module  TRIG to A1
unsigned long Key;
unsigned long Key1;
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
int flag=0;
unsigned char Lpwm_val = 120; //initialized left wheel speed at 100
unsigned char Rpwm_val = 120; //initialized right wheel speed at 100
int Car_state=0;             //the working state of car
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao=90;    //initialized angle of motor at 90°
void Sensor_IO_Config()     //IO initialized function of three line tracking , all setting at input
{
  pinMode(SensorLeft,INPUT);
  pinMode(SensorMiddle,INPUT);
  pinMode(SensorRight,INPUT);
}
void Sensor_Scan(void) //function of reading-in signal of line tracking module 
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}
void M_Control_IO_config(void)
{
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM)   
}
void Set_Speed(unsigned char Left,unsigned char Right) //function of setting speed
{
  analogWrite(Lpwm_pin,Left);
  analogWrite(Rpwm_pin,Right);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
     Car_state = 1; 
       
    }
void turnR()        //turning right(dual wheel)
    {
     digitalWrite(pinRB,LOW);  //making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  //making motor move towards left front
     Car_state = 4;
    
    }
void turnL()         //turning left(dual wheel)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );   //making motor move towards right front
     digitalWrite(pinLB,LOW);   //making motor move towards left rear
     digitalWrite(pinLF,HIGH);
     Car_state = 3;
     
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     Car_state = 5;
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
     Car_state = 2;
         
    }
void servoL()         //back up
    {
     if(DuoJiao<=180){  //motor turning left
                  myservo.attach(A2);
                  DuoJiao+=10;
                  myservo.write(DuoJiao);
                  delay(200);}             
         
    }
void servoR()         //back up
    {
     if(DuoJiao-10>=0){  //motor turning right 
                  myservo.attach(A2);
                  DuoJiao-=10;
                  myservo.write(DuoJiao);
                  delay(200);}     
         
    }
void speedUp()
{
  if(Rpwm_val+10<=250&&Rpwm_val+10<=250){  //increasing speed
                  Lpwm_val+=10; Rpwm_val+=10;
                  Set_Speed(Lpwm_val,Rpwm_val);                 
}

}
void speedDown()
{
  if(Rpwm_val-10>=0&&Rpwm_val-10>=0){  //decreasing speed
                  Lpwm_val-=10; Rpwm_val-=10;
                  Set_Speed(Lpwm_val,Rpwm_val);
                  
                  }

}

void spin() {
    digitalWrite(pinRB, LOW); // making motor move towards right rear
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, HIGH); // making motor move towards left front
    digitalWrite(pinLF, LOW);
    Car_state = 6;
}
         
void Xunji_Mode(void) //function of line tracking 
{
 Sensor_Scan();
 if (SM == HIGH)// middle sensor in black area
{
if (SL == LOW & SR == HIGH) // black on left, white on right, turn left
{
turnR();
}
else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
{
turnL();
}
else // white on both sides, going forward
{
advance();
}
}
else // middle sensor on white area
{
if (SL== LOW & SR == HIGH)// black on left, white on right, turn left
{
turnR();
}
else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
{
turnL();
}
else // all white, stop
{
back();
delay(100);
stopp() ; 
}
}
}


void Self_Control(void)//self-going, ultrasonic obstacle avoidance
{
  long a;
  SR04 sr04 = SR04(inputPin,outputPin);
     a=sr04.Distance();
a=Ultrasonic_Ranging(1);


   int H;
   myservo.write(DuoJiao);
  // H = Ultrasonic_Ranging(1);
   delay(300);
   IR_scan();   
   if(Ultrasonic_Ranging(1) < 15)         
   {IR_scan();
       stopp();              
       delay(100);
       back();               
       delay(100);
    }
           
  if(Ultrasonic_Ranging(1) < 30)        
      {
        IR_scan();
        stopp();  
        delay(100);            
        myservo.write(0);
     
        int L = ask_pin_L(2);
        delay(300);      
         myservo.write(180);
       
        int R = ask_pin_R(3);
        delay(300);      

        if(ask_pin_L(2) > ask_pin_R(3))   
        {IR_scan();
         back(); 
        delay(100);      
        turnL();
       delay(400);                  
       stopp();  
       delay(50);            
       myservo.write(DuoJiao);
       H = Ultrasonic_Ranging(1);
       delay(500); 
        }
        
      if(ask_pin_L(2)  <= ask_pin_R(3))   
      {IR_scan();
       back();  
       delay(100);  
       turnR(); 
       delay(400);   
       stopp();  
       delay(50);            
       myservo.write(DuoJiao);
       H = Ultrasonic_Ranging(1);
       delay(300);        
        }   
        if (ask_pin_L(2)  < 35 && ask_pin_R(3)< 35)   
        {IR_scan();
       stopp();            
       delay(50);
       back(); 
       delay(50);                   
        }          
      }
      else                      
      {
      advance();                
      }                 
}

int Ultrasonic_Ranging(unsigned char Mode)//function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation

{ 
  int old_distance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);

     SR04 sr04 = SR04(inputPin,outputPin);
  long distance;
  distance=sr04.Distance();
 // int distance = pulseIn(inputPin, HIGH);  // reading the duration of high level
  //distance= distance/58;   // Transform pulse time to distance   
  if(Mode==1){
         Serial.print("\n H = ");
         Serial.print(distance,DEC); 
        return distance;
  }
   else  return distance;
} 
int ask_pin_L(unsigned char Mode)    
  { 
  int old_Ldistance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  int Ldistance = pulseIn(inputPin, HIGH); 
  Ldistance= Ldistance/58;   // Transform pulse time to distance   
  if(Mode==2){
         Serial.print("\n L = ");
         Serial.print(Ldistance,DEC); 
         return Ldistance;
  }
   else  return Ldistance;
} 
int ask_pin_R(unsigned char Mode)   
   { 
  int old_Rdistance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); // 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  int Rdistance = pulseIn(inputPin, HIGH); 
  Rdistance= Rdistance/58;   // Transform pulse time to distance   
  if(Mode==3){
         Serial.print("\n R = ");
         Serial.print(Rdistance,DEC); 
         return Rdistance;
  }
   else  return Rdistance;
} 

void IR_Control(void)   //remote control，when pressing“#”，it quitting from the mode
{delay(100);
if(Key!=IR_ESC)
  {
   if(irrecv.decode(&results))  //to judge whether serial port receive data
    {
     Key = results.value;
    switch(Key)
     {
       case IR_Go:advance();   //UP
       break;
       case IR_Back: back();   //back
       break;
       case IR_Left:turnL();   //Left    
       break;
       case IR_Right:turnR(); //Righ
       break;
       case IR_Stop:stopp();   //stop
       break;
       case IR_Servo_L: if(DuoJiao<=180){  //motor turning left
                  myservo.attach(A2);
                  DuoJiao+=10;
                  myservo.write(DuoJiao);
                  delay(200);}             
       break;
      case IR_Servo_R: if(DuoJiao-10>=0){  //motor turning right 
                  myservo.attach(A2);
                  DuoJiao-=10;
                  myservo.write(DuoJiao);
                  delay(200);} 
       break;
      case IR_Speed_UP:if(Rpwm_val+10<=250&&Rpwm_val+10<=250){  //increasing speed
                  Lpwm_val+=10; Rpwm_val+=10;
                  Set_Speed(Lpwm_val,Rpwm_val);
                 
                  }
       break;
      case IR_Speed_DOWN:if(Rpwm_val-10>=0&&Rpwm_val-10>=0){  //decreasing speed
                  Lpwm_val-=10; Rpwm_val-=10;
                  Set_Speed(Lpwm_val,Rpwm_val);
                  
                  }
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    } 
  }
}
 

void Bluetooth_Control() //Bluetooth remote control
{  
  int a=Ultrasonic_Ranging(1);
  Serial.print("cm");
        //int L = ask_pin_L(2);
        //int R = ask_pin_R(3);

   if(Serial.available()) //to judge whether serial port receive data
    {
        
     Bluetooth_val=Serial.read();  //reading value of Bluetooth serial port, giving the value to val
    switch(Bluetooth_val)
     {
       case 'U':advance(); //UP
       break;
       case 'D': back();   //back
       break;
       case 'L':turnL();   //Left
       break;
       case 'R':turnR();  //Right
       break;
       case 'S':stopp();    //stop
       break;   
      case 'P': Serial.println("Pao Pao Pao"); //print "Pao Pao Pao"
      break;
      case 'X': servoL();
      break;
      case 'Y': servoR();
      break;
      case 'A': speedUp();
      break;
      case 'B': speedDown();
      break;
      case 'Z': spin(); // Spinbreak;
     }
    } 
}
void IR_scan()
{//delay(200);
  
  if(irrecv.decode(&res))  //to judge whether serial port receive data
    {
     Key1 = res.value;
    switch(Key1)
     {
       case IR_XunJi_Mode: flag=5;stopp();//delay(200);
       break;
       case IR_Self_Control: flag=6;stopp();//delay(200);
       break;
       case IR_IR_Control: flag=7;Key=0;stopp();//delay(200);
       break;
       case IR_Bluetooth_Control: flag=8;stopp();//delay(200);
       break;
       default: 
       break;
     }
     irrecv.resume(); // Receive the next value  
     }
  }




void setup() 
{ 
   myservo.attach(A2);
   M_Control_IO_config();     //motor controlling the initialization of IO
   Set_Speed(Lpwm_val,Rpwm_val);  //setting initialized speed
  
   Sensor_IO_Config();            //initializing IO of line tracking module 
   irrecv.enableIRIn();           //starting receiving IR remote control signal
   pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
   pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
   Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud 
    myservo.write(DuoJiao);
   stopp();                       //stop
   delay(1000);
   myservo.detach();
} 
void loop() 
{  
  
     IR_scan();
      if(flag==5)
     {myservo.detach();
      Xunji_Mode();
      }
      if(flag==6)
     {myservo.attach(A2);
      Self_Control();
      }
      if(flag==7)
     {myservo.detach();
      IR_Control();
      }
      if(flag==8)
     {myservo.detach();
      Bluetooth_Control();
      }
  delay(10);
}
