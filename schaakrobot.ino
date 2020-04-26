/*
In dit programma luisterd de arduino tot die een comando ontvangt van de raspberry pi.
in dit commando dat serieel doorgegeven wordt zit de start en eindpositie van de speler die tegen de robot speelt(wit) als ook de start en eindpositie van de robot zelf(zwart) die door de raspberry pi bepaald wordt in een python programma sunfish.py
een voorbeeld hiervan zou zijn: a2a3e7e600
in dit voorbeeld zijn de eerste 2 leestekens afkomstig van het door ons ingegeven coördinaat van een schaakstuk die we wensen te verplaatsen: a2
de 3de en 4de leestekens zijn de door ons ingegeven coördinaten om aan te geven waar we dit schaakstuk wensen te plaatsen: a3
de 5 tem de 8ste leestekens zijn op analoge manier opgebouwd met begin en eindpositie bepaald door de raspberry pi: e7e6
de 9de character is om aan te duiden of er door de ingevoerde zet van de gebruiker(wit) een schaakstuk van de computer(zwart) verwijderd moet worden: 0 (hier niet het geval)
de 10de character is om aan te duiden of de computer(zwart) een schaatsuk van de gebruiker(wit) moet weghalen: 0 (hier niet het geval)
De arduino gebruikt deze gegevens om te bepalen welke functies op te roepen om een pad aan te sturen en om de grijper te openen of te sluiten.
*/

//Insert library
#include <Servo.h>
#include <Wire.h>

//Servo pins
#define basePin 2
#define shoulderPin 3
#define elbowPin 4
#define wristPin 5
#define ulnaPin 6
#define gripperPin 10

//Button pins
#define IOPinA 7
#define IOPinB 8
#define IOPinC 9

//Servomotors + pin connection
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo ulna;
Servo gripper;

//comunications
String data = "start";
String dataOld = "start";
String data0 = "";
String data1 = "";
String data2 = "";
String data3 = "";
String data4 = "";
String data5 = "";
String data6 = "";
String data7 = "";
String player1remove = "";
String player2remove = "";
String startplayer1 = "";
String endplayer1 = "";
String startplayer2 = "";
String endplayer2 = "";


//angles in microseconds
double baseMicros = 1500;
double shoulderMicros = 1550;
double elbowMicros = 1500;
double wristMicros = 16500;
double ulnaMicros = 1500;
double gripperMicros = 1500;

//Button States
int buttonStateA = 0;
int buttonStateB = 0;
int buttonStateC = 0;

//extra variables
int grip = 0; //opening of the gripper
float L1 = 180; // lenght of arm 1
float L2 = 183; // length of arm 2
float L3 = 182; // length of arm 3 

//position
float x = 200;
float y = 0;
float z = 180;
float phi = 0;
float psi = 0;

//startposition
float xStart = 313;
float yStart = 0;
float zStart = 100;
float phiStart = -0.30;
float psiStart = 0;

//destination
float xDest = 200;
float yDest = 0;
float zDest = 180;
float phiDest = -0.95;
float psiDest = 0;

//pathposition
int AantPosHoek = 20;
int AantPos = 0;
int Pos = 1;
float xPath = 200;
float yPath = 0;
float zPath = 180;
float phiPath = -0.95;
float psiPath = 0;

//hulp Variabelen
float pi=3.14159265;
float delta1 = 0;
float p1 = 0;
float delta2 = 0;
float p2 = 0;
float epsilon1 = 0;
float epsilon2 = 0;
float epsilon3 = 0;
float gamma = 0;

//Motorhoeken in Radialen
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;
float theta5 = 0;

//counters
int counter = 0;
int countermain = 0;
int incomingByte = 0;
String inString = "";
int straitpath = 0;


//Berekend het aantal benodigde IntermediatSteps om het pad te volgen
int IntermediatSteps(){
  AantPos = sqrt((xDest-xStart)*(xDest-xStart)+(yDest-yStart)*(yDest-yStart)+(zDest-zStart)*(zDest-zStart));                                        //de afstand van het beginpunt tot aan het eindpunt 
  if (AantPos < abs((phiDest-phiStart)*100)  && xDest==xStart && yDest==yStart && zDest==zStart && ((phiDest-phiStart)==!0)){ 
    AantPos = abs(phiDest-phiStart)*100;
  }
}

//Geeft de IntermediatSteps in een pad
int PathPoint(){
  IntermediatSteps();
  for (Pos = 1 ; Pos <= AantPos ; Pos++){
    if (abs((xDest-xPath)) <=1){
      xPath = xDest;
      }
    else{  
    xPath = xStart + Pos * ((xDest-xStart)/AantPos); 
    }  
    if (abs((yDest-yPath)) <=1){
      yPath = yDest;
      }     
    else{  
    yPath = yStart + Pos * ((yDest-yStart)/AantPos); 
    }
    if (abs((zDest-zPath)) <=1){
      zPath = zDest;
      }
    else{  
    zPath = zStart + Pos * ((zDest-zStart)/AantPos); 
    }
    if (abs((phiDest-phiPath)*100) <=1){
      phiPath = phiDest;
      }
    else{  
    phiPath = phiStart + Pos * 0.01*(((phiDest-phiStart)*100)/AantPos);
    }
    x = xPath;
    y = yPath;                  
    z = zPath;
    phi = phiPath;
    psi = psiDest;
    
    InversKinematics(x,y,z,phi,psi);
    AnglesToMicros();
    ServoUpdate();
   
    delay (10);
   }
   xStart = xDest;
   yStart = yDest;
   zStart = zDest;
   phiStart = phiDest;
   psiStart = psiDest;
}

int InversKinematics(double x1,double y1,double z1,float phi1,float psi1){
  
  theta1=atan2(y1,x1);
  delta1=atan(z1/sqrt(x1*x1+y1*y1));
  p1=sqrt(x1*x1+y1*y1+z1*z1);
  p2=sqrt(p1*p1+L3*L3-2*p1*L3*cos(phi1-delta1));
  gamma=acos((p1*p1+p2*p2-L3*L3)/(2*p2*p1));
  if (delta1>phi1){
      delta2=delta1+gamma;
  }
  else{
      delta2=delta1-gamma;
  }
  epsilon1=acos((L1*L1+p2*p2-L2*L2)/(2*L1*p2));
  epsilon2=acos((p2*p2+L2*L2-L1*L1)/(2*p2*L2));
  epsilon3=acos((p2*p2+L3*L3-p1*p1)/(2*p2*L3));
  theta2=epsilon1+delta2;
  theta3=pi-acos((L1*L1+L2*L2-p2*p2)/(2*L1*L3));
  if (delta1<phi1){
      theta4=pi-epsilon3+epsilon2;
  }
  else{
      theta4=epsilon2-(pi-epsilon3);
  }
  theta5=psi1;
}


//sturen van de servo
int ServoUpdate(){
    base.write(baseMicros);
    shoulder.write(shoulderMicros);
    elbow.write(elbowMicros);
    wrist.write(wristMicros);
    ulna.write(ulnaMicros);
    gripper.write(gripperMicros);
}

//in deze functie worden de hoeken in radialen omgezet naar de micros voor de servosturing
int AnglesToMicros(){
    baseMicros = theta1*(-572.958)+1500;
    shoulderMicros = theta2*480+770;
    elbowMicros = theta3*573+550;
    wristMicros = theta4*557+1675;
    ulnaMicros = theta5*572.958+1550; 
}

int ChessStone(String stone){
 if (stone == "a1"){
     xDest = 312;
     yDest = -111;
     zDest = -5;
 }
 else if (stone == "a2"){
     xDest = 278;
     yDest = -111;
     zDest = -12;
 }
 else if (stone == "a3"){
     xDest = 247;
     yDest = -111;
     zDest = -18;
 }
 else if (stone == "a4"){
     xDest = 216;
     yDest = -111;
     zDest = -20;
 }
 else if (stone == "a5"){
     xDest = 186;
     yDest = -111;
     zDest = -21;
 }
 else if (stone == "a6"){
     xDest = 159;
     yDest = -110;
     zDest = -23;
 }
 else if (stone == "a7"){
     xDest = 129;
     yDest = -110;
     zDest = -24;
 }
 else if (stone == "a8"){
     xDest = 102;
     yDest = -112;
     zDest = -24;
 }
 else if (stone == "b1"){
     xDest = 312;
     yDest = -79;
     zDest = -5;
 }
 else if (stone == "b2"){
     xDest = 278;
     yDest = -79;
     zDest = -12;
 }
 else if (stone == "b3"){
     xDest = 247;
     yDest = -79;
     zDest = -18;
 }
 else if (stone == "b4"){
     xDest = 216;
     yDest = -78;
     zDest = -20;
 }
 else if (stone == "b5"){
     xDest = 190;
     yDest = -78;
     zDest = -20;
 }
 else if (stone == "b6"){
     xDest = 161;
     yDest = -78;
     zDest = -22;
 }
 else if (stone == "b7"){
     xDest = 133;
     yDest = -79;
     zDest = -24;
 }
 else if (stone == "b8"){
     xDest = 107;
     yDest = -81;
     zDest = -24;
 }
 else if (stone == "c1"){
     xDest = 312;
     yDest = -45;
     zDest = -7;
 }
 else if (stone == "c2"){
     xDest = 280;
     yDest = -44;
     zDest = -16;
 }
 else if (stone == "c3"){
     xDest = 250;
     yDest = -44;
     zDest = -20;
 }
 else if (stone == "c4"){
     xDest = 221;
     yDest = -43;
     zDest = -22;
 }
 else if (stone == "c5"){
     xDest = 193;
     yDest = -43;
     zDest = -23;
 }
 else if (stone == "c6"){
     xDest = 166;
     yDest = -43;
     zDest = -24;
 }
 else if (stone == "c7"){
     xDest = 140;
     yDest = -44;
     zDest = -24;
 }
 else if (stone == "c8"){
     xDest = 115;
     yDest = -48;
     zDest = -23;
 }
 else if (stone == "d1"){
     xDest = 313;
     yDest = -12;
     zDest = -8;
 }
 else if (stone == "d2"){
     xDest = 281;
     yDest = -12;
     zDest = -16;
 }
 else if (stone == "d3"){
     xDest = 251;
     yDest = -12;
     zDest = 20;
 }
 else if (stone == "d4"){
     xDest = 223;
     yDest = -12;
     zDest = -22;
 }
 else if (stone == "d5"){
     xDest = 194;
     yDest = -12;
     zDest = -23;
 }
 else if (stone == "d6"){
     xDest = 167;
     yDest = -12;
     zDest = -24;
 }
 else if (stone == "d7"){
     xDest = 142;
     yDest = -11;
     zDest = -23;
 }
 else if (stone == "d8"){
     xDest = 117;
     yDest = -11;
     zDest = -23;
 }
 else if (stone == "e1"){
     xDest = 313;
     yDest = 21;
     zDest = -8;
 }
 else if (stone == "e2"){
     xDest = 281;
     yDest = 21;
     zDest = -16;
 }
 else if (stone == "e3"){
     xDest = 251;
     yDest = 22;
     zDest = -20;
 }
 else if (stone == "e4"){
     xDest = 223;
     yDest = 22;
     zDest = -21;
 }
 else if (stone == "e5"){
     xDest = 194;
     yDest = 22;
     zDest = -23;
 }
 else if (stone == "e6"){
     xDest = 167;
     yDest = 22;
     zDest = -24;
 }
 else if (stone == "e7"){
     xDest = 142;
     yDest = 22;
     zDest = -23;
 }
 else if (stone == "e8"){
     xDest = 117;
     yDest = 24;
     zDest = -23;
 }
 else if (stone == "f1"){
     xDest = 313;
     yDest = 51;
     zDest = -8;
 }
 else if (stone == "f2"){
     xDest = 281;
     yDest = 51;
     zDest = -16;
 }
 else if (stone == "f3"){
     xDest = 251;
     yDest = 51;
     zDest = -20;
 }
 else if (stone == "f4"){
     xDest = 223;
     yDest = 51;
     zDest = -21;
 }
 else if (stone == "f5"){
     xDest = 194;
     yDest = 52;
     zDest = -22;
 }
 else if (stone == "f6"){
     xDest = 167;
     yDest = 53;
     zDest = -24;
 }
 else if (stone == "f7"){
     xDest = 140;
     yDest = 55;
     zDest = -23;
 }
 else if (stone == "f8"){
     xDest = 113;
     yDest = 61;
     zDest = -22;
 }
 else if (stone == "g1"){
     xDest = 314;
     yDest = 81;
     zDest = -4;
 }
 else if (stone == "g2"){
     xDest = 282;
     yDest = 81;
     zDest = -14;
 }
 else if (stone == "g3"){
     xDest = 252;
     yDest = 81;
     zDest = -18;
 }
 else if (stone == "g4"){
     xDest = 222;
     yDest = 82;
     zDest = -20;
 }
 else if (stone == "g5"){
     xDest = 193;
     yDest = 85;
     zDest = -20;
 }
 else if (stone == "g6"){
     xDest = 164;
     yDest = 87;
     zDest = -23;
 }
 else if (stone == "g7"){
     xDest = 137;
     yDest = 87;
     zDest = -23;
 }
 else if (stone == "g8"){
     xDest = 109;
     yDest = 89;
     zDest = -23;
 }
 else if (stone == "h1"){
     xDest = 314;
     yDest = 114;
     zDest = -2;
 }
 else if (stone == "h2"){
     xDest = 282;
     yDest = 114;
     zDest = -13;
 }
 else if (stone == "h3"){
     xDest = 251;
     yDest = 116;
     zDest = -17;
 }
 else if (stone == "h4"){
     xDest = 220;
     yDest = 117;
     zDest = -18;
 }
 else if (stone == "h5"){
     xDest = 191;
     yDest = 117;
     zDest = -21;
 }
 else if (stone == "h6"){
     xDest = 163;
     yDest = 117;
     zDest = -22;
 }
 else if (stone == "h7"){
     xDest = 135;
     yDest = 116;
     zDest = -22;
 }
 else if (stone == "h8"){
     xDest = 106;
     yDest = 118;
     zDest = -22;
 }
}

int PartGripper(String PositionGripper){
  
    ChessStone(PositionGripper);
    zDest = 100;
    PathPoint();                            //bewegen naar positie ChessStone op hoogte 100
    delay(1000);
    ChessStone(PositionGripper);
    PathPoint();                            //zakken tot op hoogte 0
    delay(1000);
    gripperMicros = 1800;
    ServoUpdate();                      //gripper in gesloten positie plaatsen
    delay(1000);
    zDest = 100;
    PathPoint();                            //bewegen naar positie ChessStone op hoogte 100
    delay(1000);
}

int PartPlacer(String PositionPlacer){
    
    ChessStone(PositionPlacer);
    zDest = 100;
    PathPoint();                            //bewegen naar positie ChessStone op hoogte 100
    delay(1000);
    ChessStone(PositionPlacer);
    PathPoint();                            //zakken tot op hoogte 0
    delay(1000);
    gripperMicros = 1200;
    ServoUpdate();                     //gripper in open posite plaatsen
    delay(1000);
    zDest = 100;
    PathPoint();                            //bewegen naar positie ChessStone op hoogte 100
    delay(1000);
    xDest = 220;
    yDest = 0;
    zDest = 100;
    PathPoint();                            //bewegen naar home positie robot
    delay(1000);
}

int PartRemove(){
    xDest = 220;
    yDest = 260;
    zDest = 100;
    PathPoint();                            //bewegen naar positie langs het schaakbord op hoogte 100
    delay(1000);
    gripperMicros = 1200;               
    ServoUpdate();                      //gripper in open posite plaatsen
    delay(1000);
    xDest = 220;
    yDest = 0;
    zDest = 100;
    PathPoint();                            //bewegen naar home positie robot
    delay(1000);
}

void setup(){

  //Activate Serial monitor 
  Serial.begin(9600);

  //Attach servomotors to pins
  base.attach(basePin);
  shoulder.attach(shoulderPin);
  elbow.attach(elbowPin);
  wrist.attach(wristPin);
  ulna.attach(ulnaPin);
  gripper.attach(gripperPin);

  //initialze pushbutton pins as input
  pinMode(IOPinA, INPUT);
  pinMode(IOPinB, INPUT);
  pinMode(IOPinC, INPUT);
}

void loop(){

  //read buttons
  buttonStateA = digitalRead(IOPinA);
  buttonStateB = digitalRead(IOPinB);
  buttonStateC = digitalRead(IOPinC);
  if (!buttonStateA){
    Serial.println(data);
  }
  
  if (data !=dataOld){
    
    
    if (player1remove == "1"){
      PartGripper(endplayer1);
      PartRemove();
    }
   
    PartGripper(startplayer1);
    PartPlacer(endplayer1);
    
    if (player2remove == "1"){
      PartGripper(endplayer2);
      PartRemove();
    }
        
    PartGripper(startplayer2);
    PartPlacer(endplayer2);
    
    dataOld = data;
  }
}
   
//inlezen van data die raspberry pi voorziet
void serialEvent(){
   Serial.flush();
  
   while (Serial.available()){
    data = Serial.readStringUntil('\n');
    data0 = data[0];
    data1 = data[1];
    data2 = data[2];
    data3 = data[3];
    data4 = data[4];
    data5 = data[5];
    data6 = data[6];
    data7 = data[7];
    player1remove = data[8];
    player2remove = data[9];
    startplayer1 =  data0 + data1;
    endplayer1 = data2 + data3;
    startplayer2 = data4 + data5; 
    endplayer2 = data6 + data7;
    }  
}
