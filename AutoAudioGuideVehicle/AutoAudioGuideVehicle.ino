/*
 * 2GM Roboteam
 * Auto audio guide vehicle
 * Build for
 * "7th Panhellenic Open Technologies in Education Competition"
 * by "2nd Junior High School Moschato - Ilissiadeion"
 * School year 2024-25
 */

#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
SoftwareSerial softSerial(4, 3); //Rx,Tx
DFRobotDFPlayerMini myDFPlayer;
//Motor A
const int motorPin1  = 7;
const int motorPin2  = 6;
const int enA = 5;
//Motor B
const int motorPin3  = 9;
const int motorPin4  = 8;
const int enB = 10;
//Ultrasonic Sensor
const int echoPin = 11;
const int trigPin = 12;
long duration, distance, cm;
//IR sensors
const int leftIR = A2;
const int middleIR = A1;
const int rightIR = A0;
int leftID, middleID, rightID;
//Button
const int buttonPin = 2;
int buttonState = 1;
int moveOn = 1;
int track = 0;

void setup() {
  //Set pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  analogWrite(enA,140);
  analogWrite(enB,140);
  //Set pins for ultrasonic sensor
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  //Set pins as inputs for IR sensors
  pinMode(leftIR, INPUT);
  pinMode(middleIR, INPUT);
  pinMode(rightIR, INPUT);
  //Set pin as inputs for button
  pinMode(buttonPin, INPUT_PULLUP);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  softSerial.begin(9600);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(softSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  myDFPlayer.volume(30);  //Set volume value (0~30)
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);  //Set different EQ
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);  //Set device we use SD as default
}

void loop() {
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
  cm = getDistance();
  Serial.print (cm);
  Serial.println(" cm");
  if (buttonState == LOW){
    moveOn = 0;
    delay(100);
    go();
  }
  if (moveOn == LOW) {    
    if (cm <= 15){
      stop();
      Serial.println("stop at object");
    }
    else {
      move();
    }
  }
  else {
    Serial.println("Ready");
    delay(100);
  }
}

void move(){
  leftID = digitalRead(leftIR);
  middleID = digitalRead(middleIR);
  rightID = digitalRead(rightIR);
  if (leftID==HIGH && middleID==HIGH && rightID==HIGH){
    stop();
    Serial.println("stop");
    moveOn = 1;
    track = track + 1;
    Serial.print("track=");
    Serial.println(track);
    playSong();  //Play mp3
    delay(1000);    
  }
  else if (leftID==LOW && middleID==HIGH && rightID==LOW){
    forward();
    Serial.println("forward");
  }
  else if (leftID==HIGH && middleID==HIGH && rightID==LOW){
    turnLeft();
    Serial.println("turnLeft");
  }
  else if (leftID==LOW && middleID==HIGH && rightID==HIGH){
    turnRight();
    Serial.println("turnRight");
  }
  else if (leftID==HIGH && middleID==LOW && rightID==LOW){
    sharpTurnLeft();
    Serial.println("sharpTurnLeft");
  }
  else if (leftID==LOW && middleID==LOW && rightID==HIGH){
    sharpTurnRight();
    Serial.println("sharpTurnRight");
  }
  else{
    backward();
    Serial.println("backward");
  }
  Serial.print(leftID);
  Serial.print(middleID);
  Serial.println(rightID);  
}

void forward(){
  // This code will turn Motor A & B clockwise for 1 sec.
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(10);  
}

void backward(){
  // This code will turn Motor A & B counter-clockwise for 1 sec.
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(10);  
}

void turnLeft(){
  //This code  will turn Motor A clockwise for 0.1 sec.
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(10);  
}

void turnRight(){
  //This code will turn Motor B clockwise for 0.1 sec.
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(10);
}

void sharpTurnLeft(){
  //This code  will turn Motor A clockwise & Motor B counter-clockwise for 0.1 sec.
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(10);  
}

void sharpTurnRight(){
  //This code will turn Motor A counter-clockwise & Motor B clockwise for 0.1 sec.
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(10);  
}

void stop(){
  //This code will stop motors
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);  
}

long getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

void go(){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(200);
}

void playSong(){
  myDFPlayer.play(track);
  if (track == 4){
    track = 0;
  }
}
