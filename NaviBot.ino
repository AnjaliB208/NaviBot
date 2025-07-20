
//************************ Robot****************************

#include <IRremote.h>// IR remote library
#include <EEPROM.h>// EEPROM pre-installed library
#include <Servo.h>
/*
   ************** Servo Motor**************
*/
Servo myservo;
const byte servostart = 97; //servo motor start point


/*
   ********************** Motor **********************
*/
int enA = 2;
int in1 = 3;
int in2 = 4;
int in3 = 5;
int in4 = 6;
int enB = 7;

/*
   ************** Ultrasonic ***************
*/
int trigpin = A1;
int echopin = A2;
long t, cm;
int distanceleft = 0;
int distanceright = 0;


/*
   ****************** IR Sensor******************
*/
int R_S = A5; //Right sensor
int M_S = A6; //Middle sensor
int L_S = A7; //Left sensor
int LEFT_IR = A4;
int RIGHT_IR = A3;

/*
   ********************IR Receiver**********************
*/
# define RECV_PIN 9 // Pin to which IR reciever is connected

IRrecv irrecv(RECV_PIN);// Pass the pin number to the function

unsigned long int btn = 0;

int mode = 0;

/*
 ************HEX code of all the buttons used in the project**************
 */
#define FWD       0xE41B7F80 //2 //go forward(2)
#define LFT       0xF30C7F80 //4 //go left(4)
#define RGT       0xF10E7F80 //6 //go right(6)
#define BWD       0xF00F7F80 //8 //go backward(8)
#define STOP      0xFE017F80 //Play|Pause //stop(0) //
#define RPEAT     0xF7087F80 //RPT //repeat the stored sequence of movement from the temporary memory(automatically stores) (REPT)
#define DEL       0xFB047F80 //EQ //delete the stored sequence of movement from temporary memory(EQ)
#define PERST     0xE11E7F80 //MUTE //copy the sequence from temp. memory to the peramanent memory(EEPROM)(MUTE)
#define PLAYEPROM 0xFC037F80 //FASTFWD //repeat the sequence stored in EEPROM(FASTFWD)
#define RESET     0xE51A7F80 //Mode //Resets the Arduino Board(Mode)

#define HF       0xF50A7F80 //1 //Human Following
#define OA       0xE01F7F80 //3 //Obstacle Avoidance
#define LF       0xFF007F80 //7 //Line Follower
#define PM       0xE6197F80 //9 //Path memorizing


/*
 ************Global Variables and Arrays for path memorize**************
 */
 
unsigned long int value = 0; // stores the incoming hex value
byte seq = 0; //stores the current number of executed sequences
byte seq_Array[50];// array to store the movement sequence in terms of integers(1 for FWD, 2 for LEFT and so on..)

//counter for counting the number of times program pass through a movement function(fwd, lft etc.)
int fwd_Counter = -1;
int lft_Counter = -1;
int rgt_Counter = -1;
int bwd_Counter = -1;
int stp_Counter = -1;

//global "current time" variables for different movement functions(fwd, lft etc.)
unsigned long int current_Time0 = 0;// for FWD movement
unsigned long int current_Time1 = 0;// for LEFT movement
unsigned long int current_Time2 = 0;// for RIGHT movement
unsigned long int current_Time3 = 0;// for BWD movement
unsigned long int current_Time4 = 0;// for STOP

//total time spend by the pgm in executing the movement(fwd, lft etc.) for a particular movement counter
unsigned long int total_Fwd_Time[10];
unsigned long int total_Lft_Time[10];
unsigned long int total_Rgt_Time[10];
unsigned long int total_Bwd_Time[10];
unsigned long int total_Stp_Time[10];

/*
 ************Arduino Reset Pin****************
 */
#define RESET_PIN A0

/*
 *****************Setup Code******************
 */
void setup() {
  // start serial communication
  Serial.begin(9600);
  // set mode of the pins as output
  for (int i = 2; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(trigpin, OUTPUT);//A1
  pinMode(echopin, INPUT);//A2
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);


  pinMode(L_S, INPUT);
  pinMode(M_S, INPUT);
  pinMode(R_S, INPUT);

  analogWrite(enA, 150); 
  analogWrite(enB, 150);
  delay(200);
  
  myservo.attach(10);
  start_servo();
  rotate();
  delay(500);
  // In case the interrupt driver crashes on setup, give a clue
  // to the user what's going on.
  Serial.println("Enabling IRin");
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("Enabled IRin");

}

void loop() {
  if (irrecv.decode()) {
    btn = irrecv.decodedIRData.decodedRawData;
    Serial.println(btn, HEX);
    delay(100);
    irrecv.resume();
  }
  delay(100);
  
  if(btn == HF)
  {
    mode = 1;
  }
  else if(btn == OA)
  {
    mode = 2;
  }
  else if(btn == LF)
  {
    mode = 3;
  }
  else if(btn == PM)
  {
    mode = 4;
  }

  
  if(mode == 1)
  {
    Serial.println("Human Following function executing");
    getdistance();
  
    int Right_Value = digitalRead(RIGHT_IR);
    int Left_Value = digitalRead(LEFT_IR);

    if((Right_Value==1) && (cm>=10  && cm<=30)&&(Left_Value==1))
    {forward();}
    else if((Right_Value==0) && (Left_Value==1)){turnright();}
    else  if((Right_Value==1) && (Left_Value==0)){turnleft();}
    else if((Right_Value==1)  && (Left_Value==1)){robostop();}
    else if(cm > 5 && cm < 10){robostop();}
    else  if(cm < 5){backward();}
  }
  else if(mode == 2)
  {

    Serial.println("Obstacle Avoidance function executing");
    getdistance();
  int leftdistance = 0;
  int rightdistance = 0;
 
  if (cm <= 30) {
    robostop();
    Serial.println("robo stop");
    delay(100);
    backward();
    Serial.println("robo backward");
    delay(400);//300
    robostop();
    Serial.println("robo stop");
    delay(200);
    leftdistance = leftsee();
    Serial.println(leftdistance);
    delay(200);
    rightdistance = rightsee();
    Serial.println(rightdistance);
 
    if (leftdistance >= rightdistance) {
      turnleft();
      delay(500);
      robostop();
      Serial.println("turnleft");
    } else {
      turnright();
      delay(500);
      robostop();
      Serial.println("turnright");
    }
  } else {
    forward();
    Serial.println("forward");
  } 
  }
  else if(mode == 3)
  {

    Serial.println("Line following function executing");
    analogWrite(enA, 100);
    analogWrite(enB, 100);
    if ((digitalRead(L_S) == 0)&&(digitalRead(M_S) == 1)&&(digitalRead(R_S) == 0)){forward();}

    if ((digitalRead(L_S) == 1)&&(digitalRead(M_S) == 1)&&(digitalRead(R_S) == 0)){turnleft();}
    if ((digitalRead(L_S) == 1)&&(digitalRead(M_S) ==0)&&(digitalRead(R_S) == 0)) {turnleft();}

    if ((digitalRead(L_S) == 0)&&(digitalRead(M_S) == 1)&&(digitalRead(R_S) == 1)){turnright();}
    if ((digitalRead(L_S) == 0)&&(digitalRead(M_S) == 0)&&(digitalRead(R_S) == 1)){turnright();}

    if ((digitalRead(L_S) == 0)&&(digitalRead(M_S) == 0)&&(digitalRead(R_S) == 0)){robostop();}
  }
  else if(mode == 4)
  {
    Serial.println("Path memorize function executing");
    if (irrecv.decode()) {
    value = irrecv.decodedIRData.decodedRawData;
    Serial.println(value, HEX);
    delay(100);
    irrecv.resume();
  }

    delay(100);
    check_Inst(value);
    value=0; 
  }
  
   // btn=0;
  
}

/*
**************Servo motor ****************************
*/
void start_servo() {
  myservo.write(servostart);
  
  delay(3000);
  for (int a = 0; a < 4; a++) {
    myservo.write(servostart);//97
    delay(50);
    myservo.write(60);
    delay(50);
    myservo.write(120);
    delay(50);
    myservo.write(servostart);
  }
}

int leftsee() {
  myservo.write(servostart);
  delay(1000);
  myservo.write(155);
  delay(1000);
  distanceleft = getdistance();
  //Serial.println(distanceleft);
  myservo.write(servostart);
  return distanceleft;
}

int rightsee() {
  myservo.write(servostart);
  delay(1000);
  myservo.write(35);//5
  delay(1000);
  distanceright = getdistance();
  //Serial.println(distanceright);
  myservo.write(servostart);
  return distanceright;
}

/*
********************Ultrasonic**************************
*/
int getdistance() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);//trigpin2
  t = pulseIn(echopin, HIGH);//echopin4
  cm = t / 29 / 2;
  Serial.println("Distance: ");
  Serial.print(cm);
  return cm;
}

//*******************************************************


void check_Inst(long int value) {

  switch (value) {
    case FWD:
      go_Forward();
      delay(10);
      break;
    case LFT:
      go_Left();
      delay(10);
      break;
    case RGT:
      go_Right();
      delay(10);
      break;
    case BWD:
      go_Backward();
      delay(10);
      break;
    case STOP:
      go_Stop();
      delay(10);
      break;
    case RPEAT:
      go_In_Seq();
      delay(10);
      break;
    case DEL:
      del_From_Local_Mem();
      delay(10);
      break;
    case PERST:
      write_To_Permt_Mem();
      delay(10);
      break;  
    case PLAYEPROM:
      Read_Permt_Mem();
      delay(10);
      break;   
    case RESET:
      pinMode(RESET_PIN, OUTPUT);
      digitalWrite(RESET_PIN,HIGH);   
      break;
                
     default:
      value = 0;
  }
}

void go_Forward() {
  forward();

  current_Time0 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 2:
      // total time elaspsed since Left button is pressed including rest time 
      total_Lft_Time[lft_Counter + 1] = (current_Time0 - current_Time1);
      lft_Counter++;
      break;

    case 3:
      // total time elaspsed since Right button is pressed including rest time 
      total_Rgt_Time[rgt_Counter + 1] = (current_Time0 - current_Time2);
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time0 - current_Time3);
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time0 - current_Time4);
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 1;
  seq++;
}

void go_Left() {
  turnleftPM();

  current_Time1 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time1 - current_Time0);
      fwd_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time1 - current_Time2);
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time1 - current_Time3);
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time1 - current_Time4);
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 2;
  seq++;
}

void go_Right() {
  turnrightPM();

  current_Time2 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time2 - current_Time0);
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time2 - current_Time1);
      lft_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time2 - current_Time3);
      bwd_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time2 - current_Time4);
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 3;
  seq++;
}

void go_Backward() {
  backward();

  current_Time3 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time3 - current_Time0);
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time3 - current_Time1);
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time3 - current_Time2);
      rgt_Counter++;
      break;

    case 5:
      total_Stp_Time[stp_Counter + 1] = (current_Time3 - current_Time4);
      stp_Counter++;
      break;
  }

  seq_Array[seq] = 4;
  seq++;
}

void go_Stop() {
  robostop();

  current_Time4 = millis();
  int i = seq_Array[(seq - 1)];
  switch (i) {
    case 1:
      total_Fwd_Time[fwd_Counter + 1] = (current_Time4 - current_Time0);
      fwd_Counter++;
      break;

    case 2:
      total_Lft_Time[lft_Counter + 1] = (current_Time4 - current_Time1);
      lft_Counter++;
      break;

    case 3:
      total_Rgt_Time[rgt_Counter + 1] = (current_Time4 - current_Time2);
      rgt_Counter++;
      break;

    case 4:
      total_Bwd_Time[bwd_Counter + 1] = (current_Time4 - current_Time3);
      bwd_Counter++;
      break;
  }

  seq_Array[seq] = 5;
  seq++;
}

void go_In_Seq(void) {
  value = 0;
  for (int i = 0; i < (seq + 1); i++) {
    int value1 = 0;
    value1 = seq_Array[i];
    switch (value1) {
      case 1:
        static int j = 0;
        go_Forward_Seq(j);
        j++;
        break;
      case 2:
        static int k = 0;
        go_Left_Seq(k);
        k++;
        break;
      case 3:
        static int l = 0;
        go_Right_Seq(l);
        l++;
        break;
      case 4:
        static int m = 0;
        go_Backward_Seq(m);
        m++;
        break;
      case 5:
        static int n = 0;
        go_Stop_Seq(n);
        n++;
        break;
      default:
        j = 0; k = 0; l = 0; m = 0; n = 0;
    }
  }
}

void del_From_Local_Mem() {
  //set the movement counters to their default values
  fwd_Counter = -1;
  lft_Counter = -1;
  rgt_Counter = -1;
  bwd_Counter = - 1;
  stp_Counter = - 1;

  //set the total movement time to its default value
  for (int i = 0; i < 10; i++) {
    total_Fwd_Time[i] = 0;
    total_Lft_Time[i] = 0;
    total_Rgt_Time[i] = 0;
    total_Bwd_Time[i] = 0;
    total_Stp_Time[i] = 0;
  }

  // Reset the sequence array(stored movement instructions)
  for (int i = 0; i < 50; i++) {
    seq_Array[i] = 0;
  }

  seq = 0;
  
}


/**********************************************************************************************************
     This function copy the data from the arrays to the EEPROM(permanent memory)
************************************************************************************************************/

void write_To_Permt_Mem(){
  // total number of movement is stored in a random address i.e, 100
  EEPROM.write(100,seq);
    
  //writing the movement sequence
  for(int i=0; i<seq; i++){ 
  EEPROM.write(2*i,seq_Array[i]);
  }

  //storing the time bw two successive movements
  for(int i=1; i<seq+1; i++){           
  if(seq_Array[i-1]==1){
    static byte a=0;
    EEPROM.write(2*i-1,(total_Fwd_Time[a])/1000);// Note: One location can store maximum value of 255, hence the time is divided by 1000 here. And then multiplied by 1000 while retreiving the data from EEPROM location
    a++;
    }
  else if(seq_Array[i-1]==2){
    static byte b=0;
    EEPROM.write(2*i-1,(total_Lft_Time[b])/1000);
    b++;
    }
  else if(seq_Array[i-1]==3){
    static byte c=0;
    EEPROM.write(2*i-1,(total_Rgt_Time[c])/1000);
    c++;
    }
  else if(seq_Array[i-1]==4){
    static byte d=0;
    EEPROM.write(2*i-1,(total_Bwd_Time[d])/1000);  
    d++;
    }
  else if(seq_Array[i-1]==5){
    static byte e=0;
    EEPROM.write(2*i-1,(total_Stp_Time[e])/1000);  
    e++;
    }             
  }
 } 

 
/**********************************************************************************************************
     This function reads the stored sequence from the EEPROM(permanent memory)
************************************************************************************************************/

void Read_Permt_Mem(){
  // Read from permanent memory
   byte x = EEPROM.read(100);
   for(int i=0; i<x+1; i++){
    byte r = EEPROM.read(2*i);
    switch(r){
      case 1:
        forward();
        break;
      case 2:
        turnleftPM();
        break;
      case 3:
        turnrightPM();
        break;
      case 4:
        backward();
        break; 
      case 5:
        robostop();
        break;                          
      }
     delay((EEPROM.read(i+1))*1000);    // multiplied by thousand because the original time was divided by 1000 while storing in EEPROM.
    }
  }
 
/**********************************************************************************************************
     These function moves the car in a direction for the time specified/stored in the total_x_time array
************************************************************************************************************/
void go_Forward_Seq(int j) {
  //go in forward direction sequence
  forward();
  delay(total_Fwd_Time[j]);
}

void go_Left_Seq(int k) {
  //go in Left direction sequence
  turnleftPM();
  delay(total_Lft_Time[k]);
}

void go_Right_Seq(int l) {
  //go in right direction sequence
  turnrightPM();
  delay(total_Rgt_Time[l]);
}

void go_Backward_Seq(int m) {
  //go in backward direction sequence
  backward();
  delay(total_Bwd_Time[m]);
}

void go_Stop_Seq(int n) {
  //go in Stop sequence
  robostop();
  delay(total_Stp_Time[n]);
}

/***********************************************************
         Motor code for movement of the bot
************************************************************/

void rotate() {
  delay(500);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}



void forward(void) {
  Serial.println("Going_Forward");
  //forward movement instructions
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnleftPM(void) {//movement_Inst_Lft
  Serial.println("Going_Left");
  // Left movement instructions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(500);// default delay for smooth rotation.
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(500);
  // NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). Hence subtract 1s before repeating this movement
}

void turnrightPM(void) {
  Serial.println("Going_Right");
  // Rgt movement instructions
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  delay(500);// default delay for smooth rotation.
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(500);
  /* NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). 
   Hence subtract 1s before repeating this movement */
}

void turnleft(void) {//movement_Inst_Lft
  Serial.println("Going_Left");
  // Left movement instructions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  /*
  delay(500);// default delay for smooth rotation.
  
  NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). 
  Hence subtract 1s before repeating this movement
  */
}

void turnright(void) {
  Serial.println("Going_Right");
  // Rgt movement instructions
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  /*
  delay(500);// default delay for smooth rotation.
  */
  // NOTE: The minimum delay for RIGHT/LEFT movement is 1S(inluding .5s ON time & .5s OFF time). Hence subtract 1s before repeating this movement 

}

void backward(void) {
  Serial.println("Going_Backward");
 // Backward movement instructions
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void robostop(void) {//movement_Inst_Stp
  Serial.println("Stopping");    
  
  // Stop movement instructions
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
