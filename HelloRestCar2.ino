#define DEBUG true
#include <Arduino.h>
#include <NewPing.h>
#define DEBUG true
#include <string.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "SSID.h"
#include <ArduinoHttpClient.h>

///////please enter your sensitive data in the Secret tab/arduino_secrets.h


#define TRIGGER_PIN_FRONT  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_FRONT     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TRIGGER_PIN_REAR  18 
#define ECHO_PIN_REAR 19
//#include "HTTPClient.h"
#include <string.h>
#include "SSID.h"
#define BEHOST "robo.sukelluspaikka.fi"
#define HTTP_MAX_BUF_LEN  130
#define MAX_DIST_CM 150
#define MANUAL_MODE_MAXDUR  300000l
#define ACTION_TIMEOUT 10000l
#define LED_RED 7
#define LED_YELLOW 6
#define LED_GREEN 5
/// wifi params

//HTTPClient http;
//char twenty [50];
String action ="";
int actionNum =0;
unsigned long actionStarted = 0;
bool seqEndReported = false;
bool manualMode = false;
long manualModeStartTime =0;

bool send =true;

/*IPAddress server(192,168,32,87); **/
//server robo.sukelluspaikka.fi 
char server[] = "robo.sukelluspaikka.fi";   
//WiFiSSLClient client;
WiFiClient cli;
HttpClient client = HttpClient(cli,server,80);
int status = WL_IDLE_STATUS;
int lastDirection =0; //0rot or stop, 1 fwd,2 bwd

/**
 * Sonar
*/
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRear(TRIGGER_PIN_REAR, ECHO_PIN_REAR, MAX_DISTANCE);

bool trafficLight(bool isFront =false, bool accurate =false){
  int sonar_cm = 0;
  if(isFront){
    sonar_cm = (int) sonarFront.ping_cm();
    //log("Sonar front:" + (String) sonar_cm + " cm");
  }else{
    sonar_cm = (int) sonarRear.ping_cm();
    //log("Sonar back:" + (String) sonar_cm + " cm");
  }
  
  

  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_YELLOW,LOW);
  digitalWrite(LED_RED,LOW);
  if(sonar_cm >0 && sonar_cm < 20){
    digitalWrite(LED_RED,HIGH);
    return true;
  }else if (sonar_cm >0 && sonar_cm < 100){
    digitalWrite(LED_YELLOW,HIGH);
  }else{
    digitalWrite(LED_GREEN,HIGH);
  }
  return false;
  
}

/************
 * Motor
 */

const byte MOTOR_A = 3;  // Motor 1 Interrupt Pin - INT 1 - LEFT Motor (patterit eessä)
const byte MOTOR_B = 2;  // Motor 2 Interrupt Pin - INT 0 - RIGHT Motor
const byte MOTOR_ADIR = 8;
const byte MOTOR_B_DIR =4;
// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;


// Motor A

int enA = 10; //A on vasen puoli kun patterit edessä
int in1 = 14; //high --> taakse
int in2 = 15; //high --> eteen

// Motor B

int enB = 9;
int in3 = 16;
int in4 = 17;

// Motor A pulse count ISR
void ISR_countA()  
{
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  
{
  counter_B++;  // increment Motor B counter value
  
}
// Function to convert from centimeters to steps
int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = 19.47; //(wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result

}


void MoveForward(int mspeed) {
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  trafficLight((mspeed>0),true);
  if(mspeed>0){ //update global
    lastDirection =1;
  }else{
    lastDirection =2;
  }

  uint8_t dir1=mspeed<0?LOW:HIGH;
  uint8_t dir2=mspeed<0?HIGH:LOW;
  mspeed=mspeed<0?-1*mspeed:mspeed;
  
  // Set Motor A direction
   digitalWrite(in1, dir1);
   digitalWrite(in2, dir2);
   // Set Motor B direction
   digitalWrite(in3, dir1);
   digitalWrite(in4, dir2);

   analogWrite(enA, mspeed);
   analogWrite(enB, mspeed);


}

// Function to Move Forward
void MoveForward(int steps, int mspeed) 
{
  
   
   //uint8_t dir1=steps<0||mspeed<0?LOW:HIGH;
   //uint8_t dir2=steps<0||mspeed<0?HIGH:LOW;

   mspeed=steps<0?-1*mspeed:mspeed;
   steps=steps<0?-1*steps:steps;

   bool isForward = mspeed>0;
   MoveForward(mspeed); 
   mspeed = isForward?mspeed:-1*mspeed;
   /*mspeed=mspeed<0?-1*mspeed:mspeed;
   // Set Motor A direction
   digitalWrite(in1, dir1);
   digitalWrite(in2, dir2);

   // Set Motor B direction
   digitalWrite(in3, dir1);
   digitalWrite(in4, dir2);*/
   
   // Go forward until step value is reached
   while (steps > counter_A && steps > counter_B) {
    bool shouldStop = trafficLight(isForward,false);
    if(shouldStop){
      break;
    }
    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
   }
    
  stopMotors();

}

/**
 * unlimited curving movement
 */

void moveLR(int speedL,int speedR){
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  uint8_t dir1L=speedL<0?LOW:HIGH;
  uint8_t dir2L=speedL<0?HIGH:LOW;
  uint8_t dir1R=speedR<0?LOW:HIGH;
  uint8_t dir2R=speedR<0?HIGH:LOW;

  
  if(speedL>0 &&speedR>0){
    trafficLight(true,true);
    lastDirection=1;
  }else if(speedL<0 && speedR <0){
    trafficLight(false,true);
    lastDirection=2;
  }


  speedL=speedL<0?-1*speedL:speedL;
  speedR=speedR<0?-1*speedR:speedR; 

  

  // Set Motor A direction
  digitalWrite(in1, dir1L);
  digitalWrite(in2, dir2L);

  // Set Motor B direction
  digitalWrite(in3, dir1R);
  digitalWrite(in4, dir2R);

  analogWrite(enA, speedL);
  analogWrite(enB, speedR);

}

//todo? nopeuserolla painotettu keskiarvo stepseistä
//molemmille puolille oma stepluku
void moveLRD(int speedL,int speedR, int steps){
 

  bool isRotation =(speedL>0 && speedR<0 ) || (speedL<0 && speedR>0) || (speedL==0 && speedR !=0) || (speedR==0 && speedR !=0);

  bool isFwd = (speedL>0 && speedR>0 && steps >0)|| (speedL<0 && speedR<0 && steps <0);

  speedL=steps<0?-1*speedL:speedL;
  speedR=steps<0?-1*speedR:speedR;

  moveLR(speedL, speedR);

  steps=steps<0?-1*steps:steps;
  

  
  
  // Go forward until step value is reached
  while (steps > counter_A && steps > counter_B) {
    if(!isRotation){
      bool shoudlStop = trafficLight(isFwd,false);
      if(shoudlStop){
        break;
      }
    }
    delay(1);    
  }

  stopMotors();


}

void stopMotors(){
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
  lastDirection =0;
}



////////////////////////////////////////////////////////////////


void setupWifi(){
  while (status != WL_CONNECTED) {
    status = WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(10000);
  }
  client.setTimeout(3000);
  printWiFiStatus();
}

void postSeqEnd(){
    String contentType = "application/json";
    String postData = "{\"seq\":\"end\"}";
    client.post("/seq/1/end",contentType, postData);
    int statusCode = client.responseStatusCode();
    String response = client.responseBody();
    log(response);
}




void log(String msg){
  //if(Serial.available()){
    Serial.println(msg);
  //}
}



void test(){
  //log("Front:");
  //Serial.print(sonarFront.ping_cm());
  //log("Rear");
  //Serial.print(sonarRear.ping_cm());

  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  digitalWrite(enA,LOW);
  digitalWrite(enB,LOW);  
//fwd left
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(enA,HIGH);
  delay(3000);
  //bwd left
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(enA,HIGH);
  delay(3000);
  //fwd left
  moveLR(130,0);
delay(3000);  
    moveLR(0,130); //fwd right
  delay(3000);
    moveLR(-130,0);//bwd left
delay(3000);
moveLR(0,-130); //bwd right
stopMotors();
//MoveForward(130,130);//fwd 130 steps
  //MoveForward(200);
  //delay(3000);
  //MoveForward(-200);
  //moveLR(120,120);
  //moveLR(110,150,100);
  //moveLR(110,154,100);
  //moveLR(110,170,100);
  //moveLR(-100,100,60);

}

void setup() {
  Serial.begin(115200);//use the hardware serial to communicate with the PC
 //Serial.begin(9600);//use the hardware serial to communicate with the PC
  log("setup()");
  // Attach the Interrupts to their ISR's
  //digitalPinToInterrupt (MOTOR_A)
  attachInterrupt(3, ISR_countA, CHANGE);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(2, ISR_countB, CHANGE);  // Increase counter B when speed sensor pin goes High
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  //test();
  delay(2000);
   setupWifi();
   log("Done");
}

void loop() {
  if(action.equals("") && send && 2048 > actionNum  ){
    String s = "/caction/" + (String)actionNum;
    log(s);
    client.get(s);
    int statusCode = client.responseStatusCode();
    action = client.responseBody();
    //client.res
    send =false;
    Serial.println("<Loop Action is:" +action +">");
    int str_len = action.length() + 1; 
    char char_array[str_len]; 
    action.toCharArray(char_array, str_len);

    char* ptr = strtok(char_array, "/");
    bool moveTypeStraight=false;
    bool distanceSpecified=false;
    bool speedLSpecified=false;
    bool speedRSpecified=false;
    bool speedSpecified=false;
    bool waitSpecified=false;
    int speed=0,speedL=0,speedR=0,distance=0, waitDur=0;
    //log("on loop start");
    while (ptr) {
      if(strcmp(ptr,"straight")==0){
        //log("straight -action");
        moveTypeStraight =true;
      } else if(strcmp(ptr,"speed")==0){
        //log("speed -action");
        speedSpecified =true;
        ptr = strtok(NULL, "/");
        speed=atoi(ptr);
        //Serial.println(ptr);
        //Serial.print("\r\n");
        //Serial.print(speed,DEC);
        //Serial.print("\r\n");
      } else if(strcmp(ptr,"speedL")==0){
        //log("speedL -action");
        moveTypeStraight =false;
        speedLSpecified =true;
        ptr = strtok(NULL, "/");
        speedL=atoi(ptr);
        Serial.print(speedL,DEC);
        Serial.print("\r\n");
      }else if(strcmp(ptr,"speedR")==0){
        //log("speedR -action");
        moveTypeStraight =false;
        speedLSpecified =true;
        ptr = strtok(NULL, "/");
        speedR=atoi(ptr);;
        //Serial.print(speedR,DEC);
        //Serial.print("\r\n");
      }else if(strcmp(ptr,"distance")==0){
        //log("distance-action");
        distanceSpecified=true;
        ptr = strtok(NULL, "/");
        distance=atoi(ptr);
        //Serial.print(distance,DEC);
        //Serial.print("\r\n");
      }else if(strcmp(ptr,"wait")==0){
        //log("seq-action");
        waitSpecified=true;
        ptr = strtok(NULL, "/");
        waitDur=atoi(ptr);
      }else if(strcmp(ptr,"halt")==0){
        //log("seq-action");
        stopMotors();
      }else if(strcmp(ptr,"seq")==0){
        log("seq-action");
      }else if(strcmp(ptr,"end")==0){
        log("end-action");
      }else if(strcmp(ptr,"manual")==0){
        //log("manual-mode-action");
        manualMode=true;
        manualModeStartTime =millis();
      }else if(strcmp(ptr,"automatic")==0){
        //log("manual-mode-action");
        manualMode=false;
        manualModeStartTime =0;
      }else {
        //log("other-action");
        //log(ptr);
      }
      ptr = strtok(NULL, "/");
    }

    //trafficLight();
    if(moveTypeStraight){
      if(distance != 0){
        int steps = CMtoSteps((float)distance);
        MoveForward(steps,speed);
      }else{
        MoveForward(speed);
      }
    }else if(speedLSpecified ||speedRSpecified){
      if(distanceSpecified){
        int steps = CMtoSteps((float)distance);
        moveLRD(speedL,speedR,steps);
      }else{
        moveLR(speedL,speedR);
      }
    }else if(waitSpecified){
      waitDur = waitDur>0?:2000;
      delay(waitDur);
    }
  } 

  if(action.endsWith("/seq/end")){
    log("sequence end catched!");
    if(!seqEndReported){
      //if( manualMode ==false || millis()>manualModeStartTime+(long)MANUAL_MODE_MAXDUR){
        postSeqEnd();
        manualMode=false;
        seqEndReported = true;
        actionNum =0;
      //}
    }
    action ="";
    send = true;
    actionNum = manualMode?actionNum:0;
    if(!manualMode){
      delay(500);
    }
  } else{
    actionStarted = millis();
    seqEndReported =false;
    //log("------<action:>-------");
    //log("Action on" +(String)action);
    //log("-------------------");
    action="";
    send=true;
    actionNum++;  
  }
  if(lastDirection >0){
    bool shouldStop = trafficLight(lastDirection==1,true);
    if(shouldStop){
      stopMotors();
    }
  }
  
  delay(100);
  //delay(3000);

}

void printWiFiStatus() {

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


