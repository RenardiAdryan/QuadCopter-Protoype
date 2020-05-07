#include "header.h"

RF24 radio(CE,CSN); // Create your nRF24 object or wireless SPI connection
// Use the same address for both devices
uint8_t address[] = { "skyWalker" };


uint8_t Datareceived[7];
int throttle,speedMode,yawSetHIGH,yawSetLOW,pitchSet,rollSet;

float nowTime,prevTime,deltaTime,timer,receivedTimeOut,commandTimeOut;
uint8_t DataSend[8];
unsigned char flagSelect,flagCommand,flagReceived=1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Setup and configure rf radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //Set power level to low, won't work well at higher levels (interfer with receiver)
  // Use dynamic payloads to improve response time
  radio.enableDynamicPayloads();
  radio.openWritingPipe(address);             // communicate back and forth.  One listens on it, the other talks to it.
  radio.openReadingPipe(1,address); 
  radio.setDataRate(RF24_250KBPS); 
  radio.setChannel(125);  //2.525 GHz  
  radio.startListening();

  attachInterrupt(0, check_radio, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver
  pinMode(PUSHBUTTON1,INPUT_PULLUP);
  pinMode(PUSHBUTTON2,INPUT_PULLUP);
  pinMode(LED1,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
      nowTime = micros();
      deltaTime = nowTime - prevTime;
      prevTime = nowTime;

      timer+=deltaTime;
      if(timer>20000){timer=0;
  
          if(flagSelect == 0 && flagReceived==1 && flagCommand == 1){ 

                    radio.stopListening();
                    DataSend[0] = 255;
                    DataSend[1] = throttle;
                    DataSend[2] = speedMode;
                    DataSend[3] = yawSetHIGH;
                    DataSend[4] = yawSetLOW;
                    DataSend[5] = pitchSet;
                    DataSend[6] = rollSet;
                    DataSend[7] = 254;
                      
             
                    radio.startWrite(DataSend,sizeof(DataSend),0);

                    flagSelect=1;
                    flagReceived=0;
                    flagCommand=0;
                    
                }
          else if(flagSelect==1){

                    radio.stopListening();
                    DataSend[0] = 253;
                    DataSend[7] = 252;
                    radio.startWrite( DataSend,sizeof(DataSend),0);

                    flagSelect=0;
            } 

         if(flagReceived == 0){if(receivedTimeOut++>2){receivedTimeOut=0;flagReceived=1;}}//8ms
         if(flagCommand == 0){if(commandTimeOut++>2){commandTimeOut=0;flagSelect=1;}}//8ms
      
//                  Serial.print(throttle);
//                    Serial.print(" | ");
//                    
//                    Serial.print(speedMode);
//                    Serial.print(" | ");
//
//                    Serial.print(yawSet);
//                    Serial.print(" | ");
//
//                    Serial.print(pitchSet);
//                    Serial.print(" | ");
//
//                   Serial.println(rollSet);
                   
                    
      }
}


void serialEvent(){
  static unsigned char flagParsing,count=4;
  static unsigned char dataReceived[6];
   if(Serial.available()){
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          // Parsing in the Future
         unsigned char incomingByte = Serial.read(); 
         if(incomingByte == '!' && count>=5){count=0;}
         else{  
                dataReceived[count] = incomingByte; 
                if(count>4){flagParsing=1;}
                count++;
          }

          
          if(flagParsing==1) {
          
          throttle = dataReceived[0];
          speedMode = dataReceived[1];
          yawSetHIGH   = dataReceived[2]; 
          yawSetLOW   = dataReceived[3]; 
          pitchSet = dataReceived[4];
          rollSet  = dataReceived[5];
           
          flagParsing=0;flagCommand =1;
          
          }


          
    } 
}


void check_radio() {
static char i,flagHeader;
bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);                     // What happened?

  // If data is available, handle it accordingly
  if ( rx ){

    if(radio.getDynamicPayloadSize() < 1){
      // Corrupt payload has been flushed
      return; 
    }
     digitalWrite(LED1, !digitalRead(LED1));
    // Read in the data
    uint8_t received[7];
    radio.read(&received,sizeof(received));
    Datareceived[0] = 0xAA;
    Datareceived[1] = received[0];
    Datareceived[2] = received[1];
    Datareceived[3] = received[2];
    Datareceived[4] = received[3];
    Datareceived[5] = received[4];
    Datareceived[6] = received[5];
    Datareceived[7] = received[6];
     
     flagReceived =1;
      Serial.write(Datareceived[0]);//0xAA
      Serial.write(Datareceived[1]);//Hyaw
      Serial.write(Datareceived[2]);//Lyaw
      Serial.write(Datareceived[3]);//HPitch
      Serial.write(Datareceived[4]);//LPitch
      Serial.write(Datareceived[5]);//HRoll
      Serial.write(Datareceived[6]);//LRoll
      Serial.write(Datareceived[7]);//0x55

     //Debug Only//
//      Serial.print(Datareceived[0]);//0xAA
//      Serial.print("|");
//      Serial.print(Datareceived[1]);//Hyaw
//      Serial.print("|");
//      Serial.print(Datareceived[2]);//Lyaw
//      Serial.print("|");
//      Serial.print(Datareceived[3]);//HPitch
//      Serial.print("|");
//      Serial.print(Datareceived[4]);//LPitch
//      Serial.print("|");
//      Serial.print(Datareceived[5]);//HRoll
//      Serial.print("|");
//      Serial.print(Datareceived[6]);//LRoll
//      Serial.print("|");
//      Serial.println(Datareceived[7]);//0x55
//     

  }
    // Start listening if transmission is complete
    if( tx || fail ){
     // Serial.println("MASUK TX || FAIL");
      radio.startListening(); 
      //Serial.println(tx ? F(":OK") : F(":Fail"));
    } 
}
