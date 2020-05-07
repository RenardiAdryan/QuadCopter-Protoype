#include "header.h"

RF24 radio(CE,CSN); // Create your nRF24 object or wireless SPI connection
// Use the same address for both devices
uint8_t address[] = { "skyWalker" };

volatile uint32_t round_trip_timer = 0;

int throttle,speedMode,yawSetHIGH,yawSetLOW,pitchSet,rollSet,receiverStatus;
uint8_t IMUdata[7];
char flag;
float nowtime,prevtime,deltatime,safeTimer,sendTimer;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Setup and configure rf radio
  radio.begin();
  radio.setPALevel(RF24_PA_MIN); //Set power level to low, won't work well at higher levels (interfer with receiver)
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
 nowtime=millis();
 deltatime = nowtime-prevtime;
 prevtime = nowtime;

 //Unconnected detection
 safeTimer+=deltatime;
 static unsigned char flaginitial=1;
 if((safeTimer)>500){
    receiverStatus = 0;
    digitalWrite(BUZZER,HIGH);
     flaginitial = 0;
    
  }
  else{ 
        receiverStatus = 1;
        if(flaginitial==0){ 
       
          flaginitial=1;
          }
           
        digitalWrite(BUZZER,LOW);
        
        }

  sendTimer+=deltatime;
  if(sendTimer>10){

    
               Serial.write('#');
               Serial.write(throttle);
               Serial.write(receiverStatus);
               Serial.write(speedMode);
               
               Serial.write(yawSetHIGH);
               Serial.write(yawSetLOW);
               
               Serial.write(pitchSet);
               Serial.write(rollSet);
               Serial.write('!');

//              Serial.write('#');
//               Serial.write(0x30);
//               Serial.write(0x31);
//               Serial.write(0x32);
//               Serial.write(0x33);
//               Serial.write(0x34);
//               Serial.write(0x35);
//               Serial.write('!');


//               Serial.print('#');Serial.print(" | ");
//               Serial.print(throttle);Serial.print(" | ");
//               Serial.print(receiverStatus);Serial.print(" | ");
//               Serial.print(speedMode);Serial.print(" | ");
//               Serial.print(yawSet);Serial.print(" | ");
//               Serial.print(pitchSet);Serial.print(" | ");
//               Serial.print(rollSet);Serial.print(" | ");
//               Serial.println('!');
               
               
    sendTimer=0;
    }            

               
}




void serialEvent(){
  static char i=10,flagSend;
   if(Serial.available()){
          // Parsing in the Future
          unsigned char incomingByte = Serial.read(); 
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

          if(incomingByte == 0xAA && i>=6){i=0;}
          else {
            IMUdata[i] = incomingByte;
            if (i >=6 && incomingByte == 0x55){ flagSend = 1; }
            i++;
            }
  } 
}




void check_radio() {
static int i;
bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);                     // What happened?

  // If data is available, handle it accordingly
  if ( rx ){
    
    if(radio.getDynamicPayloadSize() < 1){
      // Corrupt payload has been flushed
      // Serial.println("Corrupt");
      return; 
    }
    
    digitalWrite(LED1, !digitalRead(LED1));
    
    // Read in the data
  static  uint8_t received[8];
  radio.read(received,sizeof(received));    

    if(received[0] == 255 && received[7] == 254){
      
     throttle  = received[1];
     speedMode = received[2];
     yawSetHIGH   = received[3];
     yawSetLOW    = received[4];
     pitchSet  = received[5];
     rollSet   = received[6];
    }
    else if(received[0] == 253 && received[7]==252){ 
      safeTimer =0;
        radio.stopListening();
        // Normal delay will not work here, so cycle through some no-operations (16nops @16mhz = 1us delay)
        for(uint32_t i=0; i<130;i++){
           __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
        }
         radio.startWrite( &IMUdata, sizeof(IMUdata),0 );//Content
     }    
  }
  
    // Start listening if transmission is complete
    if( tx || fail ){
    radio.startListening(); 
    //Serial.println(tx ? F(":OK") : F(":Fail"));
    }  
}
