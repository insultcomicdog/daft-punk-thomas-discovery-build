#include <Wire.h>

#include <MuxShield.h>

//Initialize the Mux Shield
MuxShield muxShield;

const int animateID = 0;
const int micID = 1;
const int switchAnimationID = 2;
const int earModeID = 3;
const int earRedID = 4;
const int earGreenID = 5;
const int earBlueID = 6;
const int eqID = 7;
const int fanID = 8;
const int mp3ID = 9;

const long debounceInterval = 100;
unsigned long previousMillis = 0;


// rotatary switch
//unsigned short switchValue = 0;  // variable to store the value coming from the sensor
//int switchPin = 0;
//byte switchPosition = 0;                         // variable to store the rotary switch position
//byte switchPositionPrevious = 0;

// potentiometers
unsigned short pot0Value = 0;                  // variable to store the value coming from the sensor
int pot0Pin = 0;
byte pot0Position = 0;                         // variable to store the potentiometer position
byte pot0Previous = 0;

unsigned short pot1Value = 0;                  // variable to store the value coming from the sensor
int pot1Pin = 1;
byte pot1Position = 0;                         // variable to store the potentiometer position
byte pot1Previous = 0;

unsigned short pot2Value = 0; // variable to store the value coming from the sensor
int pot2Pin = 2;
byte pot2Position = 0;                         // variable to store the potentiometer position
byte pot2Previous = 0;

unsigned short pot3Value = 0; // variable to store the value coming from the sensor
int pot3Pin = 3;
byte pot3Position = 0;                         // variable to store the potentiometer position
byte pot3Previous = 0;

unsigned short pot4Value = 0; // variable to store the value coming from the sensor
int pot4Pin = 4;
byte pot4Position = 0;                         // variable to store the potentiometer position
byte pot4Previous = 0;

//tactile buttons
const int animationLEDPin = 0;
int animationButtonPin = 0;
int animationButtonState = 0;         // variable for reading the pushbutton status

const int micLEDPin = 1;
int micButtonPin = 1;
int micButtonState = 0;         // variable for reading the pushbutton status

const int eqLEDPin = 2;
int eqButtonPin = 2;
int eqButtonState = 0;         // variable for reading the pushbutton status

int eqNextModeButtonPin = 3;
int eqNextModeButtonState = 0;         // variable for reading the pushbutton status
int eqPrevModeButtonPin = 4;
int eqPrevModeButtonState = 0;         // variable for reading the pushbutton status
int eqSpeedUpButtonPin = 5;
int eqSpeedUpButtonState = 0;         // variable for reading the pushbutton status
int eqSpeedDownButtonPin = 6;
int eqSpeedDownButtonState = 0;         // variable for reading the pushbutton status
int fanButtonPin = 7;
int fanButtonState = 0;         // variable for reading the pushbutton status

const int mp3LEDPin = 3;
int mp3ButtonPin = 8;
int mp3ButtonState = 0;         // variable for reading the pushbutton status


boolean useMic = false;
boolean useEQ = true;
boolean useFan = false;
boolean useMP3 = false;

void setup() {
  
  muxShield.setMode(1,DIGITAL_IN_PULLUP);  //set I/O 1 as digital in with pullup
  muxShield.setMode(2,DIGITAL_OUT);        //set I/O 2 as digital output
  muxShield.setMode(3,ANALOG_IN);          //set I/O 3 as analog input
  
  //Serial.begin(28800);
  Serial.begin(9600);
//  Serial.println("setup");
  
  Wire.begin(); // join i2c bus (address optional for master)
}

void loop() {
  
  //Read analog inputs on IO3
  //switchValue = muxShield.analogReadMS(3, switchPin);  //IO3, pin 0
  pot0Value = muxShield.analogReadMS(3, pot0Pin);  //IO3, pin 0
  pot1Value = muxShield.analogReadMS(3, pot1Pin);  //IO3, pin 1
  pot2Value = muxShield.analogReadMS(3, pot2Pin);  //IO3, pin 2
  pot3Value = muxShield.analogReadMS(3, pot3Pin);  //IO3, pin 3
  pot4Value = muxShield.analogReadMS(3, pot4Pin);  //IO3, pin 4
  
  int animationButtonValue = muxShield.digitalReadMS(1, animationButtonPin);  //IO1, pin 0
  int micButtonValue = muxShield.digitalReadMS(1, micButtonPin);  //IO1, pin 1
  
  int eqButtonValue = muxShield.digitalReadMS(1, eqButtonPin);  //IO1, pin 2
  int eqNextModeButtonValue = muxShield.digitalReadMS(1, eqNextModeButtonPin);  //IO1, pin 3
  int eqPrevModeButtonValue = muxShield.digitalReadMS(1, eqPrevModeButtonPin);  //IO1, pin 4
  int eqSpeedUpButtonValue = muxShield.digitalReadMS(1, eqSpeedUpButtonPin);  //IO1, pin 5
  int eqSpeedDownButtonValue = muxShield.digitalReadMS(1, eqSpeedDownButtonPin);  //IO1, pin 6
  int fanButtonValue = muxShield.digitalReadMS(1, fanButtonPin);  //IO1, pin 7
  int mp3ButtonValue = muxShield.digitalReadMS(1, mp3ButtonPin);  //IO1, pin 8
  
// Serial.println("pot0Value");
// Serial.println(pot0Value);
  
//  Serial.println("animationButtonValue");
//  Serial.println(animationButtonValue);
//  
//  Serial.println("micButtonValue");
//  Serial.println(micButtonValue);
  
  

  if (millis() - previousMillis >= debounceInterval) {
    
    // if the button state has changed:
    if (animationButtonValue != animationButtonState) {
      animationButtonState = animationButtonValue; 
    }
    
    // if the button state has changed:
    if (micButtonValue != micButtonState) {
      micButtonState = micButtonValue; 
    }
    
    // if the button state has changed:
    if (eqButtonValue != eqButtonState) {
        eqButtonState = eqButtonValue;
      
       if(eqButtonState == LOW){  
          if(!useEQ){ 
          useEQ  = true;
          sendCommand (eqID, 1);
        } else {
           useEQ  = false;
           sendCommand (eqID, 0);
        }
      }   
    }
    
    // if the button state has changed:
    if (eqNextModeButtonValue != eqNextModeButtonState) {
         eqNextModeButtonState = eqNextModeButtonValue;
         if(eqNextModeButtonState == LOW){  
           sendCommand (eqID, 2);
         }
    }
    
    // if the button state has changed:
    if (eqPrevModeButtonValue != eqPrevModeButtonState) {
         eqPrevModeButtonState = eqPrevModeButtonValue;
         if(eqPrevModeButtonState == LOW){  
           sendCommand (eqID, 3);
         }
    } 
    
    // if the button state has changed:
    if (eqSpeedDownButtonValue != eqSpeedDownButtonState) {
         eqSpeedDownButtonState = eqSpeedDownButtonValue;
         if(eqSpeedDownButtonState == LOW){  
           sendCommand (eqID, 4);
         }
    }
    
    // if the button state has changed:
    if (eqSpeedUpButtonValue != eqSpeedUpButtonState) {
         eqSpeedUpButtonState = eqSpeedUpButtonValue;
         if(eqSpeedUpButtonState == LOW){  
           sendCommand (eqID, 5);
         }
    }
    
    // if the button state has changed:
    if (fanButtonValue != fanButtonState) {
        fanButtonState = fanButtonValue;
         
        if(fanButtonState == LOW){  
          if(!useFan){ 
            useFan  = true;
            sendCommand (fanID, 1);
          } else {
            useFan  = false;
            sendCommand (fanID, 0);
          }
        }   
    }
    
    // if the button state has changed:
    if (mp3ButtonValue != mp3ButtonState) {
        mp3ButtonState = mp3ButtonValue;
         
        if(mp3ButtonState == LOW){  
          if(!useMP3){ 
            useMP3  = true;
            sendCommand (mp3ID, 1);
//            Serial.println("mp3ID");
//            Serial.println(mp3ID);
//            Serial.println("value");
//            Serial.println(1);
          } else {
            useMP3  = false;
            sendCommand (mp3ID, 0);
//            Serial.println("mp3ID");
//            Serial.println(mp3ID);
//            Serial.println("value");
//            Serial.println(0);
          }         
        }   
    }
    
    if (animationButtonState == HIGH && micButtonState == LOW) {   
     
     if(useMic){ 
        useMic  = false;
        sendCommand (animateID, 1);
     }

    } else if (animationButtonState == LOW && micButtonState == HIGH) {
       if(!useMic){ 
        useMic  = true;
        sendCommand (micID, 1);
       }
    }
    
    if(useEQ){
       muxShield.digitalWriteMS(2, eqLEDPin, HIGH);  //IO2, pin 2
    } else{
       muxShield.digitalWriteMS(2, eqLEDPin, LOW);  //IO2, pin 2
    }
    
    if(useMP3){
       muxShield.digitalWriteMS(2, mp3LEDPin, HIGH);  //IO2, pin 3
    } else{
       muxShield.digitalWriteMS(2, mp3LEDPin, LOW);  //IO2, pin 3
    }

    if(!useMic){
      muxShield.digitalWriteMS(2, animationLEDPin, HIGH);  //IO2, pin 0
      muxShield.digitalWriteMS(2, micLEDPin, LOW);  //IO2, pin 1      
    } else {
      muxShield.digitalWriteMS(2, animationLEDPin, LOW);  //IO2, pin 0
      muxShield.digitalWriteMS(2, micLEDPin, HIGH);  //IO2, pin 1      
    }
    
//    if (switchValue < 50) {                          // switch position 1
//      switchPosition = 1;
//      //Serial.println("switch position = 1");
//    }
//    if (switchValue > 50 && switchValue < 140) {     // switch position 2
//      switchPosition = 2;
//      //Serial.println("switch position = 2");
//    }
//    if (switchValue > 141 && switchValue < 230) {    // switch position 3
//      switchPosition = 3;
//      //Serial.println("switch position = 3");
//    }
//    if (switchValue > 231 && switchValue < 330) {    // switch position 4
//      switchPosition = 4;
//      //Serial.println("switch position = 4");
//    }
//    if (switchValue > 331 && switchValue < 420) {    // switch position 5
//      switchPosition = 5;
//      //Serial.println("switch position = 5");
//    }
//    if (switchValue > 421 && switchValue < 510) {    // switch position 6
//      switchPosition = 6;
//      //Serial.println("switch position = 6");
//    }
//    if (switchValue > 511 && switchValue < 600) {    // switch position 7
//      switchPosition = 7;
//      //Serial.println("switch position = 7");
//    }
//    if (switchValue > 601 && switchValue < 700) {    // switch position 8
//      switchPosition = 8;
//      //Serial.println("switch position = 8");
//    }
//    if (switchValue > 701 && switchValue < 790) {    // switch position 9
//      switchPosition = 9;
//      //Serial.println("switch position = 9");
//    }
//    if (switchValue > 791 && switchValue < 890) {    // switch position 10
//      switchPosition = 10;
//      //Serial.println("switch position = 10");
//    }
//    if (switchValue > 891 && switchValue < 970) {    // switch position 11
//      switchPosition = 11;
//      //Serial.println("switch position = 11");
//    }
//    if (switchValue > 971 && switchValue < 1023) {   // switch position 12
//      switchPosition = 12;
//      //Serial.println("switch position = 12");
//    }


    if (pot0Value < 64) {                         // potentiometer position 0
      pot0Position = 0;
      //Serial.println("potentiometer position = 1");
    }
    if (pot0Value >  64 && pot0Value < 127) {     // potentiometer position 1
      pot0Position = 1;
      //Serial.println("potentiometer position = 2");
    }
    if (pot0Value > 128 && pot0Value < 191) {    // potentiometer position 2
      pot0Position = 2;
      //Serial.println("potentiometer position = 3");
    }
    if (pot0Value > 192 && pot0Value < 255) {    // potentiometer position 3
      pot0Position = 3;
      //Serial.println("potentiometer position = 4");
    }
    if (pot0Value > 256 && pot0Value < 319) {    // potentiometer position 4
      pot0Position = 4;
      //Serial.println("potentiometer position = 5");
    }
    if (pot0Value > 320 && pot0Value < 383) {    // potentiometer position 5
      pot0Position = 5;
      //Serial.println("potentiometer position = 6");
    }
    if (pot0Value > 384 && pot0Value < 447) {    // potentiometer position 6
      pot0Position = 6;
      //Serial.println("potentiometer position = 7");
    }
    if (pot0Value > 448 && pot0Value < 511) {    // potentiometer position 7
      pot0Position = 7;
      //Serial.println("potentiometer position = 8");
    }
    if (pot0Value > 512 && pot0Value < 575) {    // potentiometer position 8
      pot0Position = 8;
      //Serial.println("potentiometer position = 9");
    }
    if (pot0Value > 576 && pot0Value < 639) {    // potentiometer position 11
      pot0Position = 9;
      //Serial.println("potentiometer position = 10");
    }
    if (pot0Value > 640 && pot0Value < 703) {    // potentiometer position 10
      pot0Position = 10;
      //Serial.println("potentiometer position = 11");
    }
    if (pot0Value > 704 && pot0Value < 815) {    // potentiometer position 11
      pot0Position = 11;
      //Serial.println("potentiometer position = 12");
    }
    if (pot0Value > 768 && pot0Value < 831) {    // potentiometer position 12
      pot0Position = 12;
      //Serial.println("potentiometer position = 13");
    }
    if (pot0Value > 832 && pot0Value < 951) {    // potentiometer position 13
      pot0Position = 13;
      //Serial.println("potentiometer position = 14");
    }
    if (pot0Value > 952 && pot0Value < 1000) {    // potentiometer position 14
      pot0Position = 14;
      //Serial.println("potentiometer position = 14");
    }
    if (pot0Value > 1001 && pot0Value < 1023) {    // potentiometer position 15
      pot0Position = 15;
      //Serial.println("potentiometer position = 15");
    }


    if (pot1Value < 205) {                          // potentiometer position 1
      pot1Position = 1;
      //Serial.println("potentiometer position = 1");
    }
    if (pot1Value > 205 && pot1Value < 410) {     // potentiometer position 2
      pot1Position = 2;
      //Serial.println("potentiometer position = 2");
    }
    if (pot1Value > 411 && pot1Value < 615) {    // potentiometer position 3
      pot1Position = 3;
      //Serial.println("potentiometer position = 3");
    }
    if (pot1Value > 616 && pot1Value < 820) {    // potentiometer position 4
      pot1Position = 4;
      //Serial.println("potentiometer position = 4");
    }
    if (pot1Value > 821 && pot1Value < 1023) {    // potentiometer position 5
      pot1Position = 5;
      //Serial.println("potentiometer position = 5");
    }
    
    if (pot2Value < 64) {                         // potentiometer position 0
      pot2Position = map(0, 0, 1023, 0, 255);
    } else if (pot2Value >  64 && pot2Value < 127) {     // potentiometer position 1
      pot2Position = map(64, 0, 1023, 0, 255);
    } else if (pot2Value > 128 && pot2Value < 191) {    // potentiometer position 2
       pot2Position = map(128, 0, 1023, 0, 255);
    } else if (pot2Value > 192 && pot2Value < 255) {    // potentiometer position 3
       pot2Position = map(192, 0, 1023, 0, 255);
    } else if (pot2Value > 256 && pot2Value < 319) {    // potentiometer position 4
       pot2Position = map(256, 0, 1023, 0, 255);
    } else if (pot2Value > 320 && pot2Value < 383) {    // potentiometer position 5
        pot2Position = map(320, 0, 1023, 0, 255);
    } else if (pot2Value > 384 && pot2Value < 447) {    // potentiometer position 6
       pot2Position = map(384, 0, 1023, 0, 255);
    } else if (pot2Value > 448 && pot2Value < 511) {    // potentiometer position 7
       pot2Position = map(448, 0, 1023, 0, 255);
    } else if (pot2Value > 512 && pot2Value < 575) {    // potentiometer position 8
       pot2Position = map(512, 0, 1023, 0, 255);
    } else if (pot2Value > 576 && pot2Value < 639) {    // potentiometer position 11
       pot2Position = map(576, 0, 1023, 0, 255);
    } else if (pot2Value > 640 && pot2Value < 703) {    // potentiometer position 10
       pot2Position = map(640, 0, 1023, 0, 255);
    } else if (pot2Value > 704 && pot2Value < 815) {    // potentiometer position 11
       pot2Position = map(704, 0, 1023, 0, 255);
    } else if (pot2Value > 768 && pot2Value < 831) {    // potentiometer position 12
       pot2Position = map(768, 0, 1023, 0, 255);
    } else if (pot2Value > 832 && pot2Value < 951) {    // potentiometer position 13
       pot2Position = map(832, 0, 1023, 0, 255);
    } else if (pot2Value > 952 && pot2Value < 1000) {    // potentiometer position 14
       pot2Position = map(952, 0, 1023, 0, 255);
    } else if (pot2Value > 1001 && pot2Value < 1023) {    // potentiometer position 15
       pot2Position = map(1023, 0, 1023, 0, 255);
    }
    
    if (pot2Value < 64) {                         // potentiometer position 0
      pot2Position = map(0, 0, 1023, 0, 255);
    } else if (pot2Value >  64 && pot2Value < 127) {     // potentiometer position 1
      pot2Position = map(64, 0, 1023, 0, 255);
    } else if (pot2Value > 128 && pot2Value < 191) {    // potentiometer position 2
       pot2Position = map(128, 0, 1023, 0, 255);
    } else if (pot2Value > 192 && pot2Value < 255) {    // potentiometer position 3
       pot2Position = map(192, 0, 1023, 0, 255);
    } else if (pot2Value > 256 && pot2Value < 319) {    // potentiometer position 4
       pot2Position = map(256, 0, 1023, 0, 255);
    } else if (pot2Value > 320 && pot2Value < 383) {    // potentiometer position 5
        pot2Position = map(320, 0, 1023, 0, 255);
    } else if (pot2Value > 384 && pot2Value < 447) {    // potentiometer position 6
       pot2Position = map(384, 0, 1023, 0, 255);
    } else if (pot2Value > 448 && pot2Value < 511) {    // potentiometer position 7
       pot2Position = map(448, 0, 1023, 0, 255);
    } else if (pot2Value > 512 && pot2Value < 575) {    // potentiometer position 8
       pot2Position = map(512, 0, 1023, 0, 255);
    } else if (pot2Value > 576 && pot2Value < 639) {    // potentiometer position 11
       pot2Position = map(576, 0, 1023, 0, 255);
    } else if (pot2Value > 640 && pot2Value < 703) {    // potentiometer position 10
       pot2Position = map(640, 0, 1023, 0, 255);
    } else if (pot2Value > 704 && pot2Value < 815) {    // potentiometer position 11
       pot2Position = map(704, 0, 1023, 0, 255);
    } else if (pot2Value > 768 && pot2Value < 831) {    // potentiometer position 12
       pot2Position = map(768, 0, 1023, 0, 255);
    } else if (pot2Value > 832 && pot2Value < 951) {    // potentiometer position 13
       pot2Position = map(832, 0, 1023, 0, 255);
    } else if (pot2Value > 952 && pot2Value < 1000) {    // potentiometer position 14
       pot2Position = map(952, 0, 1023, 0, 255);
    } else if (pot2Value > 1001 && pot2Value < 1023) {    // potentiometer position 15
       pot2Position = map(1023, 0, 1023, 0, 255);
    }
    
    if (pot3Value < 64) {                         // potentiometer position 0
       pot3Position = map(0, 0, 1023, 0, 255);
    } else if (pot3Value >  64 && pot3Value < 127) {     // potentiometer position 1
       pot3Position = map(64, 0, 1023, 0, 255);
    } else if (pot3Value > 128 && pot3Value < 191) {    // potentiometer position 2
       pot3Position = map(128, 0, 1023, 0, 255);
    } else if (pot3Value > 192 && pot3Value < 255) {    // potentiometer position 3
       pot3Position = map(192, 0, 1023, 0, 255);
    } else if (pot3Value > 256 && pot3Value < 319) {    // potentiometer position 4
       pot3Position = map(256, 0, 1023, 0, 255);
    } else if (pot3Value > 320 && pot3Value < 383) {    // potentiometer position 5
       pot3Position = map(320, 0, 1023, 0, 255);
    } else if (pot3Value > 384 && pot3Value < 447) {    // potentiometer position 6
       pot3Position = map(384, 0, 1023, 0, 255);
    } else if (pot3Value > 448 && pot3Value < 511) {    // potentiometer position 7
       pot3Position = map(448, 0, 1023, 0, 255);
    } else if (pot3Value > 512 && pot3Value < 575) {    // potentiometer position 8
       pot3Position = map(512, 0, 1023, 0, 255);
    } else if (pot3Value > 576 && pot3Value < 639) {    // potentiometer position 11
       pot3Position = map(576, 0, 1023, 0, 255);
    } else if (pot3Value > 640 && pot3Value < 703) {    // potentiometer position 10
       pot3Position = map(640, 0, 1023, 0, 255);
    } else if (pot3Value > 704 && pot3Value < 815) {    // potentiometer position 11
       pot3Position = map(704, 0, 1023, 0, 255);
    } else if (pot3Value > 768 && pot3Value < 831) {    // potentiometer position 12
       pot3Position = map(768, 0, 1023, 0, 255);
    } else if (pot3Value > 832 && pot3Value < 951) {    // potentiometer position 13
       pot3Position = map(832, 0, 1023, 0, 255);
    } else if (pot3Value > 952 && pot3Value < 1000) {    // potentiometer position 14
       pot3Position = map(952, 0, 1023, 0, 255);
    } else if (pot3Value > 1001 && pot3Value < 1023) {    // potentiometer position 15
       pot3Position = map(1023, 0, 1023, 0, 255);
    }
    
    if (pot4Value < 64) {                         // potentiometer position 0
       pot4Position = map(0, 0, 1023, 0, 255);
    } else if (pot4Value >  64 && pot4Value < 127) {     // potentiometer position 1
       pot4Position = map(64, 0, 1023, 0, 255);
    } else if (pot4Value > 128 && pot4Value < 191) {    // potentiometer position 2
       pot4Position = map(128, 0, 1023, 0, 255);
    } else if (pot4Value > 192 && pot4Value < 255) {    // potentiometer position 3
       pot4Position = map(192, 0, 1023, 0, 255);
    } else if (pot4Value > 256 && pot4Value < 319) {    // potentiometer position 4
       pot4Position = map(256, 0, 1023, 0, 255);
    } else if (pot4Value > 320 && pot4Value < 383) {    // potentiometer position 5
       pot4Position = map(320, 0, 1023, 0, 255);
    } else if (pot4Value > 384 && pot4Value < 447) {    // potentiometer position 6
       pot4Position = map(384, 0, 1023, 0, 255);
    } else if (pot4Value > 448 && pot4Value < 511) {    // potentiometer position 7
       pot4Position = map(448, 0, 1023, 0, 255);
    } else if (pot4Value > 512 && pot4Value < 575) {    // potentiometer position 8
       pot4Position = map(512, 0, 1023, 0, 255);
    } else if (pot4Value > 576 && pot4Value < 639) {    // potentiometer position 11
       pot4Position = map(576, 0, 1023, 0, 255);
    } else if (pot4Value > 640 && pot4Value < 703) {    // potentiometer position 10
       pot4Position = map(640, 0, 1023, 0, 255);
    } else if (pot4Value > 704 && pot4Value < 815) {    // potentiometer position 11
       pot4Position = map(704, 0, 1023, 0, 255);
    } else if (pot4Value > 768 && pot4Value < 831) {    // potentiometer position 12
       pot4Position = map(768, 0, 1023, 0, 255);
    } else if (pot4Value > 832 && pot4Value < 951) {    // potentiometer position 13
       pot4Position = map(832, 0, 1023, 0, 255);
    } else if (pot4Value > 952 && pot4Value < 1000) {    // potentiometer position 14
       pot4Position = map(952, 0, 1023, 0, 255);
    } else if (pot4Value > 1001 && pot4Value < 1023) {    // potentiometer position 15
       pot4Position = map(1023, 0, 1023, 0, 255);
    }
    
//    pot2Position = map(pot2Value, 0, 1023, 0, 255);
//    pot3Position = map(pot3Value, 0, 1023, 0, 255);
//    pot4Position = map(pot4Value, 0, 1023, 0, 255);

    previousMillis = millis();
  }
  
//  if (switchPosition != switchPositionPrevious) {
//    if(switchPosition == 1 && switchPositionPrevious == 12){
//       sendCommand(switchAnimationID, 1);
//    } else if (switchPosition == 12 && switchPositionPrevious == 1) {
//       sendCommand(switchAnimationID, 0);
//    } else {
//      (switchPosition > switchPositionPrevious) ? sendCommand(switchAnimationID, 1) : sendCommand(switchAnimationID, 0);
//    }
//    
//    switchPositionPrevious = switchPosition;
//  }

    if (pot0Position != pot0Previous) {
          //(pot0Position > pot0Previous) ? sendCommand(switchAnimationID, 1) : sendCommand(switchAnimationID, 0);
         sendCommand(switchAnimationID, pot0Position);
          
//         Serial.println("switchAnimationID");
//         Serial.println(switchAnimationID);
//         Serial.println("pot0Position");
//         Serial.println(pot0Position);


          pot0Previous = pot0Position;
      }
  
  if (pot1Position != pot1Previous) {
      sendCommand(earModeID, pot1Position);
      pot1Previous = pot1Position;
    }
    
    if (pot2Position != pot2Previous) {
       sendCommand(earRedID, pot2Position );
       pot2Previous = pot2Position;   
    }
    
    if (pot3Position != pot3Previous) {
       sendCommand(earGreenID, pot3Position );
       pot3Previous = pot3Position;   
    }
    if (pot4Position != pot4Previous) {
       sendCommand(earBlueID, pot4Position );
       pot4Previous = pot4Position;   
    }
}

void sendCommand (byte sensorID, byte value) {
   Serial.println("sensorID");
   Serial.println(sensorID);
   Serial.println("value");
   Serial.println(value);
  
   Wire.beginTransmission(4); // transmit to device #4
   Wire.write(sensorID);              // id for sensor
   Wire.write(value);              // sends one byte  
   Wire.endTransmission();    // stop transmitting
}
