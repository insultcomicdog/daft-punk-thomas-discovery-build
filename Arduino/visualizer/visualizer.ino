#include <HT1632.h>


char EQ_BAR [] = {0b1111, 0b1111, 0b1111, 0b1111};
#define EQ_BAR_WIDTH 	 2
#define EQ_BAR_HEIGHT 	 8

int i = 0;

void setup () {
  Serial.begin(9600);
  HT1632.begin(8, 7, 6);
  
  HT1632.clear();
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 0, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 3, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 6, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 9, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 12, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 15, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 18, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 21, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 24, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 27, 0);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 30, 0);
  HT1632.render();
}

void loop () {
  
  // Simple rendering example
  HT1632.clear();
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 0, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 3, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 6, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 9, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 12, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 15, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 18, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 21, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 24, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 27, i);
  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 30, i);
  HT1632.render();
  ++i;
  //*/

  delay(200);
  //delay(10);
}
