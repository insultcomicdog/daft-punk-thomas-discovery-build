/***********************************************************************
 * HT1624.pde - Arduino demo program for Holtek HT1632 LED driver chip,
 *            As implemented on the Sure Electronics DE-DP016 display board
 *            (16*24 dot matrix LED module.)
 * Nov, 2008 by Bill Westfield ("WestfW")
 *   Copyrighted and distributed under the terms of the Berkely license
 *   (copy freely, but include this notice of original author.)
 *
 * Adapted for 8x32 display by FlorinC.
 ***********************************************************************/

// comment out this line for the 8x32 display;
//#define _16x24_

#include <WProgram.h>
#include "ht1632.h"
//#include <HT1632.h>
#include <avr/pgmspace.h>
#include "font3.h"
#include "myfont.h"

#ifdef _16x24_
  #define X_MAX 23
  #define Y_MAX 15
#else
  #define X_MAX 31
  #define Y_MAX 7
#endif

//(fc) switched to a different set of pins than the original, to accomodate the SD shield;
#define HT1632_DATA     6    // Data pin (pin 7)
#define HT1632_WRCLK    7    // Write clock pin (pin 5)
#define HT1632_CS       8    // Chip Select (1, 2, 3, or 4)

#define plot(x,y,v)  ht1632_plot(x,y,v)
#define cls          ht1632_clear

int DISPDELAY=0;
int frame = 0;
int n=0;
int loops=0;
int maxframes=0;
int framestart=0;
int typeanimation=0;
char *msg="Hello World";
int crtPos = 0;
int background=1;
int animCounter = 0;  //tek
#define MAXANIMATIONS 14




/***********************************************************************
 * ht1632_chipselect / ht1632_chipfree
 * Select or de-select a particular ht1632 chip.
 * De-selecting a chip ends the commands being sent to a chip.
 * CD pins are active-low; writing 0 to the pin selects the chip.
 ***********************************************************************/

void ht1632_chipselect(byte chipno)
{
  DEBUGPRINT("\nHT1632(%d) ", chipno);
  digitalWrite(chipno, 0);
}

void ht1632_chipfree(byte chipno)
{
  DEBUGPRINT(" [done %d]", chipno);
  digitalWrite(chipno, 1);
}


/*
 * we keep a copy of the display controller contents so that we can
 * know which bits are on without having to (slowly) read the device.
 * Note that we only use the low four bits of the shadow ram, since
 * we're shadowing 4-bit memory.  This makes things faster, and we
 * use the other half for a "snapshot" when we want to plot new data
 * based on older data...
 */
// (fc) covers the case for 32x8 as well (64 bytes, 4 bits)
byte ht1632_shadowram[96];  // our copy of the display's RAM


/*
 * ht1632_writebits
 * Write bits (up to 8) to h1632 on pins HT1632_DATA, HT1632_WRCLK
 * Chip is assumed to already be chip-selected
 * Bits are shifted out from MSB to LSB, with the first bit sent
 * being (bits & firstbit), shifted till firsbit is zero.
 */
void ht1632_writebits (byte bits, byte firstbit)
{
  DEBUGPRINT(" ");
  while (firstbit) {
    DEBUGPRINT((bits&firstbit ? "1" : "0"));
    digitalWrite(HT1632_WRCLK, LOW);
    if (bits & firstbit) {
      digitalWrite(HT1632_DATA, HIGH);
    } 
    else {
      digitalWrite(HT1632_DATA, LOW);
    }
    digitalWrite(HT1632_WRCLK, HIGH);
    firstbit >>= 1;
  }
}


/*
 * ht1632_sendcmd
 * Send a command to the ht1632 chip.
 * A command consists of a 3-bit "CMD" ID, an 8bit command, and
 * one "don't care bit".
 *   Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx Free
 */
static void ht1632_sendcmd (byte command)
{
  ht1632_chipselect(HT1632_CS);  // Select chip
  ht1632_writebits(HT1632_ID_CMD, 1<<2);  // send 3 bits of id: COMMMAND
  ht1632_writebits(command, 1<<7);  // send the actual command
  ht1632_writebits(0, 1); 	/* one extra dont-care bit in commands. */
  ht1632_chipfree(HT1632_CS); //done
}

/*
 * ht1632_clear
 * clear the display, and the shadow memory, and the snapshot
 * memory.  This uses the "write multiple words" capability of
 * the chipset by writing all 96 words of memory without raising
 * the chipselect signal.
 */
void ht1632_clear()
{
  char i;

  ht1632_chipselect(HT1632_CS);  // Select chip
  ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
  ht1632_writebits(0, 1<<6); // Send address
  for (i = 0; i < 96/2; i++) // Clear entire display
    ht1632_writebits(0, 1<<7); // send 8 bits of data
  ht1632_chipfree(HT1632_CS); // done
  for (i=0; i < 96; i++)
    ht1632_shadowram[i] = 0;
}


/*
 * ht1632_senddata
 * send a nibble (4 bits) of data to a particular memory location of the
 * ht1632.  The command has 3 bit ID, 7 bits of address, and 4 bits of data.
 *    Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 Free
 * Note that the address is sent MSB first, while the data is sent LSB first!
 * This means that somewhere a bit reversal will have to be done to get
 * zero-based addressing of words and dots within words.
 */
static void ht1632_senddata (byte address, byte data)
{
  ht1632_chipselect(HT1632_CS);  // Select chip
  ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6); // Send address
  ht1632_writebits(data, 1<<3); // send 4 bits of data
  ht1632_chipfree(HT1632_CS); // done
}

void ht1632_setup()
{
  pinMode(HT1632_CS, OUTPUT);
  digitalWrite(HT1632_CS, HIGH); 	/* unselect (active low) */
  pinMode(HT1632_WRCLK, OUTPUT);
  pinMode(HT1632_DATA, OUTPUT);
  ht1632_sendcmd(HT1632_CMD_SYSDIS);    // Disable system
  ht1632_sendcmd(HT1632_CMD_COMS00);    // 16*32, PMOS drivers
  //ht1632_sendcmd(HT1632_CMD_MSTMD); 	/* Master Mode */
  ht1632_sendcmd(HT1632_CMD_SYSEN); 	/* System on */
  ht1632_sendcmd(HT1632_CMD_LEDON); 	/* LEDs on */
  for (byte i=0; i<64; i++)
    ht1632_senddata(i, 0);  // clear the display!
  delay(1000);
  Serial.begin(9600);                    // DS1307 clock chip setup
  Serial.println("We wait a bit...");   
  delay(5000);
  Serial.println("Now we begin!");
}


/*
 * Copy a character glyph from the myfont data structure to
 * display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 */
void ht1632_putchar(int x, int y, char c,int background)
{
  // fonts defined for ascii 32 and beyond (index 0 in font array is ascii 32);
  byte charIndex;

  // replace undisplayable characters with blank;
  if (c < 32 || c > 126)
  {
    charIndex = 0;
  }
  else
  {
    charIndex = c - 32;
  }

  // move character definition, pixel by pixel, onto the display;
  // fonts are defined as one byte per row;
  for (byte row=0; row<8; row++)
  {
    byte rowDots = pgm_read_byte_near(&font3[charIndex][row]);
    
    for (byte col=0; col<6; col++)
    {
      if (rowDots & (1<<(5-col)))
        plot(x+col, y+row, 1*background);
      else
        plot(x+col, y+row, (1*background+1)%2);
    }
  }
}
void ht1632_plot (int x, int y, char val)
{
  if (x<0 || x>X_MAX || y<0 || y>Y_MAX)
     return;
  
  char addr, bitval;

  /*
   * The 4 bits in a single memory word go DOWN, with the LSB
   * (first transmitted) bit being on top.  However, writebits()
   * sends the MSB first, so we have to do a sort of bit-reversal
   * somewhere.  Here, this is done by shifting the single bit in
   * the opposite direction from what you might expect.
   */
  bitval = 8>>(y&3);  // compute which bit will need set

#ifdef _16x24_
  addr = (x<<2) + (y>>2);  // compute which memory word this is in
#else
// (fc)
  addr = (x<<1) + (y>>2);  // compute which memory word this is in
#endif

  if (val) {  // Modify the shadow memory
    ht1632_shadowram[addr] |= bitval;
  } 
  else {
    ht1632_shadowram[addr] &= ~bitval;
  }
  // Now copy the new memory value to the display
  ht1632_senddata(addr, ht1632_shadowram[addr]);
}


/*
 * get_shadowram
 * return the value of a pixel from the shadow ram.
 */
byte get_shadowram(byte x, byte y)
{
  byte addr, bitval;

  bitval = 8>>(y&3);  // compute which bit will need set
  addr = (x<<2) + (y>>2);  // compute which memory word this is in
  return (0 != (ht1632_shadowram[addr] & bitval));
}


/*
 * snapshot_shadowram
 * Copy the shadow ram into the snapshot ram (the upper bits)
 * This gives us a separate copy so we can plot new data while
 * still having a copy of the old data.  snapshotram is NOT
 * updated by the plot functions (except "clear")
 */
void snapshot_shadowram()
{
  for (char i=0; i< sizeof ht1632_shadowram; i++) {
    ht1632_shadowram[i] = (ht1632_shadowram[i] & 0x0F) | ht1632_shadowram[i] << 4;  // Use the upper bits
  }
}


/*
 * get_snapshotram
 * get a pixel value from the snapshot ram (instead of
 * the actual displayed (shadow) memory
 */
byte get_snapshotram(byte x, byte y)
{
  byte addr, bitval;

  bitval = 128>>(y&3);  // user upper bits!

#ifdef _16x24_
  addr = (x<<2) + (y>>2);  // compute which memory word this is in
#else
// (fc)
  addr = (x<<1) + (y>>2);  // compute which memory word this is in
#endif

  if (ht1632_shadowram[addr] & bitval)
    return 1;
  return 0;
}


/*
* This works equally well for both 16x24 and 8x32 matrices.
*/
void animationinfo(int animation,int *maxframes,int *loops,int *framestart,int *DISPDELAY,int *typeanimation,int *background)
{switch(animation){
case(0):*maxframes=9; //ecualizador
        *DISPDELAY=50;
        *framestart=0;
        *loops=5;
        *typeanimation=1;
        break;
case(1):*maxframes=62; //llena-vacia
        *DISPDELAY=0;
        *framestart=9;
        *loops=1;
        *typeanimation=1;
        break;
case(2):*maxframes=6; //ojos
        *DISPDELAY=100;
        *framestart=71;
        *loops=4;
        *typeanimation=1;
        break;
case(3):*maxframes=27; //OK
        *DISPDELAY=50;
        *framestart=77;
        *loops=1;
        *typeanimation=1;
        break;
case(4):*maxframes=2; //corazon
        *DISPDELAY=500;
        *framestart=104;
        *loops=4;
        *typeanimation=1;
        break;
case(5): //technologic
        *DISPDELAY=0;
        *typeanimation=0;
        msg="ZARDS   ZARDS";
        *background=0;
        break;
case(6): //DAFT PUNK
        *DISPDELAY=0;
        *typeanimation=0;
        msg="      DAFT PUNK";
        *background=1;
        break;
case(7): //SLAMFIELD
        *DISPDELAY=0;
        *typeanimation=0;
        msg="      SLAMFIELD";
        *background=1;
        break;  
case(8):*maxframes=61; //kit
        *DISPDELAY=0;
        *framestart=106;
        *loops=3;
        *typeanimation=1;
        break;
     
     
case(9): //|||||
        *DISPDELAY=0;
        *typeanimation=0;
        msg="| | | | | | | | | | | | | | |";
        *background=1;
        break;
case(10): //|||||
        *DISPDELAY=0;
        *typeanimation=0;
        msg="||||||||||||||||||||||||||";
        *background=0;
        break;
case(11):*maxframes=63; //Cardiograma
        *DISPDELAY=0;
        *framestart=167;
        *loops=4;
        *typeanimation=1;
        break;
case(12):*maxframes=5; //Ventanas
        *DISPDELAY=500;
        *framestart=230;
        *loops=1;
        *typeanimation=1;
        break;
case(13):*maxframes=9; //Space
        *DISPDELAY=250;
        *framestart=235;
        *loops=1;
        *typeanimation=1;
        break;    
 case(14): //Fancy Sauce
        *DISPDELAY=0;
        *typeanimation=0;
        msg="THIS IS A GREAT LED MATRIX....FOR ME TO POOP ON!";
        *background=0;
        break;    
case(15): //HAPPY HALLOWEEN
        *DISPDELAY=0;
        *typeanimation=0;
        msg="   HAPPY HALLOWEEN";
        *background=1;
        break;      
  }}
/*
* This works equally well for both 16x24 and 8x32 matrices.
*/
void displayScrollingLine()
{
  // shift the whole screen 6 times, one column at a time;
  if (animCounter >=15){animCounter=0;}else{animCounter++;} //tek. Plays all 14 sequences sequentially instead of random
  
  animationinfo(animCounter,&maxframes,&loops,&framestart,&DISPDELAY,&typeanimation,&background);
  if(typeanimation==1)
  {
  for(n=0;n<loops;n++)
  {
  for(frame=0;frame<maxframes;frame++)
  {

  for (byte row=0; row<32; row++)
  {  byte rowDots = pgm_read_byte_near(&myfont[frame+framestart][row]);
    
    for (byte col=0; col<8; col++)
    {
      if (rowDots & (1<<(7-col)))
        plot(row, col, 1);
      else
        plot(row, col, 0);
    }
  }
  delay(DISPDELAY);

}}}
else
{ while(1)
{
  // shift the whole screen 6 times, one column at a time;   
  for (int x=0; x < 6; x++)   
  {   
    ht1632_putchar(-x, 0, msg[crtPos],background);   
    ht1632_putchar(-x+6,  0, ((crtPos+1 < strlen(msg)) ? msg[crtPos+1] : ' '),background);  
    ht1632_putchar(-x+12, 0, ((crtPos+2 < strlen(msg)) ? msg[crtPos+2] : ' '),background);  
    ht1632_putchar(-x+18, 0, ((crtPos+3 < strlen(msg)) ? msg[crtPos+3] : ' '),background);  
    ht1632_putchar(-x+24, 0, ((crtPos+4 < strlen(msg)) ? msg[crtPos+4] : ' '),background);  
    ht1632_putchar(-x+30, 0, ((crtPos+5 < strlen(msg)) ? msg[crtPos+5] : ' '),background);  
    ht1632_putchar(-x+36, 0, ((crtPos+6 < strlen(msg)) ? msg[crtPos+6] : ' '),background);   
    delay(DISPDELAY);   
  }   
  
  crtPos++;   
  if (crtPos >= strlen(msg))   
  {   
 crtPos = 0; 
  break; 
}   }}
}

/***********************************************************************
 * traditional Arduino sketch functions: setup and loop.
 ***********************************************************************/

void setup ()
{
  ht1632_setup();
  Serial.begin(9600);
  cls();
}

void loop ()
{
  // display line;
  displayScrollingLine();
}

