#include <Adafruit_NeoPixel.h>

#define PIN_RIGHT_EAR 4
#define PIN_LEFT_EAR 5 
uint16_t colourIndex = 0 ;

// Parameter 1 = number of pixels in right_ear
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel right_ear = Adafruit_NeoPixel(24, PIN_RIGHT_EAR, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel left_ear = Adafruit_NeoPixel(24, PIN_LEFT_EAR, NEO_GRB + NEO_KHZ800);

#define N_PIXELS  24  // Number of pixels in strand
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     40  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 40  // Rate of peak falling dot
 
byte
  earPeak      = 0,      // Used for falling dot
  earDotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  earMinLvlAvg = 0,      // For dynamic adjustment of graph low & high
  earMaxLvlAvg = 512;


#include <HT1632.h>

#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Wire.h>

// Microphone connects to Analog Pin 0.  Corresponding ADC channel number
// varies among boards...it's ADC0 on Uno and Mega, ADC7 on Leonardo.
// Other boards may require different settings; refer to datasheet.
#ifdef __AVR_ATmega32U4__
 #define ADC_CHANNEL 7
#else
 #define ADC_CHANNEL 0
#endif

int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter
int16_t micRawSample  = 0;

byte
  peak[8],      // Peak level of each column; used for falling dots
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data
int
  col[8][10],   // Column levels for the prior 10 frames
  minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
  colDiv[8];    // Used when filtering FFT output to 8 columns

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  miThe software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
PROGMEM uint8_t
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  *colData[] = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };


char EQ_BAR [] = {0b1111, 0b1111, 0b1111, 0b1111, 0b1111, 0b1111, 0b1111, 0b1111};
#define EQ_BAR_WIDTH 	 4
#define EQ_BAR_HEIGHT 	 8

int barXPositions [] = {0,4,8,12,16,20,24,28};
int micSwitchPin = 2;  
boolean useMic = false;


/////////
//animation imports and vars

#include <WProgram.h>
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

int DISPDELAYMULTIPLIER = 50;
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


void setup () {
  Serial.begin(9600);
  
  Serial.println("setup");
  sayHi();
  HT1632.begin(8, 7, 6);
  
//  ht1632_setup();
//  Serial.begin(9600);
//  cls();
  
  pinMode(micSwitchPin, INPUT_PULLUP);


//  HT1632.clear();
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 0, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 4, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 8, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 12, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 16, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 20, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 24, 0);
//  HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, 28, 0);
//  HT1632.render();
  
  
  uint8_t i, j, nBins, binNum, *data;

  memset(peak, 0, sizeof(peak));
  memset(col , 0, sizeof(col));

  for(i=0; i<8; i++) {
    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&colData[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    binNum       = pgm_read_byte(&data[1]);
    for(colDiv[i]=0, j=2; j<nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);
  }
  
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
  
  
  memset(vol, 0, sizeof(vol));
  
  right_ear.begin();
  right_ear.show(); // Initialize all pixels to 'off'
  
  left_ear.begin();
  left_ear.show(); // Initialize all pixels to 'off'
  
}

void loop () {
  
  if(digitalRead (micSwitchPin)  == HIGH){
      useMic = true;
   } else {
      useMic = false;
   }
  
   if(useMic){
       
      uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
      uint16_t minLvl, maxLvl;
      int      level, y, sum;
    
      while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish
    
      fft_input(capture, bfly_buff);   // Samples -> complex #s
      samplePos = 0;                   // Reset sample counter
      ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
      fft_execute(bfly_buff);          // Process complex data
      fft_output(bfly_buff, spectrum); // Complex -> spectrum
    
      // Remove noise and apply EQ levels
      for(x=0; x<FFT_N/2; x++) {
        L = pgm_read_byte(&noise[x]);
        spectrum[x] = (spectrum[x] <= L) ? 0 :
          (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
      }
      
      HT1632.clear();
      
      // Downsample spectrum output to 8 columns:
      for(x=0; x<8; x++) {
        data   = (uint8_t *)pgm_read_word(&colData[x]);
        nBins  = pgm_read_byte(&data[0]) + 2;
        binNum = pgm_read_byte(&data[1]);
        for(sum=0, i=2; i<nBins; i++)
          sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
        col[x][colCount] = sum / colDiv[x];                    // Average
        minLvl = maxLvl = col[x][0];
        for(i=1; i<10; i++) { // Get range of prior 10 frames
          if(col[x][i] < minLvl)      minLvl = col[x][i];
          else if(col[x][i] > maxLvl) maxLvl = col[x][i];
        }
        // minLvl and maxLvl indicate the extents of the FFT output, used
        // for vertically scaling the output graph (so it looks interesting
        // regardless of volume level).  If they're too close together though
        // (e.g. at very low volume levels) the graph becomes super coarse
        // and 'jumpy'...so keep some minimum distance between them (this
        // also lets the graph go to zero when no sound is playing):
        if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
        minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
        maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)
    
        // Second fixed-point scale based on dynamic min/max levels:
        level = 10L * (col[x][colCount] - minLvlAvg[x]) /
          (long)(maxLvlAvg[x] - minLvlAvg[x]);
    
        // Clip output and convert to byte:
        if(level < 0L)      c = 0;
        else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
        else                c = (uint8_t)level;
    
        if(c > peak[x]) peak[x] = c; // Keep dot on top
    
        if(peak[x] <= 0) { // Empty column?
          HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, barXPositions[x], 8); //matrix.drawPixel(x, y, LED_RED);
          continue;
        } else if(c < 8) { // Partial column?
           HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, barXPositions[x], 7 - c);
        }
        
        // The 'peak' dot color varies, but doesn't necessarily match
        // the three screen regions...yellow has a little extra influence.
        y = abs(8 - peak[x]);
        HT1632.drawImage(EQ_BAR, EQ_BAR_WIDTH,  EQ_BAR_HEIGHT, barXPositions[x], y);   
      }
    
       HT1632.render();
    
      // Every third frame, make the peak pixels drop by 1:
      if(++dotCount >= 3) {
        dotCount = 0;
        for(x=0; x<8; x++) {
          if(peak[x] > 0) peak[x]--;
        }
      }
    
      if(++colCount >= 10) colCount = 0;
      
      ////////////////
 
      Serial.println(micRawSample);
      

      
        uint16_t earMinLvl, earMaxLvl;
        int      n, height;       
       
        n   = micRawSample;                      // Raw reading from mic 
        
     
        n   = abs(n - 512 - DC_OFFSET); // Center on zero
        n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
        lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
       
        // Calculate bar height based on dynamic min/max levels (fixed point):
        height = TOP * (lvl - earMinLvlAvg) / (long)(earMaxLvlAvg - earMinLvlAvg);
       
        if(height < 0L)       height = 0;      // Clip output
        else if(height > TOP) height = TOP;
        if(height > earPeak)     earPeak   = height; // Keep 'peak' dot at top
       
       
        // Color pixels based on rainbow gradient
        for(i=0; i<N_PIXELS; i++) {
          if(i >= height)             
          {
            right_ear.setPixelColor(i,   0,   0, 0);
            left_ear.setPixelColor(i,   0,   0, 0);
          }
          else {
            right_ear.setPixelColor(i,Wheel(map(i,0,right_ear.numPixels()-1,30,150)));
            left_ear.setPixelColor(i,Wheel(map(i,0,left_ear.numPixels()-1,30,150)));
          }
          
        }
       
       
       
        // Draw peak dot  
        if(earPeak > 0 && earPeak <= N_PIXELS-1) {
          right_ear.setPixelColor(earPeak,Wheel(map(earPeak,0,right_ear.numPixels()-1,30,150)));
          left_ear.setPixelColor(earPeak,Wheel(map(earPeak,0,right_ear.numPixels()-1,30,150)));
        }
      
         right_ear.show(); // Update strip
         left_ear.show(); // Update strip
       
      // Every few frames, make the peak pixel drop by 1:
       
          if(++dotCount >= PEAK_FALL) { //fall rate 
            
            if(earPeak > 0) earPeak--;
            dotCount = 0;
          }
       
       
       
        vol[volCount] = n;                      // Save sample for dynamic leveling
        if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
       
        // Get volume range of prior frames
        minLvl = maxLvl = vol[0];
        for(i=1; i<SAMPLES; i++) {
          if(vol[i] < minLvl)      minLvl = vol[i];
          else if(vol[i] > maxLvl) maxLvl = vol[i];
        }
        // minLvl and maxLvl indicate the volume range over prior frames, used
        // for vertically scaling the output graph (so it looks interesting
        // regardless of volume level).  If they're too close together though
        // (e.g. at very low volume levels) the graph becomes super coarse
        // and 'jumpy'...so keep some minimum distance between them (this
        // also lets the graph go to zero when no sound is playing):
        if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
        earMinLvlAvg = (earMinLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
        earMaxLvlAvg = (earMaxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
        
    } else {
        displayScrollingLine();
    
//        // Some example procedures showing how to display to the pixels:
//        colorWipe(right_ear.Color(255, 0, 0), 50); // Red
//        colorWipe(right_ear.Color(0, 255, 0), 50); // Green
//        colorWipe(right_ear.Color(0, 0, 255), 50); // Blue
//        
//        rainbow(20);
//        rainbowCycle(20);
    }

} 

ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  micRawSample         = ADC; // 0-1023
  
  capture[samplePos] =
    ((micRawSample > (512-noiseThreshold)) &&
     (micRawSample < (512+noiseThreshold))) ? 0 :
    micRawSample - 512; // Sign-convert for FFT; -512 to +511

  if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}


///////////////////////////////////

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
        *DISPDELAY=50*DISPDELAYMULTIPLIER;
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
        *DISPDELAY=100*DISPDELAYMULTIPLIER;
        *framestart=71;
        *loops=4;
        *typeanimation=1;
        break;
case(3):*maxframes=27; //OK
        *DISPDELAY=50*DISPDELAYMULTIPLIER;
        *framestart=77;
        *loops=1;
        *typeanimation=1;
        break;
case(4):*maxframes=2; //corazon
        *DISPDELAY=500*DISPDELAYMULTIPLIER;
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
        *DISPDELAY=500*DISPDELAYMULTIPLIER;
        *framestart=230;
        *loops=1;
        *typeanimation=1;
        break;
case(13):*maxframes=9; //Space
        *DISPDELAY=250*DISPDELAYMULTIPLIER;
        *framestart=235;
        *loops=1;
        *typeanimation=1;
        break;    
 case(14): //Fancy Sauce
        *DISPDELAY=0;
        *typeanimation=0;
        msg="THIS IS A GREAT LED MATRIX....FOR ME TO POOP ON!!";
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
  
  Serial.println(DISPDELAY);  
  
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
  
  rainbowCycleCustom(0);
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
    
    rainbowCycleCustom(0);
    delay(DISPDELAY);   
  }   
  
  crtPos++;   
  if (crtPos >= strlen(msg))   
  {   
 crtPos = 0; 
  break; 
}   }}
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<right_ear.numPixels(); i++) {
      right_ear.setPixelColor(i, c);
      right_ear.show();
      
      left_ear.setPixelColor(i, c);
      left_ear.show();
      
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<right_ear.numPixels(); i++) {
      right_ear.setPixelColor(i, Wheel((i+j) & 255));
      left_ear.setPixelColor(i, Wheel((i+j) & 255));
    }
    right_ear.show();
    left_ear.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< right_ear.numPixels(); i++) {
      right_ear.setPixelColor(i, Wheel(((i * 256 / right_ear.numPixels()) + j) & 255));
      left_ear.setPixelColor(i, Wheel(((i * 256 / right_ear.numPixels()) + j) & 255));
    }
    right_ear.show();
    left_ear.show();
    delay(wait);
  }
}

void rainbowCycleCustom(uint8_t wait) {
  uint16_t i;
  
  colourIndex+=10;
  
  if(colourIndex>256){
    colourIndex=0;
  }

  for(i=0; i< right_ear.numPixels(); i++) {
    right_ear.setPixelColor(i, Wheel(((i * 256 / right_ear.numPixels()) + colourIndex) & 255));
    left_ear.setPixelColor(i, Wheel(((i * 256 / right_ear.numPixels()) + colourIndex) & 255));
  }
  right_ear.show();
  left_ear.show();
  delay(wait);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return right_ear.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   return left_ear.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return right_ear.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return left_ear.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return right_ear.Color(0, WheelPos * 3, 255 - WheelPos * 3);
   return left_ear.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
