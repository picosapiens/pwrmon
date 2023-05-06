//-----------------------------------------------------------------------------
// pwrmon.ino
//-----------------------------------------------------------------------------
// Copyright 2023 Picosapiens
//
// This file is part of pwrmon.
//
//  pwrmon is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  pwrmon is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Girino.  If not, see <http://www.gnu.org/licenses/>.
//
//-----------------------------------------------------------------------------

#include "SPI.h"
#include "Adafruit_GFX_AS.h" // https://github.com/rogerclarkmelbourne/Arduino_STM32/tree/master/STM32F1/libraries/Adafruit_GFX_AS
#include "Adafruit_ILI9341_STM.h" // https://github.com/rogerclarkmelbourne/Arduino_STM32/tree/master/STM32F1/libraries/Adafruit_ILI9341_STM
#include <arduinoFFT.h>

// Pinout for Maple Mini (// For the Adafruit shield, these are the default)
#define TFT_CS       PA4 //  13 // PB4                  
#define TFT_DC       PA3 // 12 // PA15                
#define TFT_RST      PA2 // 14 // PB3  
#define analogVoltagePin PA0
#define clamp1Pin PA1
#define clamp2Pin PA6

#define SCREENHEIGHT 320
#define SCREENWIDTH 240
#define BARWIDTH 45

#define TRIGGERLEVEL 1024
#define TRIGGERTIMEOUT 10

#define COLOR_BLACK 0x0000       ///<   0,   0,   0
#define COLOR_WHITE 0xFFFF       ///< 255, 255, 255
#define COLOR_BLUE 0xF800        ///< 255,   0,   0
#define COLOR_RED 0x0C7F //0x001F        ///<   0,   0, 255
#define COLOR_GREEN 0x07E0       ///<   0, 255,   0
#define COLOR_DARKGREEN 0x03E0   ///<   0, 125,   0
#define COLOR_YELLOW 0x07FF        ///<   0, 255, 255
#define COLOR_DARKGREY 0x7BEF    ///< 123, 125, 123

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST); // Use hardware SPI

#define ADCBUFFERSIZE 2048 // must be power of two
uint16_t triggerindex;
uint16_t ADCCounter=0;
uint32_t ADCBuffer[ADCBUFFERSIZE]; 
//uint16_t DualBuffer[ADCBUFFERSIZE];
//uint16_t ADCBuffer[ADCBUFFERSIZE]; 
/* = {
   184,  187,  190,  193,  196,  199,  202,  205,  208,  211,  214,  217,  221,  224,  227,  230 ,
   233,  236,  239,  242,  246,  249,  252,  255,  258,  261,  264,  268,  271,  274,  277,  280 ,
   283,  286,  289,  293,  296,  299,  302,  305,  308,  311,  314,  317,  320,  323,  326,  329 ,
   332,  335,  338,  341,  344,  347,  350,  353,  356,  359,  362,  365,  368,  371,  373,  376 ,
   379,  382,  384,  387,  390,  393,  395,  398,  401,  403,  406,  408,  411,  413,  416,  418 ,
   421,  423,  426,  428,  430,  433,  435,  437,  440,  442,  444,  446,  448,  450,  452,  454 ,
   456,  458,  460,  462,  464,  466,  468,  470,  472,  473,  475,  477,  478,  480,  482,  483 ,
   485,  486,  488,  489,  490,  492,  493,  494,  495,  497,  498,  499,  500,  501,  502,  503 ,
   504,  505,  506,  507,  507,  508,  509,  510,  510,  511,  511,  512,  513,  513,  513,  514 ,
   514,  514,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515 ,
   514,  514,  514,  513,  513,  512,  512,  511,  511,  510,  510,  509,  508,  507,  507,  506 ,
   505,  504,  503,  502,  501,  500,  499,  498,  497,  495,  494,  493,  492,  490,  489,  487 ,
   486,  484,  483,  481,  480,  478,  477,  475,  473,  471,  470,  468,  466,  464,  462,  460 ,
   458,  456,  454,  452,  450,  448,  446,  444,  442,  439,  437,  435,  432,  430,  428,  425 ,
   423,  421,  418,  416,  413,  411,  408,  405,  403,  400,  398,  395,  392,  390,  387,  384 ,
   381,  379,  376,  373,  370,  367,  365,  362,  359,  356,  353,  350,  347,  344,  341,  338 ,
   335,  332,  329,  326,  323,  320,  317,  314,  311,  308,  305,  302,  298,  295,  292,  289 ,
   286,  283,  280,  277,  273,  270,  267,  264,  261,  258,  255,  251,  248,  245,  242,  239 ,
   236,  233,  230,  226,  223,  220,  217,  214,  211,  208,  205,  202,  199,  196,  193,  190 ,
   187,  184,  181,  178,  175,  172,  169,  166,  163,  160,  157,  154,  151,  148,  146,  143 ,
   140,  137,  135,  132,  129,  126,  124,  121,  118,  116,  113,  111,  108,  106,  103,  101 ,
    98,   96,   93,   91,   89,   86,   84,   82,   79,   77,   75,   73,   71,   69,   67,   65 ,
    63,   61,   59,   57,   55,   53,   51,   49,   47,   46,   44,   42,   41,   39,   37,   36 ,
    34,   33,   31,   30,   29,   27,   26,   25,   24,   22,   21,   20,   19,   18,   17,   16 ,
    15,   14,   13,   12,   12,   11,   10,    9,    9,    8,    8,    7,    6,    6,    6,    5 ,
     5,    5,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4 ,
     5,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   11,   12,   12,   13 ,
    14,   15,   16,   17,   18,   19,   20,   21,   22,   24,   25,   26,   27,   29,   30,   32 ,
    33,   35,   36,   38,   39,   41,   42,   44,   46,   48,   49,   51,   53,   55,   57,   59 ,
    61,   63,   65,   67,   69,   71,   73,   75,   77,   80,   82,   84,   87,   89,   91,   94 ,
    96,   98,  101,  103,  106,  108,  111,  114,  116,  119,  121,  124,  127,  129,  132,  135 ,
   138,  140,  143,  146,  149,  152,  154,  157,  160,  163,  166,  169,  172,  175,  178,  181 ,
   184,  187,  190,  193,  196,  199,  202,  205,  208,  211,  214,  217,  221,  224,  227,  230 ,
   233,  236,  239,  242,  246,  249,  252,  255,  258,  261,  264,  268,  271,  274,  277,  280 ,
   283,  286,  289,  293,  296,  299,  302,  305,  308,  311,  314,  317,  320,  323,  326,  329 ,
   332,  335,  338,  341,  344,  347,  350,  353,  356,  359,  362,  365,  368,  371,  373,  376 ,
   379,  382,  384,  387,  390,  393,  395,  398,  401,  403,  406,  408,  411,  413,  416,  418 ,
   421,  423,  426,  428,  430,  433,  435,  437,  440,  442,  444,  446,  448,  450,  452,  454 ,
   456,  458,  460,  462,  464,  466,  468,  470,  472,  473,  475,  477,  478,  480,  482,  483 ,
   485,  486,  488,  489,  490,  492,  493,  494,  495,  497,  498,  499,  500,  501,  502,  503 ,
   504,  505,  506,  507,  507,  508,  509,  510,  510,  511,  511,  512,  513,  513,  513,  514 ,
   514,  514,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515,  515 ,
   514,  514,  514,  513,  513,  512,  512,  511,  511,  510,  510,  509,  508,  507,  507,  506 ,
   505,  504,  503,  502,  501,  500,  499,  498,  497,  495,  494,  493,  492,  490,  489,  487 ,
   486,  484,  483,  481,  480,  478,  477,  475,  473,  471,  470,  468,  466,  464,  462,  460 ,
   458,  456,  454,  452,  450,  448,  446,  444,  442,  439,  437,  435,  432,  430,  428,  425 ,
   423,  421,  418,  416,  413,  411,  408,  405,  403,  400,  398,  395,  392,  390,  387,  384 ,
   381,  379,  376,  373,  370,  367,  365,  362,  359,  356,  353,  350,  347,  344,  341,  338 ,
   335,  332,  329,  326,  323,  320,  317,  314,  311,  308,  305,  302,  298,  295,  292,  289 ,
   286,  283,  280,  277,  273,  270,  267,  264,  261,  258,  255,  251,  248,  245,  242,  239 ,
   236,  233,  230,  226,  223,  220,  217,  214,  211,  208,  205,  202,  199,  196,  193,  190 ,
   187,  184,  181,  178,  175,  172,  169,  166,  163,  160,  157,  154,  151,  148,  146,  143 ,
   140,  137,  135,  132,  129,  126,  124,  121,  118,  116,  113,  111,  108,  106,  103,  101 ,
    98,   96,   93,   91,   89,   86,   84,   82,   79,   77,   75,   73,   71,   69,   67,   65 ,
    63,   61,   59,   57,   55,   53,   51,   49,   47,   46,   44,   42,   41,   39,   37,   36 ,
    34,   33,   31,   30,   29,   27,   26,   25,   24,   22,   21,   20,   19,   18,   17,   16 ,
    15,   14,   13,   12,   12,   11,   10,    9,    9,    8,    8,    7,    6,    6,    6,    5 ,
     5,    5,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4,    4 ,
     5,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   11,   12,   12,   13 ,
    14,   15,   16,   17,   18,   19,   20,   21,   22,   24,   25,   26,   27,   29,   30,   32 ,
    33,   35,   36,   38,   39,   41,   42,   44,   46,   48,   49,   51,   53,   55,   57,   59 ,
    61,   63,   65,   67,   69,   71,   73,   75,   77,   80,   82,   84,   87,   89,   91,   94 ,
    96,   98,  101,  103,  106,  108,  111,  114,  116,  119,  121,  124,  127,  129,  132,  135 ,
   138,  140,  143,  146,  149,  152,  154,  157,  160,  163,  166,  169,  172,  175,  178,  181 
};*/
float uspersample = 13.5; // nominal time step
float uVpercount = 60000;

float freq = 0;
float thd = 0;
float Vrms = 0;
float Amps1 = 0;
float Amps2 = 0;
int Wm = 0; // Watt minutes
int Watts = 0;
int lastmin = 0;
float Asum = 0;
int Adiv = 0;

int sum3(int i) // to filter out noise
{
  return (int16_t)ADCBuffer[(ADCCounter+i+ADCBUFFERSIZE-2)%ADCBUFFERSIZE]+(int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE]+(int16_t)ADCBuffer[(ADCCounter+i+2)%ADCBUFFERSIZE];
}

void plot_waveform() // plot waveform and calculate THD
{
  #define ZEROVOLTS 2048 // ADC counts corresponding to zero volts
  #define SHIFTDISTANCE 5
  uint16_t datamax = 0;
  uint16_t datamin = 4095;
  uint16_t crossings[2] = {0, ADCBUFFERSIZE-1};
  uint16_t foundpcrossings = 0;
  
  // Identify two consecutive positive crossings of zero volts
  for( int i=2; i<ADCBUFFERSIZE-1; i++)
  {
    if( (int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE] > datamax )
      datamax = (int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE];
    if( (int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE] < datamin )
      datamin = (int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE];
    if ((sum3(i) <= 3*ZEROVOLTS) && (sum3(i + 1) > 3*ZEROVOLTS))
      crossings[foundpcrossings++] = i;
    if ( 2 == foundpcrossings )
      break;
  }

  //tft.drawFastVLine(120,220,99,COLOR_RED); // col, y, h, color -- seems to freeze up the code??
  //Serial.print("Found "); Serial.print(foundpcrossings); Serial.print(" positive crossings.\n");

  
  // Draw plot
  int x0 = BARWIDTH+1;
  int x1;
  int col = BARWIDTH+1;
  
  int lastvalue = (ADCBuffer[ADCCounter]) >> SHIFTDISTANCE;
  int value = lastvalue;
  
  //tft.drawFastVLine(120,220,100,COLOR_YELLOW); // col, y, h, color
  //tft.fillRect(col,220, 1, 100, COLOR_BLACK);
  tft.fillRect(col,220, 240-2*BARWIDTH-1, 100, COLOR_BLACK);

  bool once = true;
  uint32_t voltsrmsac = 0;
  uint32_t amps1rms = 0;
  uint32_t amps2rms = 0;
  for( int i=1; i<crossings[1]-crossings[0]; i++)
  {
    // Find the first index in the next column
    x1 = BARWIDTH + (240-2*BARWIDTH-1)*i/(crossings[1]-crossings[0]); //map( i, 0, crossings[1]-crossings[0], BARWIDTH+1, 240-BARWIDTH-1 );
   /* if( x1 > col )
    {
      value = ADCBuffer[(ADCCounter+crossings[0]+i)%ADCBUFFERSIZE] >> SHIFTDISTANCE;
      //tft.drawFastVLine(col+1,220,100,COLOR_BLUE); // col, y, h, color
      //tft.fillRect(col,220, x1-x0+1, 100, COLOR_BLACK);
      tft.drawLine(x0, SCREENHEIGHT-1-lastvalue, x1, SCREENHEIGHT-1-value, COLOR_GREEN);
      col++;
      x0 = x1;
      lastvalue = value;
    }*/
     value = ((int16_t)ADCBuffer[(ADCCounter+crossings[0]+i)%ADCBUFFERSIZE]) >> SHIFTDISTANCE;
     if(once)
     {
       x0 = x1;
       once = false;
       lastvalue = value;
     }
     tft.drawLine(x0, SCREENHEIGHT-1-lastvalue, x1, SCREENHEIGHT-1-value, COLOR_GREEN);
     x0 = x1;
     lastvalue = value;
     voltsrmsac += value;
     
  }
  uint32_t voltsmean = voltsrmsac/(crossings[1]-crossings[0]+1);
  uint32_t amps1mean = 0;
  uint32_t amps2mean = 0;
  for(int i = 0; i<=crossings[1]-crossings[0]; i++)
  {
    if( i%2 )
      amps2rms += (int16_t)ADCBuffer[(ADCCounter+i+1)%ADCBUFFERSIZE];
    else
      amps1rms += (int16_t)ADCBuffer[(ADCCounter+i+1)%ADCBUFFERSIZE];
  }
  amps1mean = (amps1mean/((crossings[1]-crossings[0]+1)/2));
  amps2mean = (amps2mean/((crossings[1]-crossings[0]+1)/2));
  voltsrmsac = 0;
  amps1rms = 0;
  amps2rms = 0;
  for(int i = 0; i<=crossings[1]-crossings[0]; i++)
  {
    voltsrmsac += sq((int16_t)ADCBuffer[(ADCCounter+i)%ADCBUFFERSIZE]-voltsmean);
    if( i%2 )
      amps2rms += sq((int16_t)ADCBuffer[(ADCCounter+i+1)%ADCBUFFERSIZE]-amps2mean);
    else
      amps1rms += sq((int16_t)ADCBuffer[(ADCCounter+i+1)%ADCBUFFERSIZE]-amps1mean);
  }
  voltsrmsac = sqrt(voltsrmsac/(crossings[1]-crossings[0]+1));
  amps1rms = sqrt(amps1rms/(crossings[1]-crossings[0]+1)); // counts
  Amps1 = 50.0/3.3*amps1rms/1024; // amps
  amps2rms = sqrt(amps2rms/(crossings[1]-crossings[0]+1)); // counts
  Amps2 = 50.0/3.3*amps2rms/1024; // amps
  Vrms = (voltsrmsac*uVpercount)/1000000;

  freq = 1.0173e6/((crossings[1]-crossings[0])*uspersample); // 1.0173 is experimentally determined correction factor for my clock
  

  // Calculate THD

  #define FFTSIZE 128
  double data[FFTSIZE], im[FFTSIZE];
  arduinoFFT FFT = arduinoFFT();
  
  int d = crossings[1] - crossings[0];
  float fsam = 1000000.0/((float)d*uspersample/FFTSIZE);
  int n;
  for(int i=0;i<FFTSIZE;i++)
  {
       n=0;
       data[i] = (float)(ADCBuffer[(ADCCounter+crossings[0]+map(i,0,FFTSIZE-1,0,d))%ADCBUFFERSIZE]);
       im[i] = 0;
  }
  FFT.DCRemoval(data, FFTSIZE);
  //FFT.Windowing(data, FFTSIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Not using window because we've trimmed the data to one cycle
  FFT.Compute(data, im, FFTSIZE, FFT_FORWARD);
  //FFT.ComplexToMagnitude(data, im, FFTSIZE);

  for(int i=0;i<FFTSIZE/2;i++)
  {
    data[i] = data[i]*data[i]+im[i]*im[i];
  }

  double pwrsum = 0;
  for(int i=2; i<FFTSIZE/2; i++) // Add up the power beyond the fundamental at index 1
  {
    pwrsum += data[i];
  }

  thd = sqrt( pwrsum / data[1]);

  return;
}


void capture_slow()
{
  ADCCounter = 0;

  // Configure ADC speed for the slowest possible continuous conversion
  adc_set_prescaler(ADC_PRE_PCLK2_DIV_8);  // 9 MHz ADC Clock
  adc_set_sample_rate(ADC1, ADC_SMPR_239_5); // Sample for 239.5 ADC clock cycles (252 total clocks for sample + conversion)
  adc_set_sample_rate(ADC2, ADC_SMPR_239_5);
  uspersample = 252/(72/8)+0.5; // Approx 35.7 kSps, which means about 595 samples per 60Hz cycle

  // Set up ADC
  ADC1->regs->CR1 |= 6 << 16; // Regular simultaneous mode. Required for ADC1 only. ADC2 will follow.
  ADC1->regs->SQR3 = PIN_MAP[analogVoltagePin].adc_channel; // ADC1 will read voltage only
  adc_set_reg_seqlen(ADC2, 2); // ADC2 will scan both current clamps
  ADC2->regs->SQR3 = (  PIN_MAP[clamp1Pin].adc_channel | ( PIN_MAP[clamp2Pin].adc_channel << 5 ) );
  ADC1->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode
  ADC2->regs->CR2 |= ADC_CR2_CONT;
  ADC2->regs->CR1 |= ADC_CR1_SCAN;
  ADC1->regs->CR2 |= ADC_CR2_DMA;     //Needs to be in DMA mode for dual mode to work
  ADC2->regs->CR2 |= ADC_CR2_DMA;     //Needs to be in DMA mode for scan mode to work
  ADC1->regs->CR1 |= 6 << 16; // Regular simultaneous mode. Required for ADC1 only. ADC2 will follow.

  bool lookfor;
  uint8_t trigger;
  uint32_t stopIndex;
  trigger = 5;
  stopIndex = ADCBUFFERSIZE+1; // will never reach this, but will update when triggered
  
  lookfor = false; // rising edge trigger

  uint32_t buffercycles = 0;
  bool currentstate;

  // Start conversion
  ADCCounter = 0;
  nvic_globalirq_disable();
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  ADC2->regs->CR2 |= ADC_CR2_SWSTART;
  
  while(true)
  {
    while (!(ADC1->regs->SR & ADC_SR_EOC)) // Wait for next sample
          ;
    ADCBuffer[ADCCounter] = ADC1->regs->DR; //(ADC1->regs->DR | (ADC2->regs->DR << 16)); // & ADC_DR_DATA; // Store sample; Reading ADC_DR clears the ADC_SR EOC bit. In simultaneous mode so this contains both ADC1 and ADC2 results
    currentstate = ((int16_t)ADCBuffer[ADCCounter<<1]>=TRIGGERLEVEL);   // We trigger based on adc1
    //DualBuffer[ADCCounter] = (ADC2->regs->DR) & ADC_DR_DATA; // Could check SR but shouldn't have to because we started the two ADC's at the same time
    ADCCounter = (ADCCounter+1) & (ADCBUFFERSIZE-1);
    
    switch(trigger)
    {
      case 5: // trigger-disabled period ( to make sure we fill at least ADCBUFFERSIZE-waitDuration before trigger )
        if( ADCCounter > 256 )
          trigger--;
        break;
      case 4: // waiting
        if(currentstate == lookfor)
        {
          lookfor = !lookfor;
          trigger--;
        }
        break;
      case 3: // armed
        if(currentstate == lookfor)
        {
          trigger--;
          triggerindex = ADCCounter;
        }
        break;
      case 2: // triggered
        stopIndex = ( triggerindex + ADCBUFFERSIZE - 16 ) & (ADCBUFFERSIZE);
        trigger--;
        break;
      case 1:
          if( ADCCounter == stopIndex )
          {
            // Take the ADC out of continuous conversion mode
            ADC1->regs->CR2 &= ~ADC_CR2_CONT;
            buffercycles = TRIGGERTIMEOUT;
            trigger--;
          }
        break;
    }
    
    if(0==ADCCounter)
      buffercycles++;
    if(buffercycles >= TRIGGERTIMEOUT)
      break;
  }
  nvic_globalirq_enable();
  ADCCounter = 2*ADCCounter; // This function was looking at 32 bit elements whereas the plot function uses 16 bit
  triggerindex = 2*triggerindex;
}



void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  
  tft.begin();
  tft.fillScreen(COLOR_BLACK);
  tft.setRotation(1);

}


void loop(void) {
  capture_slow();
  
  int msec = millis();
  int t_s = msec/1000;
  int t_m = t_s/60;
  int t_h = t_m/60;
  t_m -= t_h*60;

  if(t_m > lastmin)
  {
    // Integrate energy consumption
    Wm += Vrms*Asum/Adiv;
    Asum = 0;
    Adiv = 0;
    
    lastmin = t_m;
  }

  // Simulated data
  /*
  float tmodel = t_s + 0.001*(msec%1000);
  Vrms = 240 + 7*sin(tmodel/3) + 3*sin(tmodel/30);
  Amps1 = 19 + 8*sin(tmodel/4) + 9*sin(tmodel/37);
  if( 0 > Amps1 )
    Amps1 = 0;
  Amps2 = 17 + 9*sin(tmodel/5) + 10*sin(tmodel/40);
  if( 0 > Amps2 )
    Amps2 = 0;
  */

  // Store power consumption data
  Asum += Amps1;
  Asum += Amps2;
  Adiv += 2; // Two because I need to divide by double the sample count because 

  Watts = (int)(Vrms*(Amps1 + Amps2)/2.0);
  
  
  // Print Watts
  tft.fillRect(BARWIDTH+1,5, 150, 35, COLOR_BLACK);
  tft.setTextSize(5);
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(46, 5);
  if( 1000 > Watts)
    tft.print(' ');
  tft.print(Watts);
  tft.setTextSize(3);
  tft.print('V');
  tft.setCursor(SCREENWIDTH-BARWIDTH-15,17);
  tft.print('A');

  // Print runtime and energy
  tft.fillRect(BARWIDTH+1,50, 150, 45, COLOR_BLACK);
  tft.setCursor(BARWIDTH+15,50);
  tft.setTextSize(3);
  tft.print(t_h); tft.print("h "); tft.print(t_m); tft.print('m');
  tft.setCursor(BARWIDTH+1,73);
  int Wh = Wm/60;
  if( 10000 > Wh )
    tft.print(' ');
  tft.print(Wh/1000); tft.print('.'); tft.print((Wh%1000)/100); tft.print("kWh");

  // Print current
  int h = Amps1*320/40;
  int color = COLOR_WHITE;
  if( Amps1 > 27.5 )
    color = COLOR_RED;
  else if (Amps1 > 20)
    color = COLOR_YELLOW;
  tft.fillRect(0,0, BARWIDTH, SCREENHEIGHT-h, COLOR_BLACK);
  tft.fillRect(0,SCREENHEIGHT-h, BARWIDTH, h, color);
  tft.drawFastHLine(0,SCREENHEIGHT*10/40,BARWIDTH,COLOR_DARKGREY);
  color = COLOR_WHITE;
  if( Amps2 > 27.5 )
    color = COLOR_RED;
  else if (Amps2 > 20)
    color = COLOR_YELLOW;
  h = Amps2*320/40;
  tft.fillRect(SCREENWIDTH-BARWIDTH, 0, BARWIDTH, SCREENHEIGHT-h, COLOR_BLACK);
  tft.fillRect(SCREENWIDTH-BARWIDTH, SCREENHEIGHT-h, BARWIDTH, h, color);
  tft.drawFastHLine(SCREENWIDTH-BARWIDTH,SCREENHEIGHT*10/40,BARWIDTH,COLOR_DARKGREY);
  tft.setTextColor(COLOR_BLUE);
  tft.setTextSize(3);
  tft.setCursor(2, SCREENHEIGHT-22);
  tft.print((int)Amps1);
  tft.setCursor(SCREENWIDTH + 2 - BARWIDTH, SCREENHEIGHT-22 );
  tft.print((int)Amps2);
  
  plot_waveform(); // Call this before printing statistics because it calculates the following statistics
  tft.setTextColor(COLOR_WHITE);
  tft.fillRect(BARWIDTH+1,105, SCREENWIDTH-2*BARWIDTH-1, 56, COLOR_BLACK);
  tft.setCursor(BARWIDTH+2,105);
  tft.print((int)(Vrms+0.5)); tft.print("Vrms");
  tft.setCursor(BARWIDTH+2,130);
  tft.print(freq); tft.print("Hz");
  tft.setCursor(BARWIDTH+2,155);
  tft.print((int)(100*thd+0.5)); tft.print("%THD");

  delay(100);
}
