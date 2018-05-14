/*******************************************************************************
* Title                 :   Fartalyzer Demo
* Filename              :   Fartalyzer.ino
* Author                :   RL
* Origin Date           :   04/10/2015
* Notes                 :   Running on Flip and Click Due
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  04/10/2015    .1               RL        Module Created.
*
*******************************************************************************/
/**
 *  @file Fartalyzer.ino
 *  @brief Demonstrates running 4 click boards
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <DueTimer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <DS1307RTC.h>
#include <Time.h>
#include <math.h>
#include "flip_click_defs.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
// Thermo 3 Click
#define TMP102_I2C_ADDR   0x48          // TMP102 I2C address

// mikroSD Click
#define SD_CS D_CS
#define SD_CD D_AN

// Methane Click
#define METH_AN B_AN

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
// Global Variables
byte leds[] = { LEDA, LEDB, LEDC, LEDD };
volatile bool wakeup_flag;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
double calculatePPM( void );
float get_temp( void );
void log_some_gas( void );

// Prototype ISRs
void wakeup_isr( void );

/******************************************************************************
* Function Definitions
*******************************************************************************/
void setup() 
{
  Serial.begin( 57600 );
  SPI.setClockDivider( SPI_CLOCK_DIV64 );
  SPI.begin();
  Wire1.begin();
  Wire.begin();

  pinMode( SD_CS, OUTPUT );
  digitalWrite( SD_CS, HIGH );
  pinMode( SD_CD, INPUT );

  for ( int i = 0; i < 4; i++ )
  {
    pinMode( leds[i], OUTPUT );
    digitalWrite( leds[i], LOW );
  }

  analogReadResolution(10);

  if ( digitalRead( SD_CD ) == LOW ) // see if the card is present
  {
    if ( !SD.begin( SD_CS ) )
      Serial.println(" Card failed, or not present" );
    else
      Serial.println( "Card found" );
  }
    
  Timer4.attachInterrupt( wakeup_isr ).start( 20000000 );
}


void loop() 
{
  float current_ppm = calculatePPM();

  current_ppm = map( current_ppm, 3, 15, 0, 4 );

  for ( int i = 0; i < 4; i++ )
     digitalWrite( leds[i], LOW );

  for ( int i = 0; i < current_ppm; i++ )
     digitalWrite( leds[i], HIGH );
  
  if( wakeup_flag || current_ppm > 3 )
      log_some_gas();
}


//Calculation of PPM
double calculatePPM()
{
  double lgPPM;
  double ppm;                     // ppm
  double Vrl;
  double Rs;                      // Rs (Ohm) - Sensor resistance
  double ratio;                   // Rs/Rl ratio
#define Rl  5000.0                // Rl (Ohm) - Load resistance
#define Vadc_5  0.0048828125      // ADC step 5V/1024 4,88mV (10bit ADC)
#define Vadc_33 0.0032226562      // ADC step 3,3V/1024 3,22mV (10bit ADC)

  // Output voltage
  Vrl = (double)analogRead( METH_AN ) * Vadc_5; // For 3.3V Vcc use Vadc_33
  Rs = Rl * (5 - Vrl) / Vrl;                 // Calculate sensor resistance
  ratio = Rs / Rl;                           // Calculate ratio
  lgPPM = (log10(ratio) * -0.9) + 1.1;       // Calculate ppm
  ppm = pow(10, lgPPM);                      // Calculate ppm

  return ppm;
}

// Get Temperature
float get_temp()
{
  int temp;
  float temperature;

  Wire1.beginTransmission( TMP102_I2C_ADDR ); // Issue I2C start signal
  Wire1.write( 0 );
  Wire1.endTransmission();
  Wire1.requestFrom( TMP102_I2C_ADDR, 2 );
  //wait for response
  while ( Wire1.available() == 0 );

  temp = Wire1.read() << 4;
  temp |= Wire1.read() >> 4;

  Wire1.endTransmission();

  if ( temp & ( 1 << 11) )               // Test negative bit
    temp |= 0xF800;                      // Set bits 11 to 15 to logic 1 to get this reading into real two complement

  temperature = (float)temp * 0.0625;    // Multiply temperature value with 0.0625 (value per bit)

  return temperature;                    // Return temperature data
}

// Logger
void log_some_gas()
{
  File dataFile;

  dataFile = SD.open( "gasser.txt", FILE_WRITE );

  if( dataFile )
  {
     char tmp_txt[100];
     float current_ppm = calculatePPM();
     tmElements_t current_time;
        
     RTC.read( current_time );
     sprintf( tmp_txt, "Day %d - Hour %d: PPM %2.2f, Temp %3.2fC\n", 
              current_time.Day, current_time.Hour, current_ppm, get_temp() );

     dataFile.write( tmp_txt );
     dataFile.flush();
     Serial.print( tmp_txt );
     
     dataFile.close();
  }
    
  wakeup_flag = false;
}

void wakeup_isr()
{
  wakeup_flag = true;
}

