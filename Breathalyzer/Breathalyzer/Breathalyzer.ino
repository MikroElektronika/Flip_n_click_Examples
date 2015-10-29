#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <DueTimer.h>
#include <LedControl.h>


#include "flip_click_defs.h"
#define AIR_AN      A_AN
#define ALC_AN      B_AN
#define AQ_MATRIX 0
#define ALC_MATRIX 0
#define TRUE 1
#define FALSE 0
LedControl AQ_led = LedControl( D_MOSI, D_SCK, D_CS, AQ_MATRIX );
LedControl ALC_led = LedControl( C_MOSI, C_SCK, C_CS, ALC_MATRIX );

  unsigned int i;   
  uint8_t array[8];
  uint8_t j;
  uint32_t value;
  uint16_t air_quality;
  unsigned int alc_measure;


float calculatePPM( uint16_t value ) {
  const float Rl      = 5000.0;               // Rl (Ohm) - Load resistance
  const float Vadc_5  = 0.0048828125;         // ADC step 5V/1024 4,88mV (10bit ADC)
  const float Vadc_33 = 0.0032226562;         // ADC step 3,3V/1024 3,22mV (10bit ADC)

  float Vrl;                                  // Output voltage
  float Rs;                                   // Rs (Ohm) - Sensor resistance
  //float ppm;                                // ppm
  float ratio;                                // Rs/Rl ratio
  float lgPPM = 0.0f;
    
  Vrl = ( (float)value ) * Vadc_5;            // For 5V Vcc use Vadc_5  and  for 3V Vcc use Vadc_33
  Rs = Rl * (5 - Vrl)/Vrl;                    // Calculate sensor resistance
  ratio = Rs/Rl;                              // Calculate ratio
  lgPPM = ( log10( ratio ) * -2.6f ) + 2.7f;  // Calculate ppm
  
  return pow( 10,lgPPM );                     // Calculate ppm 
}

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  unsigned int i;
  
  AQ_led.shutdown(0,false);
  /* Set the brightness to a medium values */
  AQ_led.setIntensity(0,8);
  /* and clear the display */
  AQ_led.clearDisplay(0);

  for (i=0; i<8; i++) {
    AQ_led.setLed(AQ_MATRIX, i, 0, TRUE);
    delay(550);
    }

  ALC_led.shutdown(0,false);
  /* Set the brightness to a medium values */
  ALC_led.setIntensity(0,8);
  /* and clear the display */
  ALC_led.clearDisplay(0);

  for (i=0; i<8; i++) {
    ALC_led.setLed(ALC_MATRIX, i, 0, TRUE);
    delay(550);
    } 
    ALC_led.setRow(ALC_MATRIX, 0, 0b11111111);
    ALC_led.setColumn(ALC_MATRIX, 7, 0b11111111);
    ALC_led.setRow(ALC_MATRIX, 7, 0b11111111);
    analogReadResolution(10);
    Serial.begin( 9600 );

}

void setdiode ( int column, int value) {
      unsigned int i;
      AQ_led.setRow(AQ_MATRIX, column, 0);
      for (i=0; i<value; i++) { 
        AQ_led.setLed(AQ_MATRIX, column, i, TRUE);
      }
  
  }

void shiftarray (uint8_t *array) {
    
    for ( int i = 7; i > 0; i--) {
        *( array + ( i ) ) = *( array + ( i - 1 ) );
    }
}

void alcohol_alarm () {
  for (i=0; i<3; i++) {
ALC_led.setColumn(ALC_MATRIX, 2, 0b00111100);
     ALC_led.setColumn(ALC_MATRIX, 3, 0b00111100);
     ALC_led.setColumn(ALC_MATRIX, 4, 0b00111100);
     ALC_led.setColumn(ALC_MATRIX, 5, 0b00111100);
     delay(500);    
     ALC_led.setColumn(ALC_MATRIX, 2, 0);
     ALC_led.setColumn(ALC_MATRIX, 3, 0);
     ALC_led.setColumn(ALC_MATRIX, 4, 0);
     ALC_led.setColumn(ALC_MATRIX, 5, 0);
     delay(250);
   }
   
  }

void loop() {
  
  
  float alc_float;

  for (i=0; i<8;i++);
    {
      value = analogRead( AIR_AN );
      Serial.print( "Analog Read: ");
      Serial.println( value );
      value = map( value, 0, 1023, 1, 20 );
      shiftarray( array );
      array[0] = value;
    }
        
    for (i=0; i<8;i++) 
    { j=array[i];
      setdiode (i ,j);
      delay (10);      
      }


  alc_measure = analogRead(ALC_AN);   
  alc_float = calculatePPM( alc_measure );
  value = ( uint32_t )( alc_float );
  if(value > 50000) value = 50000;
  value = map( value, 5000, 50000, 0, 10 );
  if ( value >= 5) alcohol_alarm();

}




