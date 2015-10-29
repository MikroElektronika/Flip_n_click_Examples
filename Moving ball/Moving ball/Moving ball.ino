

#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <LedControl.h>
#include <SPI.h>
#include <Wire.h>

#include "flip_click_defs.h"
#define C_MATRIX 0
#define D_MATRIX 0
#define TRUE 1
#define FALSE 0
LedControl D_led = LedControl( D_MOSI, D_SCK, D_CS, D_MATRIX );
LedControl C_led = LedControl( C_MOSI, C_SCK, C_CS, C_MATRIX );
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

int i;
int row;
byte value;
sensors_event_t event; 

// HEADS UP, INCREMENT AND DECREMENT FUNCTIONS FOR MATRIX D TAKE UP NEGATIVE VALUES ONLY!


int incrementD (int row, byte value) {
  if ( row < -7) row=-7;
  row = 8 + row;
  if (row == 7) {
    D_led.setRow(C_MATRIX, 6, 0b00000000);  
    C_led.setRow(C_MATRIX, 0, value);
  }

  else if (row == 1)
  {   D_led.setRow(C_MATRIX,  0, 0b00000000);
      D_led.setRow(C_MATRIX, row+1, value);  
      
  }

  else  { 
    D_led.setRow(C_MATRIX, row-1, 0b00000000);  
    D_led.setRow(C_MATRIX, row+1, value);
  } 
  row = row - 8;
  row++;
  return row;
}


int incrementC (int row, byte value) {
    if (row < 0) incrementD(row, value);
    if (row == 7) {
        // RING BUZZER;
        analogWrite(A_PWM, 100);
        delay(500);
        analogWrite(A_PWM, 0);
        return row;
      }

    if (row == 6) {
        C_led.setRow(C_MATRIX, row-1, 0b00000000);  
        C_led.setRow(C_MATRIX, row+1, value);  
      }

    if (row == 0) {
        D_led.setRow(D_MATRIX, 7, 0b00000000);  
        C_led.setRow(C_MATRIX, 0, value);  
        C_led.setRow(C_MATRIX, 1, value);  
      }

      else {
          C_led.setRow(C_MATRIX, row-1, 0b00000000);  
          C_led.setRow(C_MATRIX, row, value);  
          C_led.setRow(C_MATRIX, row+1, value);  
        }
    row++;
    return row;
}


byte MOVECENTERUP (byte value) {
  if (value == 0b11000000) {
      // RING BUZZER!
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
      return value;
    }

  value = value << 1;
  C_led.setRow(C_MATRIX, 0, value );
  D_led.setRow(C_MATRIX, 7, value );
  
  return value;
  }
  
byte MOVECENTERDOWN (byte value) {
  if (value == 0b00000011) {
      // RING BUZZER!
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
      return value;
    }

  value = value >> 1;
  C_led.setRow(C_MATRIX, 0, value );
  D_led.setRow(C_MATRIX, 7, value );
  
  return value;
}


byte moveCUP (int row, byte value) {
  if (row < 0) moveDUP(row, value);
  
  if (value == 0b11000000) {
    // RING BUZZER!!;
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
    return value;
  }

  else {
  value = value << 1;
  C_led.setRow(C_MATRIX, row, value);
  C_led.setRow(C_MATRIX, row-1, value);
  }
  return value;
}

byte moveCDOWN (int row, byte value) {
  if (row < 0) moveDDOWN (row, value);
  if (value == 0b00000011) {
    // RING BUZZER!!;
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
    return value;
     }
     
  else {
  value = value >> 1; 
  C_led.setRow(C_MATRIX, row, value);
  C_led.setRow(C_MATRIX, row-1, value);
  }
  
  return value;

}


byte moveDUP (int row, byte value) {
  if ( row < -7) row=-7;
  row = 8 + row;
  if (value == 0b11000000) {
    // RING BUZZER!!;
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
    return value;
  }

  value = value << 1;
  D_led.setRow(D_MATRIX, row, value);
  D_led.setRow(D_MATRIX, row-1, value);
  row = 8 - row;
  return value;
}

byte moveDDOWN (int row, byte value) {
  if ( row < -7) row=-7;
  row = 8 + row;
  if (value == 0b00000011) {
    // RING BUZZER!!;
      analogWrite(A_PWM, 100);
      delay(500);
      analogWrite(A_PWM, 0);
    return value;
  }
  value = value >> 1;
  D_led.setRow(D_MATRIX, row, value);
  D_led.setRow(D_MATRIX, row-1, value);
  row = 8 - row;
  return value;
}


int decrementD (int row, byte value) {
  if ( row < -7) row=-7;
  row = 8 + row;
  if ( row == 2) { 
          D_led.setRow(D_MATRIX, row, 0b00000000);  
          D_led.setRow(D_MATRIX, row-1, value);
          D_led.setRow(D_MATRIX, row-2, value);
          }

          
       else if (row < 2) { 
          // SOUND BUZZER
          analogWrite(A_PWM, 100);
          delay(500);
          analogWrite(A_PWM, 0);
          row = row - 8;
          return row;
       }
       
       else {
       D_led.setRow(D_MATRIX, row, 0b00000000);  
       D_led.setRow(D_MATRIX, row-1, value);
       D_led.setRow(D_MATRIX, row-2, value);
       }
       row = row - 8;
  return row -1;     
  
}


int decrementC (int row, byte value) {
  if (row == 1) {
        C_led.setRow(C_MATRIX, row, 0b00000000);  
        C_led.setRow(C_MATRIX, row-1, value);
        D_led.setRow(D_MATRIX, 7,value);
  }

  else if (row == 0) {
      C_led.setRow(C_MATRIX, 0, 0b00000000);  
      D_led.setRow(D_MATRIX, 7,value);
      D_led.setRow(D_MATRIX, 6,value);
    
    }

  else if ( row < 0 ) {
    decrementD(row, value);
  }

  else {
        C_led.setRow(C_MATRIX, row, 0b00000000);  
        C_led.setRow(C_MATRIX, row-1, value);
        C_led.setRow(C_MATRIX, row-2, value);
  }

  return row-1;
}


void updateposition(int16_t x, int16_t y) {
    if (x>0) row = decrementC(row, value);
    else if (x < 0) row = incrementC(row, value);

    if ( y > 0) // move UP;
    {
      
       if ( row == 0) value = MOVECENTERUP(value);
      else value = moveCUP ( row, value);
    
    }
    
    else if ( y < 0) // move DOWN; 
    {
    
      if ( row == 0) value = MOVECENTERDOWN(value);
      else  value = moveCDOWN ( row, value);
    
    }
   
}


void setup() {
  Serial.begin(9600);
  Wire1.begin();
  // put your setup code here, to run once:
  
  C_led.shutdown(0,false);
  
  /* Set the brightness to a medium values */
  
  C_led.setIntensity(0,8);
  
  /* and clear the display */
  
  C_led.clearDisplay(0);
  
  
  for (i=0; i<8; i++) {
    C_led.setLed(C_MATRIX, i, 0, TRUE);
    delay(50);
    }
  
  D_led.shutdown(0,false);
  
  /* Set the brightness to a medium values  */
  
  D_led.setIntensity(0,8);
  
  /* and clear the display */
  
  D_led.clearDisplay(0);
  
  
  for (i=0; i<8; i++) {
    D_led.setLed(D_MATRIX, i, 0, TRUE);
    delay(50);
    }

  for (i=0; i<8; i++) {
    C_led.setLed(C_MATRIX, i, 0, FALSE);
    delay(50);
    }
  for (i=0; i<8; i++) {
    D_led.setLed(D_MATRIX, i, 0, FALSE);
    delay(50);
    }
    
  C_led.setRow(C_MATRIX, 4, 0b00011000);  // set the LEDs vertically
  C_led.setRow(C_MATRIX, 3, 0b00011000);
  delay(500);

      value = 0b00011000;
      row = 4;
     

    /* Initialise the sensor */

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_4_G);

}

int16_t x,y,z;
void loop() {
   accel.getEvent(&event);
   x = map(event.acceleration.x, -8, 8, -4, 4);
   y = map(event.acceleration.y, -8, 8, -4, 4);
   Serial.print("X: "); Serial.print(x); Serial.print("  ");
   Serial.print("Y: "); Serial.print(y); Serial.print("  ");
   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
   delay(50);
   updateposition(x,y);

}
