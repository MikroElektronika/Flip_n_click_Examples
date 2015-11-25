/*******************************************************************************
* Title                 :   Testbench
* Filename              :   TestBench.ino
* Author                :   RL
* Origin Date           :   25/11/2015
* Notes                 :   Running on Flip and Click Due CPU
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  04/10/2015    .1               RL        Module Created.
*  07/10/2015    .2               RL        Added pin definitions
*
*******************************************************************************/
/**
 *  @file TestBench.ino
 *
 *  @brief Demonstrates running 4 click boards and tests features
 *
 *
 *  <b>Main-board:</b>
 *  Flip and Click
 *
 *  <b>Click Boards Used:</b>
 *  <ul>
 *  <li>RTC2 click</li>
 *  <li>Buzz click</li>
 *  <li>Accel Click</li>
 *  <li>Button R Click</li>
 *  </ul>
 */
/******************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <DueTimer.h>
#include <Time.h>
#include <DS1307RTC.h>
#include "flip_click_defs.h"

// Flash Chip
#define FLASH_CS   B_CS
#define FLASH_HD   B_RST
#define FLASH_WP   B_PWM

// Constants
#define SERIAL_FLASH_CMD_RDID  0x9F    
#define SERIAL_FLASH_CMD_READ  0x03
#define SERIAL_FLASH_CMD_WRITE 0x02
#define SERIAL_FLASH_CMD_WREN  0x06
#define SERIAL_FLASH_CMD_RDSR  0x05
#define SERIAL_FLASH_CMD_ERASE 0xC7    
#define SERIAL_FLASH_CMD_EWSR  0x06    
#define SERIAL_FLASH_CMD_WRSR  0x01
#define SERIAL_FLASH_CMD_SER   0xD8    

// Button
#define BTN_PWM    D_PWM
#define BTN_INT    D_INT

// LEDs
byte leds[4] = { LEDA, LEDB, LEDC, LEDD };

// GPS
byte gps_buffer[100];
uint8_t gps_position;

// Thermo 3 Click
#define TMP102_I2C_ADDR   0x48          // TMP102 I2C address

// Time
volatile uint8_t test_count;

// Prototypes
void rotation( void );
void report_time( void );
void print2digits(int number);
void write_number( void );
void read_number( void );
float get_temp( void );
void print_gps( void );

// Implimentation
void setup()
{
  byte id = 0x00;
  Serial.begin( 57600 );  /**< For debugging purposes */
  Serial1.begin( 9600 );
  SPI.begin( FLASH_CS );          /**< Used with  */
  Wire.begin();           /**< Used with RTC */
  Wire1.begin();          /**< Used with Thermo3 */

  for ( int i = 0; i < 4; i++ )
  {
    pinMode( leds[i], OUTPUT );
    digitalWrite( leds[i], LOW );
  }

  // Setup Flash click
  pinMode( FLASH_CS, OUTPUT );  digitalWrite( FLASH_CS, HIGH );
  pinMode( FLASH_HD, OUTPUT );  digitalWrite( FLASH_HD, HIGH );
  pinMode( FLASH_WP, OUTPUT );  digitalWrite( FLASH_WP, HIGH );
  
  // Read manufacturer ID 
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDID, SPI_CONTINUE );
  id = SPI.transfer( FLASH_CS, 0x00 );
  Serial.print( "Manufacturer ID: 0x" );  Serial.println( id, HEX );

  if( id != 0x1C  )
  {    
    test_count = 10;
    return;
  }
  
  // Reset write protect
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_EWSR );
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_EWSR, SPI_CONTINUE );
  SPI.transfer( FLASH_CS, 0 );

  // Erase chip
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_WREN );
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_ERASE );
  do
  {
      SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
      id = SPI.transfer( FLASH_CS, 0x00 );
  } while( id & 0x01 );
  
  // Display status register on Flash click
  SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
  id = SPI.transfer( FLASH_CS, 0x00 );
  Serial.print( "Status Register: b" );  Serial.println( id, BIN );
   
  Timer2.attachInterrupt( print_gps    ).start( 9000000 );   // Every 9s
  Timer3.attachInterrupt( rotation     ).start( 80000 );     // Every 80ms
  Timer4.attachInterrupt( report_time  ).start( 10000000 );  // Every 10s
  Timer5.attachInterrupt( write_number ).start( 5000000 );   // Every 5s
  Timer6.attachInterrupt( pulse        ).start( 100000 );    // Every 100ms
  Timer7.attachInterrupt( read_number  ).start( 7000000 );   // Every 7s
  
}

void loop()
{
   if( Serial1.available() )
   {
      if( gps_position == 99 ) gps_position = 0;
      gps_buffer[gps_position++] = Serial1.read();
   }

   if( test_count > 1 )
   {
        if( test_count == 10 )
        {
          for ( int i = 0; i < 4; i++ )
          {          
              digitalWrite( leds[i], HIGH );
          }

          return;
        }
        
        analogWrite( BTN_PWM, 0 );
        Timer2.detachInterrupt();
        Timer3.detachInterrupt();
        Timer4.detachInterrupt();
        Timer5.detachInterrupt();
        Timer6.detachInterrupt();
        Timer7.detachInterrupt();

        for ( int i = 0; i < 4; i++ )
        {          
          digitalWrite( leds[i], LOW );
        }

        return;
   }
}


void rotation()
{
  static byte myturn;

  for ( int i = 0; i < 4; i++ )
    digitalWrite( leds[i], LOW );

  digitalWrite( leds[myturn++], HIGH );

  if ( myturn > 3 )
    myturn = 0;
}

void report_time()
{
  tmElements_t tm;
  char tmp_txt[100];

  if( RTC.read( tm ) )
  {
    Serial.print("Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  } else {
    test_count = 10;
    return;    
  }

  test_count++;
}

void print2digits(int number) 
{
  if (number >= 0 && number < 10) 
  {
    Serial.write('0');
  }
  Serial.print(number);
}

void write_number()
{
    static byte count;
    uint8_t tmp;
    
    do
    {
        SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
        tmp = SPI.transfer( FLASH_CS, 0x00 );
    } while( tmp & 0x01 );

    SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_WREN );
    SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_SER, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0x00, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff );

    do
    {
        SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
        tmp = SPI.transfer( FLASH_CS, 0x00 );
    } while( tmp & 0x01 );
    
    SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_WREN );
    SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_WRITE, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0x00, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, count );    
    
    do
    {
        SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
        tmp = SPI.transfer( FLASH_CS, 0x00 );
    } while( tmp & 0x01 );

    Serial.print( "Wrote Number:" );
    Serial.println( count++ );
}

void read_number()
{
    byte number;
    byte tmp;
    
    do
    {
        SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_RDSR, SPI_CONTINUE );
        tmp = SPI.transfer( FLASH_CS, 0x00 );
    }
    while( tmp & 0x01 );
    
    SPI.transfer( FLASH_CS, SERIAL_FLASH_CMD_READ, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0x00, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff, SPI_CONTINUE );
    SPI.transfer( FLASH_CS, 0xff, SPI_CONTINUE );
    number = SPI.transfer( FLASH_CS, 0x00 );
  
    Serial.print( "Read Number: " );
    Serial.println( number );
}

void pulse()
{
    static byte pulse;
    static bool dir;

    analogWrite( BTN_PWM, pulse );

    if( pulse == 255 )
        dir = false;
    else if( pulse == 20 )
        dir = true;

    if( dir )
        pulse++;
    else
        pulse--;
}

void print_gps()
{
    char tmp[100];
    gps_buffer[ gps_position + 1 ] = '\0';
    if( gps_buffer[0] == '\0' );
        test_count = 10;
    sprintf( tmp, "GPS: %s", gps_buffer );  
    Serial.println( tmp );
}

void btn_isr()
{
    static unsigned long last_interrupt_time;
    unsigned long interrupt_time = millis();

    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ( interrupt_time - last_interrupt_time > 80 )
    {
         // Todo something    
    }

    last_interrupt_time = interrupt_time;

}
