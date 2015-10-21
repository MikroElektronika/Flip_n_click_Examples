/*******************************************************************************
* Title                 :   Magic 8 Ball Demo
* Filename              :   Magic8Ball.ino
* Author                :   RL
* Origin Date           :   04/10/2015
* Notes                 :   Running on Flip and Click Due CPU
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  04/10/2015    .1               RL        Module Created.
*  07/10/2015    .2               RL        Added resources.h and graphics
*
*******************************************************************************/
/**
 *  @file Magic8Ball.ino
 *
 *  @brief Demonstrates running 4 click boards
 *
 *  A classical spin on the <i>"Magic 8 Ball"</i>.  Demonstrates an efficient use of
 *  processing power as well as ulilizing 4 click sockets.
 *
 *  <b>Main-board:</b>
 *  Flip and Click
 *
 *  <b>Click Boards Used:</b>
 *  <ul>
 *  <li>OLED W Click</li>
 *  <li>Buzz Click</li>
 *  <li>Accel Click</li>
 *  <li>Button R Click</li>
 *  </ul>
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <DueTimer.h>
#include <U8glib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include "flip_click_defs.h"
#include "resources.h"

/******************************************************************************
 * Module Preprocessor Constants
 *****************************************************************************/
// OLED
#define SSD_WIDTH  96
#define SSD_HEIGHT 40
#define SSD_TEXT_SIZE 1
#define SSD_CS     A_CS
#define SSD_DC     A_PWM
#define SSD_RESET  A_RST


// Button
#define BTN_PWM    C_PWM
#define BTN_INT    C_INT


// Accel
#define ACCEL_INT  D_INT
#define ACCEL_SCL  D_I2C1_SCL
#define ACCEL_SDA  D_I2C1_SDA

// Buzzer
#define BUZZER_PWM B_PWM

#define TIMEOUT 10000000
/******************************************************************************
 * Module Preprocessor Macros
 *****************************************************************************/

/******************************************************************************
 * Module Typedefs
 *****************************************************************************/
enum
{
    WAITING = 0,
    INSTRUCTION,
    QUESTION,
    READ_ACCEL,
    ANSWERING
};

/******************************************************************************
 * Module Variable Definitions
 *****************************************************************************/
volatile byte state;
volatile bool display_entered = false;

const char responses[][21] =
{
    "Try Again",
    "Signs Say Yes",
    "Signs Say No",
    "Maybe",
    "For Sure",
    "No Way!",
    "You're Kidding",
    "Ask a Nerd",
    "Ask your Mother",
    "Absolutely Not"
};


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified( 1212 );
U8GLIB_SSD1306_96X40 u8g( SSD_CS, SSD_DC ); /**< Display object */
byte leds[4] = { LEDA, LEDB, LEDC, LEDD };

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void rotation( void );
void pulse( void );
void display_waiting( void );
void display_instruction( void );
void display_question( void );
void display_read_accel( void );
void display_answering( void );
void reset_system( void );

/******************************************************************************
 * Function Definitions
 *****************************************************************************/
void setup()
{
    Serial.begin( 57600 );  /**< For debugging purposes */
    SPI.begin();            /**< Used with OLED, Rotary, Thermo */
    Wire.begin();           /**< Used with RTC */

    for( int i = 0; i < 4; i++ )
    {
        pinMode( leds[i], OUTPUT );
        digitalWrite( leds[i], LOW );
    }

    pinMode( BTN_PWM, OUTPUT );
    pinMode( BTN_INT, INPUT_PULLUP );

    analogWrite( BTN_PWM, 150 );
    attachInterrupt( digitalPinToInterrupt( BTN_INT ), btn_isr, FALLING );

    /* Initialise the sensor */
    if( !accel.begin() )
    {
        while( 1 );
    }

    Serial.println( "Accelerometer is Running" );
    accel.setRange( ADXL345_RANGE_4_G );

    state = WAITING;
    display_entered = false;
}

void loop()
{

    switch( state )
    {
        case WAITING:
            if( !display_entered )
                display_waiting();
            break;
        case INSTRUCTION:
            if( !display_entered )
                display_instruction();
            break;
        case QUESTION:
            if( !display_entered )
                display_question();
            break;
        case READ_ACCEL:
            if( !display_entered )
                display_read_accel();

            sensors_event_t event;
            accel.getEvent( &event );

            if( event.acceleration.x > 30 || event.acceleration.x < -30 ||
                    event.acceleration.y > 30 || event.acceleration.y < -30 ||
                    event.acceleration.z > 30 || event.acceleration.z < -30 )
            {
                analogWrite( BUZZER_PWM, 100 );
                delay( 1000 );
                analogWrite( BUZZER_PWM, 255 );
                display_entered = false;
                state = ANSWERING;
            }
            break;
        case ANSWERING:
            if( !display_entered )
                display_answering();
            break;
    }
}

/************************************
         Private Functions
 ***********************************/
void rotation()
{
    static byte myturn;

    for( int i = 0; i < 4; i++ )
        digitalWrite( leds[i], LOW );

    digitalWrite( leds[myturn++], HIGH );

    if( myturn > 3 )
        myturn = 0;
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

void display_waiting()
{

    reset_system();
    display_entered = true;

    u8g.firstPage();
    do
    {
        u8g.drawXBMP( 1, 0, magic8_width, magic8_height, magic8_bits );
        u8g.setFont( u8g_font_courR10 );
        u8g.drawStr( 42, 12, "Press" );
        u8g.drawStr( 50, 24, "RED" );
        u8g.drawStr( 42, 36, "Button" );
    }
    while( u8g.nextPage() );


    Timer3.attachInterrupt( rotation ).start( 80000 ); // Every 80ms
    Timer4.attachInterrupt( pulse ).start( 2000 ); // Every 20ms
}

void display_instruction()
{
    display_entered = true;
    Timer3.detachInterrupt();
    Timer4.detachInterrupt();

    for( int i = 0; i < 4; i++ )
        digitalWrite( leds[i], LOW );

    analogWrite( BTN_PWM, 255 );

    u8g.firstPage();

    do
    {
        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 1, 10, "This is the magical" );
        u8g.drawStr( 18, 20, "Magic 8 Ball" );
        u8g.drawStr( 1, 30, "Click the BTN to" );
        u8g.drawStr( 1, 40, "ask your question" );
    }
    while( u8g.nextPage() );

    Timer5.attachInterrupt( reset_system ).start( TIMEOUT );
}

void display_question()
{
    display_entered = true;
    Timer5.detachInterrupt();
    
    for( int i = 0; i < 4; i++ )
        digitalWrite( leds[i], HIGH );

    u8g.firstPage();
    do
    {
        u8g.drawXBMP( 0, 0, question_width, question_height, question_bits );

        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 39, 10, "ASK ME" );
        u8g.drawStr( 40, 23, "Then Press" );
        u8g.drawStr( 39, 36, "The Button" );
    }
    while( u8g.nextPage() );

    analogWrite( BTN_PWM, 0 );
    
    Timer3.attachInterrupt( rotation ).start( 80000 );    // Every 80ms
    Timer5.attachInterrupt( reset_system ).start( TIMEOUT );
}

void display_read_accel()
{
    Timer5.detachInterrupt();
    Timer3.detachInterrupt();

    u8g.firstPage();
    do
    {
        u8g.setFont( u8g_font_helvB10 );
        u8g.drawStr( 3, 25, "*SHAKE ME*" );
    }
    while( u8g.nextPage() );

    display_entered = true;
    Timer5.attachInterrupt( reset_system ).start( TIMEOUT );
}

void display_answering()
{
    Timer5.detachInterrupt();
    randomSeed( analogRead( 0 ) );
    uint8_t rand_num = random( 0, 9 );;

    u8g.firstPage();
    do
    {
        int tmp_width = u8g.getStrWidth( responses[rand_num] );

        u8g.setFont( u8g_font_helvB10 );
        u8g.drawStr( 11, 12, "*ANSWER*" );
        u8g.setFont( u8g_font_7x13 );

        if( u8g.getStrWidth( responses[rand_num] ) > u8g.getWidth() )
        {
            u8g.setFont( u8g_font_5x8 );
            u8g.drawStr( 0, 25, responses[rand_num] );
        }
        else
        {
            u8g.drawStr( ( u8g.getWidth() - tmp_width ) / 2, 25, responses[rand_num] );
        }

        //u8g.drawStr( 1, 40, "Press BTN to Repeat" );
    }
    while( u8g.nextPage() );

    display_entered = true;
    Timer5.attachInterrupt( reset_system ).start( TIMEOUT );
}

void reset_system()
{
    state = WAITING;
    display_entered = false;
    Timer5.detachInterrupt();
}

void btn_isr()
{
    static unsigned long last_interrupt_time;
    unsigned long interrupt_time = millis();

    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ( interrupt_time - last_interrupt_time > 80 )
    {
        if( state == READ_ACCEL )
            return;

        state++;
        //display_entered = false;

        if( state > ANSWERING )
            state = WAITING;
    }

    display_entered = false;
    last_interrupt_time = interrupt_time;

}


