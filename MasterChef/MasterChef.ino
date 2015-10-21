/*******************************************************************************
* Title                 :   Master Chef Demo
* Filename              :   MasterChef.ino
* Author                :   RL
* Origin Date           :   04/10/2015
* Notes                 :   Running on Flip and Click Due CPU
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  04/10/2015    .1               RL        Module Created.
*
*******************************************************************************/
/**
 *  @file MasterChef.ino
 *  @brief Demonstrates running 4 click boards
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <stdio.h>
#include <Time.h>
#include <U8glib.h>
#include <DueTimer.h>
#include <DS1307RTC.h>
#include "flip_click_defs.h"
#include "resources.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
// OLED
#define SSD_CS     A_CS
#define SSD_DC     A_PWM
#define SSD_RESET  A_RST

// RTC
#define RTC_INT    D_INT

// Rotary B
#define ROT_ECB    C_AN
#define ROT_ECA    C_PWM
#define ROT_INT    C_INT
#define ROT_RST    C_RST
#define ROT_CS     C_CS

// THERMO
#define THERM_CS   B_CS

#define DS1307_ID       0x68
#define DS1307_CONTROL  0x07

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
enum
{
    WAITING = 0,
    SET_TIME,
    SET_TEMP,
    COUNT_DOWN,
    ALARM
};

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
volatile byte state;  /**< State needs to be volatile because of interrupt */
volatile bool display_entered = false;
volatile bool A_set;
volatile bool B_set;
volatile bool temp_update_flag;
volatile tmElements_t timer_time;
volatile uint8_t set_temp = 0;

uint16_t rot_leds;
float current_temp = 0.0f;

U8GLIB_SSD1306_96X40 u8g( SSD_CS, SSD_DC ); /**< Display object */
byte leds[4] = { LEDA, LEDB, LEDC, LEDD };  /**< Built in LEDs  */

/******************************************************************************
* Function Prototypes
*******************************************************************************/
// Menu displays
void menu_waiting( void );
void menu_set_time( void );
void menu_set_temp( void );
void menu_count_down( void );
void menu_alarm( void );

// RTC
void rtc_enable_swo( void );
void rtc_disable_swo( void );

// Rotary
void write_rot( uint16_t led );
void rot_animate_right( void );
void rot_animate_left( void );

// Helper functions
void add_min( void );
void sub_min( void );
void add_temp( void );
void sub_temp( void );
void reset_system( void );

// ISRs
void menu_isr( void );
void time_tick_isr( void );
void rot_left_isr( void );
void rot_right_isr( void );
void read_temp_isr( void );
/******************************************************************************
* Function Definitions
*******************************************************************************/
void setup()
{
    Serial.begin( 57600 );  /**< For debugging purposes */
    SPI.begin();            /**< Used with OLED, Rotary, Thermo */
    Wire.begin();           /**< Used with RTC */

    // RTC - ( Real Time Clock )
    rtc_disable_swo();
    pinMode( RTC_INT, INPUT );

    // Built in LEDs
    for( int i = 0; i < 4; i++ )
    {
        pinMode( leds[i], OUTPUT );
        digitalWrite( leds[i], LOW );
    }

    // Display Pins init
    pinMode( SSD_CS, OUTPUT );
    digitalWrite( SSD_CS, HIGH );
    pinMode( SSD_DC, OUTPUT );
    digitalWrite( SSD_DC, HIGH );
    pinMode( SSD_RESET, OUTPUT );
    digitalWrite( SSD_RESET, HIGH );
    u8g.setColorIndex( 1 ); /**<  Sets displays to single bw color */

    // Rotary pins init
    pinMode( ROT_CS, OUTPUT );
    digitalWrite( ROT_CS, HIGH );
    pinMode( ROT_INT, INPUT );
    digitalWrite( ROT_INT, LOW );
    pinMode( ROT_ECA, INPUT );
    pinMode( ROT_ECB, INPUT );
    write_rot( 0 );

    attachInterrupt( digitalPinToInterrupt( ROT_INT ), menu_isr, RISING );

    state = WAITING;
}

// Looping function
void loop()
{
    if( temp_update_flag )
        temp_update();

    switch( state )
    {
        case WAITING:
            if( !display_entered )
            {
                reset_system();
                menu_waiting();
            }
            break;
        case SET_TIME:
            if( !display_entered )
            {
                attachInterrupt( digitalPinToInterrupt( ROT_ECA ), rot_left_isr, CHANGE );
                attachInterrupt( digitalPinToInterrupt( ROT_ECB ), rot_right_isr, CHANGE );
                display_entered = true;
            }
            menu_set_time();
            break;
        case SET_TEMP:
            menu_set_temp();
            break;
        case COUNT_DOWN:
            if( !display_entered )
            {
                display_entered = true;
                rtc_enable_swo();
                delay( 100 );
                detachInterrupt( digitalPinToInterrupt( ROT_ECA ) );
                detachInterrupt( digitalPinToInterrupt( ROT_ECB ) );
                attachInterrupt( digitalPinToInterrupt( RTC_INT ), time_tick_isr, RISING );
                Timer5.attachInterrupt( read_temp_isr ).start( 1000000 );
            }
            menu_count_down();
            break;
        case ALARM:
            if( !display_entered )
            {
                menu_alarm();
            }
            break;
    }
}

/************************************
 *********Private Functions**********
 ***********************************/
/**
 * @brief Menu display waiting
 *
 * Initial screen presented to user
 */
void menu_waiting()
{    
    u8g.firstPage();

    do
    {
        u8g.setFont( u8g_font_courB10 );
        u8g.drawStr( 40, 18, "Press" );
        u8g.drawStr( 40, 32, "Button");
        u8g.drawXBMP( 0, 1, chef_width, chef_height, chef_bits );
        
    }
    while( u8g.nextPage() );

    display_entered = true;
}

/**
 * @brief Set Time menu
 */
void menu_set_time()
{
    tmElements_t tm;
    char tmp[20];

    RTC.read( tm );
    set_temp = 0;

    u8g.firstPage();

    do
    {
        u8g.setFont( u8g_font_5x8 );
        sprintf( tmp, "Time: %02d:%02d:%02d", tm.Hour, tm.Minute, tm.Second );
        u8g.drawStr( 1, 10, tmp );
        u8g.drawStr( 1, 20, "Set:" );
        sprintf( tmp, "%02dh %02dm", timer_time.Hour, timer_time.Minute );
        u8g.setFont( u8g_font_profont15 );
        u8g.drawStr( 22, 30, tmp );
        u8g.drawXBMP( u8g.getWidth() - timer_width, 1, timer_width, timer_height, timer_bits );
        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 1, 40, "Press Btn Continue" );
    }
    while( u8g.nextPage() );
}

/**
 * @brief Set Temp menu
 */
void menu_set_temp()
{
    char tmp[20];
    
    u8g.firstPage();

    do
    {
        u8g.setFont( u8g_font_5x8 );
        sprintf( tmp, "Current: %3.2fC", current_temp );
        u8g.drawStr( 1, 10, tmp );
        u8g.drawStr( 1, 20, "Set:" );
        sprintf( tmp, "%dC", set_temp );
        u8g.setFont( u8g_font_courR14 );
        u8g.drawStr( 40, 27, tmp ); 
        u8g.drawXBMP( u8g.getWidth() - temp_width, 2, temp_width, temp_height, temp_bits );
        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 1, 40, "Press Btn Continue" );
    }
    while( u8g.nextPage() );
}

/**
 * @brief Count Down menu
 */
void menu_count_down()
{
    char tmp[20];

    u8g.firstPage();

    do
    {
        if( current_temp < set_temp )
        {
            digitalWrite( LEDC, HIGH );
            u8g.drawXBMP( u8g.getWidth() - heat_width, 0, heat_width, heat_height, heat_bits );
        } else {
            digitalWrite( LEDC, LOW );
        }
        
        u8g.setFont( u8g_font_courR14 );
        sprintf( tmp, 
                 "%02d:%02d",  
                 timer_time.Minute,
                 timer_time.Second );
        
        u8g.drawStr( 1, 25, tmp );
        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 1, 40, "Btn Cancel" );
    }
    while( u8g.nextPage() );
}

/**
 * @brief Alarm menu
 */
void menu_alarm()
{
    rtc_disable_swo();
    detachInterrupt( digitalPinToInterrupt( RTC_INT ) );
    Timer5.detachInterrupt();
    
    u8g.firstPage();

    do
    {
        u8g.drawXBMP( 0, 0, alarm_width, alarm_height, alarm_bits );
        u8g.setFont( u8g_font_5x8 );
        u8g.drawStr( 40, 40, "Btn to Clr" );
    }
    while( u8g.nextPage() );

    display_entered = true;
}

/**
 * @brief Enables the Square Wave Output for the RTC
 */
void rtc_enable_swo()
{
    Wire.beginTransmission( DS1307_ID );
    Wire.write( DS1307_CONTROL );
    Wire.write( 0x10 | 0x80 );
    Wire.endTransmission();
}

/**
 * @brief Disables the Square Wave Output for the RTC
 */
void rtc_disable_swo()
{
    Wire.beginTransmission( DS1307_ID );
    Wire.write( DS1307_CONTROL );
    Wire.write( 0x00 | 0x80 );
    Wire.endTransmission();
}

/**
 * @brief Writes digit to LEDs around rotary switch
 *
 * @param uint16_t led - number of led to turn on
 */
void write_rot( uint16_t led )
{
    SPI.transfer( led >> 8 );
    SPI.transfer( led & 0xFF );
    digitalWrite( ROT_CS, LOW );
    delayMicroseconds( 10 );
    digitalWrite( ROT_CS, HIGH );
}

void rot_animate_right()
{

    if( rot_leds == 0xffff )
    {
        write_rot( rot_leds );
        rot_leds = 0;
        Timer4.detachInterrupt();
    }

    write_rot( rot_leds );
    rot_leds <<= 1;
    rot_leds |= 0x1;

}

void rot_animate_left()
{
    if( rot_leds == 0 )
        Timer4.detachInterrupt();

    write_rot( rot_leds );
    rot_leds >>= 1;
}

void add_min()
{
    if( timer_time.Minute == 59 )
    {
        timer_time.Minute = 0;
        if( timer_time.Hour < 10 )
            timer_time.Hour++;
    }
    else
        timer_time.Minute++;
}

void sub_min()
{
    if( timer_time.Minute == 0 && timer_time.Hour > 0 )
    {
        timer_time.Hour--;
        timer_time.Minute = 59;
    }
    else if( timer_time.Minute > 0 )
        timer_time.Minute--;
}

void add_temp()
{
    if( set_temp < 301 )
        set_temp++;
}

void sub_temp()
{
    if( set_temp > 0 )
        set_temp--;
}


void reset_system()
{
    set_temp = 0;
    timer_time.Hour   = 0;
    timer_time.Minute = 0;
    timer_time.Second = 0;
    detachInterrupt( digitalPinToInterrupt( RTC_INT ) );
    rtc_disable_swo();
    // TODO: If timer is attached detach
    Timer5.detachInterrupt();
    delay( 100 );
    digitalWrite( LEDA, LOW );
    digitalWrite( LEDC, LOW );
    write_rot( 0 );
}

/**************************** ISR ***************************/

/**
 * @brief Changes between menus
 *
 * Interrupt ISR for changing menus from rotary switch
 *
 */
void menu_isr()
{
    static unsigned long last_interrupt_time;
    unsigned long interrupt_time = millis();

    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ( interrupt_time - last_interrupt_time > 200 )
    {
        if( state == ALARM )
            state = WAITING;
        else if( state == COUNT_DOWN )
        {
            state = WAITING;
        }
        else
            state++;
    }

    display_entered = false;
    last_interrupt_time = interrupt_time;
}

/**
 * @brief ISR for representing RTC square wave output
 */
void time_tick_isr()
{
    static uint16_t rot_leds;
    uint32_t rem_seconds = 0;

    digitalWrite( LEDA, ( digitalRead( LEDA ) == HIGH ) ? LOW : HIGH );

    rem_seconds += timer_time.Hour   * 3600;
    rem_seconds += timer_time.Minute * 60;
    rem_seconds += timer_time.Second;

    if( rem_seconds == 1 )
    {
        state = ALARM;
        display_entered = false;
    }
    else
    {
        rem_seconds -= 1;
        timer_time.Hour = rem_seconds / 3600;
        rem_seconds -= ( timer_time.Hour * 3600 );
        timer_time.Minute = ( rem_seconds / 60 );
        rem_seconds -= ( timer_time.Minute * 60 );
        timer_time.Second = rem_seconds;
    }

    if( rot_leds == 0 )
        rot_leds = 0xffff;
    else
        rot_leds >>= 1;

    write_rot( rot_leds );
}


void rot_left_isr()
{
    // Looking for low to high on Channel A
    if( digitalRead( ROT_ECA ) == HIGH )
    {
        if( digitalRead( ROT_ECB ) == LOW )
        {
            switch( state )
            {
                case SET_TIME:
                    add_min();
                    break;
                case SET_TEMP:
                    add_temp();
                    break;
            }
        } else {
            switch( state )
            {
                case SET_TIME:
                    sub_min();
                    break;
                case SET_TEMP:
                    sub_temp();
                    break;
            } 
        }
    } else {
        if( digitalRead( ROT_ECB ) == HIGH )
        {            
            switch( state )
            {
                case SET_TIME:
                    add_min();
                    break;
                case SET_TEMP:
                    add_temp();
                    break;
            }
        } else {
             switch( state )
            {
                case SET_TIME:
                    sub_min();
                    break;
                case SET_TEMP:
                    sub_temp();
                    break;
            } 
        }
    }        
}

void rot_right_isr()
{
     // Looking for low to high on Channel A
    if( digitalRead( ROT_ECB ) == HIGH )
    {
        if( digitalRead( ROT_ECA ) == HIGH )
        {
            switch( state )
            {
                case SET_TIME:
                    add_min();
                    break;
                case SET_TEMP:
                    add_temp();
                    break;
            }
        } else {
            switch( state )
            {
                case SET_TIME:
                    sub_min();
                    break;
                case SET_TEMP:
                    sub_temp();
                    break;
            } 
        }
    } else {
        if( digitalRead( ROT_ECA ) == LOW )
        {            
            switch( state )
            {
                case SET_TIME:
                    add_min();
                    break;
                case SET_TEMP:
                    add_temp();
                    break;
            }
        } else {
             switch( state )
            {
                case SET_TIME:
                    sub_min();
                    break;
                case SET_TEMP:
                    sub_temp();
                    break;
            } 
        }
    }        
}

void temp_update()
{
    uint8_t temp[4];
    int16_t whole;
    int16_t rem;

    digitalWrite( THERM_CS, LOW );
    delay( 350 );

    for( int i = 0; i < 4; i++ )
        temp[i] = SPI.transfer( 0x00 );

    digitalWrite( THERM_CS, HIGH );

    whole = ( temp[0] << 8 ) | temp[1];
    rem = ( whole >> 2 ) & 0x03;
    current_temp = ( float )rem * 0.25f;
    current_temp += ( whole >> 4 );

    temp_update_flag = false;
}

/**
 * @brief Reads Tempurature from Thermo Click
 *
 * @returns double - current temp
 */
void read_temp_isr( void )
{
    temp_update_flag = true;
}


/*************** END OF FUNCTIONS ***************************************************************************/









