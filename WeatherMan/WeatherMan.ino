/*******************************************************************************
 * Title                 :   WeatherMan Demo
 * Filename              :   WeatherMan.ino
 * Author                :   RL
 * Origin Date           :   04/10/2015
 * Notes                 :   Running on Flip and Click Due CPU
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
 *
 *    Date    Software Version    Initials   Description
 *  04/10/2015    .1               RL        Module Created
 *  06/10/2015    .2               RL        Added icons and resource.h
 *  06/10/2015    .3               RL        Added interrupts and timer
 *
 *******************************************************************************/
/**
 *  @file WeatherMan.ino
 *  @brief Demonstrates runing multiple click boards on flip and click board
 *  
 *  This example uses the following boards:
 *  Main Board - Flip and Click
 *  
 *  <b>Click Boards:</b>
 *  <ul>
 *  <li>OLED W Click</li>
 *  <li>Thunder Click</li>
 *  <li>Weather Click</li>
 *  <li>Ambient 2 Click</li>
 *  </ul>
 *  
 *  The following libraries are used:
 *  <ul>
 *  <li>Wire</li>
 *  <li>SPI</li>
 *  <li>DueTimer</li>
 *  <li>BME280_MOD-1022</li>
 *  <li>U8glib</li>
 *  <li>AS3935</li>
 *  <li>flip_click_defs</li>
 *  <li>resources</li>
 *  </ul>
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <DueTimer.h>
#include <BME280_MOD-1022.h>
#include <U8glib.h>
#include <AS3935.h>
#include "flip_click_defs.h"
#include "resources.h"

/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
// Ambient
#define OPT3001_I2C_ADDR   0x44

// Thunder
#define THUNDER_CS         B_CS
#define THUNDER_INT        B_INT
#define INDOR              0x12
#define OUTDOOR            0x0E

// OLED
#define SSD_CS             A_CS
#define SSD_DC             A_PWM
#define SSD_RESET          A_RST

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
#define get_thunder_distance()   weater_data.thunder_distance
#define get_temp()               weather_data.temp
#define get_humidity()           weather_data.humidity
#define get_pressure()           weather_data.pressure
#define get_ambient()            weather_data.ambient_light

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef struct
{
    float temp;
    float humidity;
    float pressure;
    uint16_t thunder_distance;
    float ambient_light;
} weather_data_t;

enum
{
    TEMPERATURE = 0,
    PRESSURE,
    HUMIDITY,
    AMBIENT,
    LIGHTNING
};

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
weather_data_t weather_data;
volatile bool update_flag;
volatile bool thunder_flag;
uint8_t state;

byte SPItransfer( byte sendByte );
U8GLIB_SSD1306_96X40 u8g( SSD_CS, SSD_DC ); /**< Display object */
AS3935 AS3935( SPItransfer, THUNDER_CS, THUNDER_INT );

byte leds[4] = { LEDA, LEDB, LEDC, LEDD };  /**< Built in LEDs  */

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

void display_init( void );
void display_update( void );

void bme_init( void );
void bme_update( void );

void thunder_init( void );
void thunder_update( void );

void ambient_init( void );
void ambient_update( void );

void weather_update( void );

//ISRs
void update_isr( void );
void thunder_detect( void );

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
void setup()
{
    Serial.begin( 57600 );  /**< For debugging purposes */
    SPI.begin();            /**< Used with OLED, Rotary, Thermo */
    SPI.setDataMode( SPI_MODE1 );
    Wire.begin();           /**< Used with RTC */

    display_init();
    bme_init();
    thunder_init();
    ambient_init();

    Timer4.attachInterrupt( update_isr ).start( 2000000 );
}

void loop()
{
    if( update_flag )
    {
        weather_update();
        display_update();
    }
}


/************************************** Private Functions **********************/
/**
 * @brief Initializes display and renders logo for 3 seconds
 */
void display_init()
{
    // Display Pins init
    pinMode( SSD_CS,       OUTPUT );
    pinMode( SSD_DC,       OUTPUT );
    pinMode( SSD_RESET,    OUTPUT );
    digitalWrite( SSD_CS,    HIGH );
    digitalWrite( SSD_DC,    HIGH );
    digitalWrite( SSD_RESET, HIGH );
    u8g.setColorIndex( 1 );  /**<  Sets displays to single black white color */

    u8g.firstPage();

    do
    {
        u8g.drawXBMP( 25, 1, weatherman_width, weatherman_height, weatherman_bits );
    }
    while( u8g.nextPage() );

    delay( 3000 );
}

/**
 * @brief Re-draws display on demand
 * 
 * While this can be called as fast as the processor will go, by teathering it 
 * to a timer interrupt, the processor can sleep, saving a substantial amount of
 * energy.
 */
void display_update()
{
    char txt[20];
    u8g.firstPage();
    do
    {
        switch( state )
        {
            case TEMPERATURE:
                u8g.setFont( u8g_font_unifont );
                sprintf( txt, "%2.1f C", get_temp() );
                u8g.drawStr( 40, 25, txt );
                u8g.drawXBMP( 0, 1, temperature_width, temperature_height, temperature_bits );
                break;
            case PRESSURE:
                u8g.setFont( u8g_font_unifont );
                sprintf( txt, "%4.0f", get_pressure() );
                u8g.drawStr( 40, 25, txt );
                u8g.drawXBMP( 0, 1, pressure_width, pressure_height, pressure_bits );
                break;
            case HUMIDITY:
                u8g.setFont( u8g_font_unifont );
                sprintf( txt, "%2.0f%%", get_humidity() );
                u8g.drawStr( 40, 25, txt );
                u8g.drawXBMP( 0, 1, humidity_width, humidity_height, humidity_bits );
                break;
            case AMBIENT:
                u8g.setFont( u8g_font_unifont );
                sprintf( txt, "%3.0f Lux", get_ambient() );
                u8g.drawStr( 40, 25, txt );

                if( get_ambient() < 0 )
                    weather_data.ambient_light = 0;

                if( get_ambient() == 100.0f )
                    u8g.drawXBMP( 0, 1, sun_width, sun_height, sun_bits );
                else if( get_ambient() <= 90.0 && get_ambient() > 70.0 )
                    u8g.drawXBMP( 0, 1, partly_cloudy_width, partly_cloudy_height,
                                  partly_cloudy_bits );
                else if( get_ambient() <= 70.0 && get_ambient() > 30.0 )
                    u8g.drawXBMP( 0, 1, rain_width, rain_height, rain_bits );
                else
                    u8g.drawXBMP( 0, 1, moon_width, moon_height, moon_bits );
                break;
            case LIGHTNING:
                if( thunder_flag )
                {
                    u8g.drawXBMP( 0, 1, lightning_width, lightning_height, lightning_bits );
                    thunder_flag = false;
                }
                else
                {
                    u8g.setFont( u8g_font_unifont );
                    u8g.drawStr( 45, 25, "None" );
                    u8g.drawXBMP( 0, 1, no_lightning_width, no_lightning_height,
                                  no_lightning_bits );
                }
                break;
        }
    }
    while( u8g.nextPage() );

    state++;

    if( state > LIGHTNING )
        state = TEMPERATURE;
}

/**
 * @brief Initializes BME280 Sensor
 * 
 * @note Creation of object initializes the bus.
 */
void bme_init()
{
    Serial.println( "BME280 Initialized" );
}

/**
 * @brief Updates weather data from BME sensor
 */
void bme_update()
{
    // need to read the NVM compensation parameters
    BME280.readCompensationParams();

    // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
    BME280.writeOversamplingPressure(
        os1x );  // 1x over sampling (ie, just one sample)
    BME280.writeOversamplingTemperature( os1x );
    BME280.writeOversamplingHumidity( os1x );

    // example of a forced sample.  After taking the measurement the chip goes back to sleep
    BME280.writeMode( smForced );

    while( BME280.isMeasuring() );

    // read out the data - must do this before calling the getxxxxx routines
    BME280.readMeasurements();
    weather_data.temp     = BME280.getTemperature();  // must get temp first
    weather_data.humidity = BME280.getHumidity();
    weather_data.pressure = BME280.getPressure();

    return;
}

/**
 * @brief Initializes the Thunder IC
 */
void thunder_init()
{
    // Thunder
    pinMode( THUNDER_CS,    OUTPUT );
    digitalWrite( THUNDER_CS, HIGH );
    pinMode( THUNDER_INT,   INPUT );

    AS3935.reset();
    if( !AS3935.calibrate() )
        Serial.println( "Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!" );

    AS3935.setIndoors();
    AS3935.enableDisturbers();

    attachInterrupt( digitalPinToInterrupt( THUNDER_INT ), thunder_detect,
                     FALLING );
}

/**
 * @brief Updates range of thunder detection
 * 
 * TODO: Unfinished
 */
void thunder_update()
{
    if( thunder_flag )
        Serial.println( "Thunder Detected" );
}

/**
 * @brief Initializes ambient light sensor
 */
void ambient_init()
{
    Wire.beginTransmission( OPT3001_I2C_ADDR );
    Wire.write( 0x01 );  /**< Configuration register */
    Wire.write( 0xC6 );  /**< Set Mode to continuous conversions */
    Wire.write( 0x10 );
    Wire.endTransmission();
}

/**
 * @brief Updates ambient light reading from sensor
 */
void ambient_update()
{
    byte tmp[2];
    int i = 0;

    Wire.beginTransmission( OPT3001_I2C_ADDR ); /**< Issue I2C start signal */
    Wire.write( 0x00 );
    Wire.endTransmission();                     /**< stop transmitting */
    Wire.beginTransmission( OPT3001_I2C_ADDR );
    Wire.write( 0x00 );
    Wire.requestFrom( OPT3001_I2C_ADDR, 2 );

    while( Wire.available() && i != 2 )
        tmp[i++] = Wire.read();

    Wire.endTransmission();

    i = ( ( tmp[0] << 8 ) | tmp[1] );   
    tmp[0] = tmp[0] >> 4;               
    // Lux equation
    weather_data.ambient_light = 0.01 * ( 2 << tmp[0] ) * i; 

    // The most useful ALS range is in the 1 to 1,000 lux range

    if( weather_data.ambient_light >= 1000.0f )  
        weather_data.ambient_light = 1000.0f;

    weather_data.ambient_light  /=  10.0f; // Lux to percents conversion
}

/**
 * @brief Wrapper for all update functions
 */
void weather_update()
{
    bme_update();
    thunder_update();
    ambient_update();
    update_flag = false;
}

/** 
 *  @brief Implementation of SPI transfer that gets passed to AS3935
 */
byte SPItransfer( byte sendByte )
{
    return SPI.transfer( sendByte );
}

/************************* ISRs *************************/
void update_isr()
{
    update_flag = true;
}

void thunder_detect()
{
    thunder_flag = true;
}

/*************** END OF FUNCTIONS ***************************************************************************/

