/*******************************************************************************
 * Title                 :   Remote Weather Station
 * Filename              :   RemoteWeather.ino
 * Author                :   RL
 * Origin Date           :   26/11/2015
 * Notes                 :   Running on Flip and Click Due CPU
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
 *
 *    Date    Software Version    Initials   Description
 *  25/10/2015    .1               RL        Module Created
 *  26/10/2015    .2               RL        Added timers
 *  26/10/2015    .3               RL        Added interrupts and timer
 *
 *******************************************************************************/
/**
 *  @file RemoteWeather.ino
 *  @brief Demonstrates runing multiple click boards on flip and click board
 *  and transmission of the various sensor readings to a coordinator.
 *  
 *  This example uses the following boards:
 *  Main Board - Flip and Click
 *  
 *  <b>Click Boards:</b>
 *  <ul>
 *  <li>XBee click</li>
 *  <li></li>
 *  <li>Weather Click</li>
 *  <li>Ambient 2 Click</li>
 *  </ul>
 *  
 *  The following libraries are used:
 *  <ul>
 *  <li>Wire</li>
 *  <li>SPI</li>
 *  <li>DueTimer</li>
 *  <li>flip_click_defs</li>
 *  </ul>
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <DueTimer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "flip_click_defs.h"

/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
// Ambient
#define OPT3001_I2C_ADDR   0x44
#define XBEE_RST A_RST

#define SEALEVELPRESSURE_HPA (1013.25)

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
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
static weather_data_t weather_data;

byte leds[4] = { LEDA, LEDB, LEDC, LEDD };  /**< Built in LEDs  */

// When to update and transmit
volatile bool update_flag;

Adafruit_BME280 bme; 
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void bme_init( void );
void bme_update( void );

void ambient_init( void );
void ambient_update( void );

void weather_update( void );
void transmit( void );

//ISRs
void update_isr( void );

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
void setup() 
{
  SPI.begin();
  Wire.begin();
  Wire1.begin();
  Serial.begin( 57600 );  // For debugging
  Serial1.begin( 9600 );

  ambient_init();
  bme_init();
  
  pinMode( XBEE_RST, OUTPUT );
  digitalWrite( XBEE_RST, HIGH );

  Timer4.attachInterrupt( update_isr ).start( 1000000 );  // Every 1s
}

void loop() 
{
  if( update_flag )
  {
      weather_update();
      transmit();
      update_flag = false;    
  }
}

void bme_init()
{
  if( !bme.begin( 0x76 ) ) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void bme_update()
{
    weather_data.temp = bme.readTemperature();
    weather_data.pressure = bme.readPressure() / 100.0f;
    weather_data.humidity = bme.readHumidity();
}

void weather_update()
{
   ambient_update();
   bme_update();  
}

void transmit()
{
  char txt[50];
  sprintf( txt, "T:%2.1f,P:%3.0f,H:%2.0f,L:%3.0f", get_temp(), get_pressure(), get_humidity(), get_ambient() );
  Serial.println( txt );
  Serial1.println( txt );
}


/**
 * @brief Initializes ambient light sensor
 */
void ambient_init()
{
    Wire1.beginTransmission( OPT3001_I2C_ADDR );
    Wire1.write( 0x01 );  /**< Configuration register */
    Wire1.write( 0xC6 );  /**< Set Mode to continuous conversions */
    Wire1.write( 0x10 );
    Wire1.endTransmission();
}

/**
 * @brief Updates ambient light reading from sensor
 */
void ambient_update()
{
    byte tmp[2];
    int i = 0;

    Wire1.beginTransmission( OPT3001_I2C_ADDR ); /**< Issue I2C start signal */
    Wire1.write( 0x00 );
    Wire1.endTransmission();                     /**< stop transmitting */
    Wire1.beginTransmission( OPT3001_I2C_ADDR );
    Wire1.write( 0x00 );
    Wire1.requestFrom( OPT3001_I2C_ADDR, 2 );

    while( Wire1.available() && i != 2 )
        tmp[i++] = Wire1.read();

    Wire1.endTransmission();

    i = ( ( tmp[0] << 8 ) | tmp[1] );   
    tmp[0] = tmp[0] >> 4;               
    // Lux equation
    weather_data.ambient_light = 0.01 * ( 2 << tmp[0] ) * i; 

    // The most useful ALS range is in the 1 to 1,000 lux range
    if( weather_data.ambient_light >= 1000.0f )  
        weather_data.ambient_light = 1000.0f;

    weather_data.ambient_light  /=  10.0f; // Lux to percents conversion
}


void update_isr()
{
    update_flag = true;
}
