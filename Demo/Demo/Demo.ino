#include <DueTimer.h>

byte leds[4] = { 38, 37, 39, 40 };

void rotation( void );

void setup() 
{
  
  for ( int i = 0; i < 4; i++ )
  {
    pinMode( leds[i], OUTPUT );
    digitalWrite( leds[i], LOW );
  }

  Timer3.attachInterrupt( rotation).start( 80000 );     // Every 80ms

}

void loop() 
{
  // put your main code here, to run repeatedly:

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
