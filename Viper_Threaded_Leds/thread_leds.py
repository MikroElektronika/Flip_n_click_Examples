################################################################################
# Threaded LEDs
#
# Created: 2016-01-30 23:19:09.376430
#
################################################################################

import streams

# create a serial port with default parameters
streams.serial()

LEDA=38
LEDB=37
LEDC=39
LEDD=40

pinMode(LEDA, OUTPUT)
pinMode(LEDB, OUTPUT)
pinMode(LEDC, OUTPUT)
pinMode(LEDD, OUTPUT)

# Define a function to be used in the various threads.
# Parameters are passed to the function when the thread is created and then used in the thread loop.
# To be continuously executed by a thread a function requires an infinite loop,
# otherwise when the function terminates the thread is closed

def threadfn( number, delay, led):
    while True:
        led_state = digitalRead(led)
        if led_state is 0:
            led_state = 1
        else:
            led_state = 0
        digitalWrite(led, led_state)
        print("I am the thread", number)
        print("I run every:", delay, "msec")
        print()                # just add an empty line for console output readability
        sleep(delay)

# create the various threads using the same function but passing different parameters
thread( threadfn, 1, 500, LEDA)
thread( threadfn, 2, 1000, LEDB)
thread( threadfn, 3, 1300, LEDC)
thread( threadfn, 4, 800, LEDD)