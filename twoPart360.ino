/*
first test of two halves radar scanning
one half tone, other octave
*/

/// This sketch code is based on the RPLIDAR driver library provided by RoboPeak

#include <RPLidar.h>
#include <FastLED.h>
#include <Arduino.h>

RPLidar lidar;
CRGB leds[16];

#define LED_ENABLE 12   // The GPIO pin for the RGB led's common lead.
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.

const int NOTES[] = {
    392,  //G
    294,  //D
    330}; //E

const byte PIEZO_PIN = 6; //connct buzzer to pin 6

// ADDING OTHER VARIABLES ------------------------------------------------------------------

float minDistanceTopHalf = 100000;
float angleAtMinDistTopHalf = 0;

float minDistanceBottomHalf = 100000;
float angleAtMinDistBottomHalf = 0;

// SETUP ------------------------------------------------------------------

void setup()
{
    // bind the RPLIDAR driver to the arduino hardware serial
    lidar.begin(Serial2);
    FastLED.addLeds<NEOPIXEL, 6>(leds, 16);

    Serial.begin(9600);

    // set pin modes
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    digitalWrite(LED_ENABLE, HIGH);
}

// FUNCTIONS ------------------------------------------------------------------------------ //

// END OF FUNCTIONS ------------------------------------------------------------------------------ //

// SETUP ------------------------------------------------------------------

void loop()
{
    if (IS_OK(lidar.waitPoint()))
    {
        //perform data processing here...
        float distance = lidar.getCurrentPoint().distance;
        float angle = lidar.getCurrentPoint().angle;

        if (lidar.getCurrentPoint().startBit)
        {
            if (minDistanceTopHalf < 100000)
            { //if the min distance is not smaller than 10m, proceede
                Serial.print("TOP HALF Angle: ");
                Serial.println(angleAtMinDistTopHalf);

                Serial.print("TOP HALF minDistance: ");
                Serial.println(minDistanceTopHalf);
            }
            if (minDistanceBottomHalf < 100000)
            { //if the min distance is not smaller than 10m IN BOTTOM HALF, proceede
                Serial.print("                             BOTTOM HALF Angle: ");
                Serial.println(angleAtMinDistBottomHalf);

                Serial.print("                             BOTTOM HALF minDistance: ");
                Serial.println(minDistanceBottomHalf);
            }

            minDistanceTopHalf = 100000;
            angleAtMinDistTopHalf = 0;

            minDistanceBottomHalf = 100000;
            angleAtMinDistBottomHalf = 0;
        }
        else
        {
            if (angle < 180)
            {
                if (distance > 0 && distance < minDistanceTopHalf)
                {
                    minDistanceTopHalf = distance;
                    angleAtMinDistTopHalf = angle;
                }
            }
            else
            {
                if (distance > 0 && distance < minDistanceBottomHalf)
                {
                    minDistanceBottomHalf = distance;
                    angleAtMinDistBottomHalf = angle - 180;
                }
            }
            

        }
    }

    else
    {
        analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

        // try to detect RPLIDAR...
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100)))
        {
            //detected...
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(1000);
        }
    }
} //END OF LOOP