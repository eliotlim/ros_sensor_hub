/*
 * UltrasoundSensor.h
 *
 * Abstraction Class for Ultrasound Range Sensors e.g. SRF05, Ping)))
 *
 * created 16 June 2017
 * by Eliot Lim    (github: @eliotlim)
 *
 * Released into the MIT License.
 */

#ifndef ULTRASOUNDSENSOR_H
#define ULTRASOUNDSENSOR_H

#include "RangeSensor.h"
#include <Ultrasonic.h>

class UltrasoundSensor : public RangeSensor {
    public:
        UltrasoundSensor(uint8_t sigPin) : UltrasoundSensor(sigPin, sigPin) {};
        UltrasoundSensor(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut = 20000UL);
        void doRange();
        unsigned int getRange();
        void setTimeout(unsigned long timeOut) { u.setTimeout(timeOut); }

    private:
        Ultrasonic u;
        unsigned int range;
};

#endif // ULTRASOUNDSENSOR_H
