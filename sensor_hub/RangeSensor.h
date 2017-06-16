/*
 * RangeSensor.h
 *
 * Abstraction Class for Range Sensors
 *
 * created 16 June 2017
 * by Eliot Lim    (github: @eliotlim)
 *
 * Released into the MIT License.
 */

#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <Arduino.h>

/*
 * Values of divisors
 */
#define CM 28
#define INC 71

class RangeSensor {
    public:
        void setUnit(uint8_t unit) { this->unit = unit; }
        uint8_t getUnit() { return unit; }
        virtual void doRange() = 0;
        virtual unsigned int getRange() = 0;
        virtual void setTimeout(unsigned long timeOut) = 0;

    protected:
        uint8_t unit = CM;
};

#endif // RANGESENSOR_H
