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

enum RangeUnit {CM, INC, US};

class RangeSensor {
    public:
        void setUnit(RangeUnit unit) { this->unit = unit; }
        RangeUnit getUnit() { return unit; }
        virtual void doRange() = 0;
        virtual unsigned int getRange() = 0;

    protected:
        RangeUnit unit;
};

#endif // RANGESENSOR_H
