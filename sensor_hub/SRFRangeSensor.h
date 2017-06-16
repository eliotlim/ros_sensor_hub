/*
 * SRFRangeSensor.h
 *
 * Abstraction Class for SRF I2C Range Sensors e.g. SRF08, SRF10
 *
 * created 16 June 2017
 * by Eliot Lim    (github: @eliotlim)
 *
 * Released into the MIT License.
 */

#ifndef SRFRANGESENSOR_H
#define SRFRANGESENSOR_H

#include "RangeSensor.h"
#include <SRFRanger.h>

class SRFRangeSensor : public RangeSensor {
    public:
        SRFRangeSensor(byte address);
        void doRange();
        unsigned int getRange();
        void setTimeout(unsigned long timeOut) { timeout = timeOut; }

    private:
        SRFRanger r;
        unsigned int range;
        unsigned long timeout = 20000UL;
};

#endif // SRFRANGESENSOR_H
