/*
 * SRFRangeSensor.cpp
 *
 * Abstraction Class for SRF I2C Range Sensors e.g. SRF08, SRF10
 *
 * created 16 June 2017
 * by Eliot Lim    (github: @eliotlim)
 *
 * Released into the MIT License.
 */

#include "SRFRangeSensor.h"

SRFRangeSensor::SRFRangeSensor(byte address) :
    r(address) {
}

void SRFRangeSensor::doRange() {
    uint8_t unit;
    switch(unit) {
        case CM:
            unit = CM;
            break;
        case INC:
            unit = INC;
            break;
        default:
            unit = CM;
    }

    r.distanceRange(unit);
    delayMicroseconds(timeout);
}

unsigned int SRFRangeSensor::getRange() {
    unsigned int range_new = r.distanceRead();
    return range_new != 0 ? range = range_new : range;
}
