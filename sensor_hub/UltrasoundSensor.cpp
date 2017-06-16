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

#include "UltrasoundSensor.h"

UltrasoundSensor::UltrasoundSensor(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut) :
    u(trigPin, echoPin, timeOut) {
}

void UltrasoundSensor::doRange() {
    uint8_t divisor;
    switch(unit) {
        case CM:
            divisor = 28;
            break;
        case INC:
            divisor = 71;
            break;
        case US:
            divisor = 1;
            break;
        default:
            divisor = 28;
    }

    this->range = u.distanceRead(divisor);
}

unsigned int UltrasoundSensor::getRange() {
    return range;
}
