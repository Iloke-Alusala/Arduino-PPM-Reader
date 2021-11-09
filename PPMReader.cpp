/*
 * The reason why I put the main code in this section is because there will be a shorter latency while transmitting signals to the drone.
 * The reason for that is because this section of the code runs with an interrupt pin. If you wish to change any part of the code, do it under
 * the function "PPMReader::handleInterrupt(void)"
 */
#include "PPMReader.h"

static PPMReader *PPMReader::ppm;

static void PPMReader::PPM_ISR(void) {
  ppm->handleInterrupt(); 
}


PPMReader::PPMReader(byte interruptPin, byte channelAmount):
    interruptPin(interruptPin), channelAmount(channelAmount) {
    // Setup an array for storing channel values
    rawValues = new unsigned [channelAmount];
    validValues = new unsigned [channelAmount];
    for (int i = 0; i < channelAmount; ++i) {
        rawValues[i] = 0;
        validValues[i] = 0;
    }
    m1 = 6, m2 = 7;
    s1 = 0, s2= 0;
    // Attach an interrupt to the pin
    pinMode(interruptPin, INPUT);
    pinMode(m1, OUTPUT);
    pinMode(m2, OUTPUT);
    
    if(ppm == NULL) {
        ppm = this;
        attachInterrupt(digitalPinToInterrupt(interruptPin), PPM_ISR, RISING);
    }
}

PPMReader::~PPMReader(void) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    if(ppm == this) ppm = NULL;
    delete [] rawValues;
    delete [] validValues;
}

void PPMReader::handleInterrupt(void) {
    // Remember the current micros() and calculate the time since the last pulseReceived()
    unsigned long previousMicros = microsAtLastPulse;
    microsAtLastPulse = micros();
    unsigned long time = microsAtLastPulse - previousMicros;

    if (time > blankTime) {
        /* If the time between pulses was long enough to be considered an end
         * of a signal frame, prepare to read channel values from the next pulses */
        pulseCounter = 0;
        /* Channel 6 represents a 3-position toggle switch on the transmitter
         * Position 1 - Turn the pulley motor off. Value sent to Arduino - 1000
         * Position 2 - Turn the motor clockwise - Lower the Package. Value sent to Arduino - 1500
         * Position 3 - Turn the motor anticlockwise - Lift the Package. Value sent to Arduino - 2000
         */
        if(validValues[6] >= 1100){
          if(validValues[6] < 1600){
            s1 = 1, s2 = 0;
          }
          else{
            s1 = 0, s2 = 1;
          }
              digitalWrite(m1, s1);
              digitalWrite(m2, s2);
        }
        else{
            digitalWrite(m1, LOW);
            digitalWrite(m2, LOW);
        }
    }
    else {
        // Store times between pulses as channel values
        if (pulseCounter < channelAmount) {
            rawValues[pulseCounter] = time;
            if (time >= minChannelValue - channelValueMaxError && time <= maxChannelValue + channelValueMaxError) {
                validValues[pulseCounter] = constrain(time, minChannelValue, maxChannelValue);
            }
        }
        ++pulseCounter;
    }
}

unsigned PPMReader::rawChannelValue(byte channel) {
    // Check for channel's validity and return the latest raw channel value or 0
    unsigned value = 0;
    if (channel >= 1 && channel <= channelAmount) {
        noInterrupts();
        value = rawValues[channel-1];
        interrupts();
    }
    return value;
}

unsigned PPMReader::latestValidChannelValue(byte channel, unsigned defaultValue) {
    // Check for channel's validity and return the latest valid channel value or defaultValue.
    unsigned value;
    if(micros() - microsAtLastPulse > failsafeTimeout) return defaultValue;
    if (channel >= 1 && channel <= channelAmount) {
        noInterrupts();
        value = validValues[channel-1];
        interrupts();
        if(value < minChannelValue) return defaultValue; // value cannot exceed maxChannelValue by design
    }
    return value;
}
