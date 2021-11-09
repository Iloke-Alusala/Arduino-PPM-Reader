#ifndef PPM_READER
#define PPM_READER

#include <Arduino.h>

class PPMReader {

    public:
    
    // The range of a channel's possible values
    unsigned minChannelValue = 1000;
    unsigned maxChannelValue = 2000;
    int mrk;
    int m1,m2, s1, s2;

    /* The maximum error (in either direction) in channel value
     * with which the channel value is still considered valid */
    unsigned channelValueMaxError = 10;

    /* The minimum value (time) after which the signal frame is considered to
     * be finished and we can start to expect a new signal frame. */
    unsigned blankTime = 2100;
	
	unsigned long failsafeTimeout = 500000L;


    private:

    // The pin from which to listen for interrupts
    byte interruptPin = 0;

    // The amount of channels to be expected from the PPM signal.
    byte channelAmount = 0;

    // Arrays for keeping track of channel values
    volatile unsigned *rawValues = NULL;
    volatile unsigned *validValues = NULL;

    // A counter variable for determining which channel is being read next
    volatile byte pulseCounter = 0;

    // A time variable to remember when the last pulse was read
    volatile unsigned long microsAtLastPulse = 0;

    // Pointer to PPMReader object used by ISR. Replace by an array if multiple PPM reader instances are needed
    static PPMReader *ppm;

    public:

    PPMReader(byte interruptPin, byte channelAmount);
    ~PPMReader(void);

    /* Returns the latest raw (not necessarily valid) value for the
     * channel (starting from 1). */
    unsigned rawChannelValue(byte channel);

    /* Returns the latest received value that was considered valid for the channel (starting from 1).
     * Returns defaultValue if the given channel hasn't received any valid values yet. */
    unsigned latestValidChannelValue(byte channel, unsigned defaultValue);

    private:

    // An interrupt service routine for handling the interrupts activated by PPM pulses
    void handleInterrupt(void);

    // Interrupt service routine function compatible with attachInterrupt. Add more funcitons if multiple PPM reader instances are needed
    static void PPM_ISR(void);

};

#endif
