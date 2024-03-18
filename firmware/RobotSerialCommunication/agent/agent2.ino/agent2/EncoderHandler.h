class EncoderHandler {
private:
    volatile unsigned long timeBeforeDebounce;
    volatile unsigned long timeAfterDebounce;
    volatile unsigned long deltaDebounce;
    volatile unsigned long startTime;
    volatile unsigned long deltaTime;
    volatile int encoder_count;

    unsigned long TIMEDEBOUNCE = 15; 

public:
    EncoderHandler(unsigned long timeDebounce) {
        this->TIMEDEBOUNCE = timeDebounce;
        this->timeBeforeDebounce = 0;
        this->timeAfterDebounce = 0;
        this->deltaDebounce = 0;
        this->startTime = 0;
        this->deltaTime = 0;
        this->encoder_count = 0;
    }

    void handleInterrupt() {
        timeBeforeDebounce = millis();
        deltaDebounce = timeBeforeDebounce - timeAfterDebounce;
        
        if (deltaDebounce > TIMEDEBOUNCE) {
            startTime = micros();
            encoder_count++;
            deltaTime = startTime - timeAfterDebounce;
        }
        timeAfterDebounce = millis();
    }

    int getCount() const {
        return encoder_count;
    }
    unsigned long getDeltaTime() const {
        return deltaTime;
    }
};
