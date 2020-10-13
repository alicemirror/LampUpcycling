/**
 * Vintage lamp
 * 
 * Arduino sketch for the Vintage Lamp project, transforming a Briomvega desk lamp
 * by mid '60s to a low-power LED desk lamp controlled by Arduino.
 */

#define LIGHT_PIN 3        ///< PWM Light output (to transistor)
#define MIC_CONTROL_PIN 4   ///< mic sempling enabler
#define VOICE_SET_PIN 4    ///< When On activate the light based on voice
#define MIN_DIMMER 30      ///< Minimum analog value of the potentiometer
#define MAX_DIMMER 255      ///< Maximum analog value of the potentiometer
#define OFF_LIGHT 0       ///< Light intensity off
#define LOW_LIGHT 50       ///< Light intensity min range
#define HIGH_LIGHT 255      ///< Light intensity max tange
#define RESPONSIVITY 50     ///< Sensitivity of the dimmer readings in ms
//!  The sample window amplitude of 50 ms correspond to a frequency of 20 Hz
#define SAMPLE_FREQ 50
#define MAX_SIGNAL 1024    ///< Absolute higher value of the analog input
#define MIC_TRIGGER 400     ///< Min sampled value to trig the light

const int sampleWindow = SAMPLE_FREQ; ///< Microphone sample window width
//! Audio sample value
unsigned int sample;
unsigned int peakToPeak;    ///< peak-to-peak level
unsigned int signalMax  = 0;     ///< Max audio read value
unsigned int signalMin = MAX_SIGNAL;     ///< Min audio read value
boolean lightStatus;

//! Initialization function
void setup() {
    Serial.begin(9600);
    // Set the light level control pin
    pinMode(LIGHT_PIN, OUTPUT);
    // Set the mic switch pin
    pinMode(MIC_CONTROL_PIN, INPUT);
    // Initialize the light status
    lightStatus = false;
}

void loop() {
    int dimmer;         ///< Analog reading
    int intensity;      ///< Converted light intensity
    //! Initialize the mic samples counter
    unsigned long startMillis = millis();  

    // Read the analog value from the potentiometer slider
    dimmer = analogRead(A0);

    // ---------------------------------------------------------
    // --------- Check for the dimmer current position
    // ---------------------------------------------------------
    
    // As the analog value is between 0 and 1023, it should be
    // mapped to the limits of the min/max light PWM value
    // Before mapping we check if the values are outside the dimmer range
    // to set the off and max intensity mode without flickering
    if(dimmer <= MIN_DIMMER) {
        intensity = OFF_LIGHT;
    } else if(dimmer >= MAX_DIMMER) {
            intensity = HIGH_LIGHT;
    } else {
        intensity = map(dimmer, MIN_DIMMER, MAX_DIMMER, LOW_LIGHT, HIGH_LIGHT);
    }

    Serial.println(intensity);

    // ---------------------------------------------------------
    // --------- Sample mic cycle for light activation
    // ---------------------------------------------------------

    if(digitalRead(MIC_CONTROL_PIN) == true) {
        // collect data for 50 mS and save the max and min values
        while ( (millis() - startMillis) < sampleWindow ) {
            // Check sample
            sample = analogRead(A1);
            if (sample < MAX_SIGNAL) {
                if (sample > signalMax) {
                signalMax = sample;
                    } else if (sample < signalMin) {
                        signalMin = sample;
                    } // Smaller than las saved min
                } // Greater than last saved max
            } // while ... sample cycle
    
        // Calculate the peak-to-peak amplitude of the last sample.
        // If the amlitude reach the trigger level, the status of the light
        // is changed
        peakToPeak = abs(signalMax - signalMin);
    
        // Reset the min/max values
        signalMax  = 0;
        signalMin = MAX_SIGNAL;

        // Check if the light status should be changed
        if(peakToPeak > MIC_TRIGGER) {
            // Change the light status
            if(lightStatus == true) {
                // Bypass the intensity value read from the slider
                // and force the value to off
                lightStatus = false; // Update the status of the light
                // Update the light
                analogWrite(LIGHT_PIN, 0);
            } else {
                lightStatus = true;  // Update the status of the light
            }
        } // Mic sampling is triggered
        // Update the light status if the light is on. This is to accept
        // light intensity changes also when the mic sensor is active.
        if(lightStatus == true) {
            // Update the light
            analogWrite(LIGHT_PIN, intensity);
        }
    } // Mic sampling is active
    else {
        // Mic sampling is inactive, so the light is set to the
        // current intensity level
        analogWrite(LIGHT_PIN, intensity);
    } // Mic sampling is not active
    delay(RESPONSIVITY);
} // Loop
