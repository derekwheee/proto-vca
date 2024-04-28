// TODO: Swap out ADCs
// TODO: Update pin mapping

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_ADS7830.h>
#include <SAMDTimerInterrupt.h>
#include <config.h>

SAMDTimer sequenceTimer(SELECTED_TIMER);

Adafruit_ADS7830 adc;
Adafruit_DotStar pixel(NUM_LEDS, DS_DATA_PIN, DS_CLCK_PIN, DOTSTAR_BRG);

// Constants
const int ANALOG_HIGH = 1023;
const int DAC_HIGH = 4095;
const int I2C_BAUD_HIGH_SPEED = 3400000; // 3.4MHz

// Define scaling ranges
const double dacRange[2] = {0, DAC_HIGH};
const double voltageRange[2] = {0, ANALOG_HIGH};
const double inverseVoltageRange[2] = {ANALOG_HIGH, 0};
const double amplificationRange[2] = {0, 1};

struct VCA
{
    bool invertGain;
    int gainPin;
    int cvPin;
    int inputPin;
    int outputPin;
    int gain;
    int cv;
    int input;
    double output;
};

VCA vcas[] = {
    // {
    //     invertGain : true,
    //     gainPin : VCA1_GAIN_PIN,
    //     cvPin : VCA1_CV_PIN,
    //     inputPin : VCA1_INPUT_PIN,
    //     outputPin : VCA1_OUTPUT_PIN,
    //     gain : 0,
    //     cv : 0,
    //     input : 0,
    //     output : 0
    // },
    {
        invertGain : true,
        gainPin : VCA2_GAIN_PIN,
        cvPin : VCA2_CV_PIN,
        inputPin : VCA2_INPUT_PIN,
        outputPin : VCA2_OUTPUT_PIN,
        gain : 0,
        cv : 0,
        input : 0,
        output : 0
    },
};

int vcaCount = sizeof(vcas) / sizeof(vcas[0]);

float mainGainFactor;
float amplification;
void handleInterrupt();
float getAmplificationFactor(VCA vca);
uint32_t readAnalogPin(int pin);
float scale(float value, const double inputRange[2], const double outputRange[2])
{
    return (value - inputRange[0]) * (outputRange[1] - outputRange[0]) / (inputRange[1] - inputRange[0]) + outputRange[0];
}

bool ranOnce = false;

void setup()
{
    // Use high speed I2C to get the best ADC sampling rate
    Wire.setClock(I2C_BAUD_HIGH_SPEED);

    if (!adc.begin())
    {
        Serial.println("Failed to initialize ADS7830!");
        while (1)
            ;
    }

    pixel.begin();
    pixel.show();

    sequenceTimer.attachInterruptInterval_MS(0.1, handleInterrupt);

    Serial.begin(115200);
}

void loop()
{
    mainGainFactor = scale(analogRead(MAIN_GAIN_PIN), voltageRange, amplificationRange);

    for (int i = 0; i < vcaCount; i++)
    {
        VCA &vca = vcas[i];

        amplification = getAmplificationFactor(vca);
        vca.gain = readAnalogPin(vca.gainPin);
        vca.cv = readAnalogPin(vca.cvPin);
        vca.input = readAnalogPin(vca.inputPin);
        vca.output = scale(vca.input * amplification, voltageRange, dacRange);
    }

    pixel.setPixelColor(0, pixel.Color(0, 255 * vcas[0].output / DAC_HIGH, 255 * vcas[0].output / DAC_HIGH));

    pixel.show();
}

void handleInterrupt()
{
    for (int i = 0; i < vcaCount; i++)
    {
        VCA &vca = vcas[i];

        analogWrite(vca.outputPin, vca.output);
    }
}

float getAmplificationFactor(VCA vca)
{
    float vcaGainFactor = vca.invertGain ? scale(vca.gain, inverseVoltageRange, amplificationRange) : scale(vca.gain, voltageRange, amplificationRange);
    float cvFactor = scale(vca.cv, voltageRange, amplificationRange);

    return mainGainFactor * vcaGainFactor * cvFactor;
}

uint32_t readAnalogPin(int pin)
{
    u_int32_t value = 0;

    if (pin >= A0)
    {
        value = analogRead(pin);
    }
    else
    {
        value = adc.readADCsingle(pin);
    }

    return value;
}