// VCA_1 output is wired to the wrong pin

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_ADS1X15.h>

// DotStar config
#define NUM_LEDS 1
#define DS_DATA_PIN 8
#define DS_CLCK_PIN 6

#define MAIN_GAIN_PIN A2
#define VCA1_GAIN_PIN A3
#define VCA1_CV_PIN 1
#define VCA1_INPUT_PIN 2
#define VCA1_OUTPUT_PIN A0
#define VCA2_GAIN_PIN A3 // TODO: Switch this back to 0
#define VCA2_CV_PIN A4
#define VCA2_INPUT_PIN A5
#define VCA2_OUTPUT_PIN A1

// Constants
#define ANALOG_HIGH 1023.0
#define DAC_HIGH 4095.0

Adafruit_ADS1015 ads1015;
Adafruit_DotStar pixel(NUM_LEDS, DS_DATA_PIN, DS_CLCK_PIN, DOTSTAR_BRG);

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
    {
        invertGain : true,
        gainPin : VCA1_GAIN_PIN,
        cvPin : VCA1_CV_PIN,
        inputPin : VCA1_INPUT_PIN,
        outputPin : VCA1_OUTPUT_PIN,
        gain : 0,
        cv : 0,
        input : 0,
        output : 0
    },
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
float getAmplificationFactor(VCA vca);
uint32_t readAnalogPin(int pin);
float scale(float value, const double inputRange[2], const double outputRange[2])
{
    return (value - inputRange[0]) * (outputRange[1] - outputRange[0]) / (inputRange[1] - inputRange[0]) + outputRange[0];
}

bool ranOnce = false;

void setup()
{
    ads1015.begin();

    pixel.begin();
    pixel.show();

    Serial.begin(115200);
}

void loop()
{
    mainGainFactor = scale(analogRead(MAIN_GAIN_PIN), voltageRange, amplificationRange);

    for (int i = 0; i < vcaCount; i++)
    {
        VCA vca = vcas[i];
        vca.gain = readAnalogPin(vca.gainPin);
        vca.cv = readAnalogPin(vca.cvPin);
        vca.input = readAnalogPin(vca.inputPin);

        float amplification = getAmplificationFactor(vca);

        vca.output = scale(vca.input * amplification, voltageRange, dacRange);

        analogWrite(vca.outputPin, vca.output);

        pixel.setPixelColor(0, pixel.Color(0, 0, 255 * vca.output / DAC_HIGH));
    }

    pixel.show();
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
        // TODO: Figure out why this is crashing
        // value = ads1015.readADC_Differential_0_1();
    }

    return value;
}