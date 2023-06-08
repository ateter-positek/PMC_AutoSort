#include <Arduino.h>
#include <Arduino_MachineControl.h>
#include "Wire.h"
#include "mbed.h"
#include <chrono>
#include "Wire.h"
#include <math.h>

using namespace std::chrono;
using namespace machinecontrol;
using namespace rtos;

// Define a structure to hold the state of each encoder
struct EncoderState {
  long count;
  long revolutions;
  bool changed;
};
#define DEBOUNCE_DELAY 50  // debounce time in milliseconds
// Define a structure to hold the state of each digital input
struct DigitalInputs {
  bool ch0;
  bool ch1;
  bool ch2;
  bool ch3;
  bool ch4;
  bool ch5;
  bool ch6;
  bool ch7;
};
DigitalInputs digitalInputs;
DigitalInputs rawDigitalInputs;
DigitalInputs lastDigitalInputs;
unsigned long lastDebounceTime[8] = {0};
volatile bool digitalInputsChanged = false;
rtos::Mutex digitalInputsMutex;
volatile bool interruptOccured = false;

void inputCallback()
{
  interruptOccured = true;
}


// Encoders
const int numEncoders = 2;
EncoderState encoderStates[numEncoders] = {0};
#define ENCODER_UPDATE_COUNTS 100

// // Encoder objects
// Encoder encoders[numEncoders] = {
//   Encoder(2, 3),  // Encoder 1 connected to pins 2 and 3
//   Encoder(4, 5)   // Encoder 2 connected to pins 4 and 5
// };

// Mutex for the encoderStates object
rtos::Mutex encoderStatesMutex;

// RTOS threads for reading encoders and digital inputs
Thread threadEncoders;
Thread threadDigitalInputs;

void checkEncoder() {
  while (true) {
    for (int i = 0; i < numEncoders; i++)
    {
    if (abs(machinecontrol::encoders[i].getPulses() - encoderStates[i].count) >= ENCODER_UPDATE_COUNTS) {
      // The count has changed by the specified amount
      encoderStatesMutex.lock();
      encoderStates[i].count = machinecontrol::encoders[i].getPulses(); //count;
      encoderStates[i].revolutions = machinecontrol::encoders[i].getRevolutions();
      encoderStates[i].changed = true;
      encoderStatesMutex.unlock();

      // Handle the changed encoder value
      // For example, print the new count and revolutions
      Serial.print("Encoder ");
      Serial.print(i);
      Serial.print(": count = ");
      Serial.print(encoderStates[i].count);
      Serial.print(", revolutions = ");
      Serial.println(encoderStates[i].revolutions);
    }
    }
    ThisThread::sleep_for(5ms);
  }
}

void checkDigitalInputs() {
  while (true) {
    if(interruptOccured)
    {
      interruptOccured = false;

      // Reading digital input resets the interrupt flag on the IOExpander
      uint32_t inputs = digital_inputs.readAll();

      // Update the rawDigitalInputs object
      rawDigitalInputs.ch0 = (inputs & (1 << DIN_READ_CH_PIN_00)) >> DIN_READ_CH_PIN_00;
      rawDigitalInputs.ch1 = (inputs & (1 << DIN_READ_CH_PIN_01)) >> DIN_READ_CH_PIN_01;
      rawDigitalInputs.ch2 = (inputs & (1 << DIN_READ_CH_PIN_02)) >> DIN_READ_CH_PIN_02;
      rawDigitalInputs.ch3 = (inputs & (1 << DIN_READ_CH_PIN_03))>> DIN_READ_CH_PIN_03;
      rawDigitalInputs.ch4 = (inputs & (1 << DIN_READ_CH_PIN_04)) >> DIN_READ_CH_PIN_04;
      rawDigitalInputs.ch5 = (inputs & (1 << DIN_READ_CH_PIN_05)) >> DIN_READ_CH_PIN_05;
      rawDigitalInputs.ch6 = (inputs & (1 << DIN_READ_CH_PIN_06)) >> DIN_READ_CH_PIN_06;
      rawDigitalInputs.ch7 = (inputs & (1 << DIN_READ_CH_PIN_07)) >> DIN_READ_CH_PIN_07;

      // Debounce the inputs
      for (int i = 0; i < 8; i++) {
        bool rawState = ((&rawDigitalInputs.ch0)[i]);
        bool lastState = ((&lastDigitalInputs.ch0)[i]);
        if (rawState != lastState) {
          lastDebounceTime[i] = millis();
        }
        if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
          bool debouncedState = ((&digitalInputs.ch0)[i]);
          if (rawState != debouncedState) {
            // The input has changed state
            ((&digitalInputs.ch0)[i]) = rawState;
            digitalInputsChanged = true;
          }
        }
        ((&lastDigitalInputs.ch0)[i]) = rawState;
      }
    }
    ThisThread::sleep_for(1ms);
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Wire.begin();

  attachInterrupt(PB_4, inputCallback, FALLING);

  if (!digital_inputs.init()) {
    Serial.println("Digital input GPIO expander initialization fail!");
  }

  // Start the RTOS threads

 threadEncoders.start(checkEncoder);
 threadDigitalInputs.start(checkDigitalInputs);
}

void loop() {
  // Empty loop
}
