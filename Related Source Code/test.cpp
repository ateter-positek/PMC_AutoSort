#include <Arduino.h>
#include <Arduino_MachineControl.h>
#include "Wire.h"
#include "mbed.h"
#include <chrono>
#include "Wire.h"
#include <math.h>


using namespace machinecontrol;

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

// Create a DigitalInputs object to hold the current state
DigitalInputs digitalInputs;

// Create a DigitalInputs object to hold the raw state
DigitalInputs rawDigitalInputs;

// Create a DigitalInputs object to hold the last state
DigitalInputs lastDigitalInputs;

// Create an array to hold the last debounce time for each channel
unsigned long lastDebounceTime[8] = {0};

// Create a flag to indicate when the digital inputs have changed
volatile bool digitalInputsChanged = false;

// Create a mutex for the digitalInputs object and the digitalInputsChanged flag
rtos::Mutex digitalInputsMutex;

volatile bool interruptOccured = false;

void inputCallback()
{
  interruptOccured = true;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Wire.begin();

  attachInterrupt(PB_4, inputCallback, FALLING);

  if (!digital_inputs.init()) {
    Serial.println("Digital input GPIO expander initialization fail!");
  }
}

void loop() {
  if(interruptOccured)
  {
    interruptOccured = false;

    // Reading digital input resets the interrupt flag on the IOExpander
    uint32_t inputs = digital_inputs.readAll();

    // Update the rawDigitalInputs object
    rawDigitalInputs.ch0 = (inputs & (1 << DIN_READ_CH_PIN_00)) >> DIN_READ_CH_PIN_00;
    rawDigitalInputs.ch1 = (inputs & (1 << DIN_READ_CH_PIN_01)) >> DIN_READ_CH_PIN_01;
    rawDigitalInputs.ch2 = (inputs & (1 << DIN_READ_CH_PIN_02)) >> DIN_READ_CH_PIN_02;
    rawDigitalInputs.ch3 = (inputs & (1 << DIN_READ_CH_PIN_03)) >> DIN_READ_CH_PIN_03;
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

  delay(1);
}
