/*
  Calculation of TLV320DAC3101 IIR & BiQuad Filter Coefficients for various 
  sample frequencies and filter types.

  The following libraries are needed:
   - Adafruit_TLV320_I2S
   - Adafruit_BusIO   

  Last updated 2026-02-14, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include "TLV320DAC3101.h"

TLV320DAC3101 dac;

void halt(const char *message) {
  Serial.println(message);
  while (true) yield();       // halt on critical errors
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("running example \"Calculate TLV320DAC3101 Filter Coefficients\":");
  Serial.println();

  // setting filter parameters for a 1st order low pass filter
  tlv320_filter_cfg_t lpf;
  lpf.section = TLV320_FILTER_SECTION_IIR;  // IIR (1st order)
  lpf.type = TLV320_FILTER_TYPE_LOW_PASS;   // filter type low pass
  lpf.fs = (float)44100;                    // Hz, sample rate
  lpf.gain = -3.0;                          // dB, gain
  
  Serial.println();
  Serial.printf("1st order Low Pass Filter Coefficients for fs=%.0fHz and gain=%.1fdB:\n", 
                lpf.fs, lpf.gain);
  Serial.printf("fc       N0      N1      N2      D1      D2\n");
  for (float fc = 100.0; fc <= lpf.fs / 2 - 100; fc += 100.0) {
    lpf.fc = fc;
    if (!dac.calculateDACFilterCoeffs(&lpf)) {
      halt("Calculating filter coefficients failed! Please check your chosen filter parameters.");
    }
    Serial.printf("%5.0fHz  0x%02x%02x  0x%02x%02x  na      0x%02x%02x  na\n", 
                  fc, lpf.N0H, lpf.N0L, lpf.N1H, lpf.N1L, lpf.D1H, lpf.D1L);                  
  }

  // setting filter parameters for a 2nd order notch filter
  tlv320_filter_cfg_t notch;
  notch.section = TLV320_FILTER_SECTION_BIQUAD_A;  // BiQuad (2nd order filter)
  notch.type = TLV320_FILTER_TYPE_NOTCH;           // filter type notch
  notch.fs = (float)48000;                         // Hz, sample rate
  notch.bw = (float)200;                           // Hz, bandwidth
  notch.gain = 2.0;                                // dB, gain
  
  Serial.println();
  Serial.printf("2nd order Notch Filter Coefficients for fs=%.0fHz, bandwidth=%.0fHz and gain=%.1fdB:\n", 
                notch.fs, notch.bw, notch.gain);
  Serial.printf("fc       N0      N1      N2      D1      D2\n");
  for (float fc = 100.0; fc <= notch.fs / 2 - 100; fc += 100.0) {
    notch.fc = fc;
    if (!dac.calculateDACFilterCoeffs(&notch)) {
      halt("Calculating filter coefficients failed! Please check your chosen filter parameters.");
    }
    Serial.printf("%5.0fHz  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x\n", 
                  fc, notch.N0H, notch.N0L, notch.N1H, notch.N1L, notch.N2H, notch.N2L, 
                  notch.D1H, notch.D1L, notch.D2H, notch.D2L);
  }

  // setting filter parameters for a 2nd order peakingEQ filter
  tlv320_filter_cfg_t eq;
  eq.section = TLV320_FILTER_SECTION_BIQUAD_A;  // BiQuad (2nd order filter)
  eq.type = TLV320_FILTER_TYPE_EQ;              // filter type
  eq.fs = (float)48000;                         // Hz, sample rate
  eq.bw = (float)800;                           // Hz, bandwidth
  eq.gain = 10.0;                               // dB, gain
  
  Serial.println();
  Serial.printf("2nd order EQ Filter Coefficients for fs=%.0fHz, bandwidth=%.0fHz and gain=%.1fdB:\n", 
                eq.fs, eq.bw, eq.gain);
  Serial.printf("fc       N0      N1      N2      D1      D2\n");
  for (float fc = 100.0; fc <= eq.fs / 2 - 100; fc += 100.0) {
    eq.fc = fc;
    if (!dac.calculateDACFilterCoeffs(&eq)) {
      halt("Calculating filter coefficients failed! Please check your chosen filter parameters.");
    }
    Serial.printf("%5.0fHz  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x\n", 
                  fc, eq.N0H, eq.N0L, eq.N1H, eq.N1L, eq.N2H, eq.N2L, 
                  eq.D1H, eq.D1L, eq.D2H, eq.D2L);
  }  

  // setting filter parameters for a treble shelf filter
  tlv320_filter_cfg_t shelf;
  shelf.section = TLV320_FILTER_SECTION_BIQUAD_A;  // BiQuad (2nd order filter)
  shelf.type = TLV320_FILTER_TYPE_TREBLE_SHELF;    // filter type
  shelf.fs = (float)44100;                         // Hz, sample rate
  shelf.gain = 8.0;                                // dB, gain
  
  Serial.println();
  Serial.printf("Treble Shelf Filter Coefficients for fs=%.0fHz and gain=%.1fdB:\n", 
                shelf.fs, shelf.gain);
  Serial.printf("fc       N0      N1      N2      D1      D2\n");
  for (float fc = 100.0; fc <= shelf.fs / 2 - 100; fc += 100.0) {
    shelf.fc = fc;
    if (!dac.calculateDACFilterCoeffs(&shelf)) {
      halt("Calculating filter coefficients failed! Please check your chosen filter parameters.");
    }
    Serial.printf("%5.0fHz  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x\n", 
                  fc, shelf.N0H, shelf.N0L, shelf.N1H, shelf.N1L, shelf.N2H, shelf.N2L, 
                  shelf.D1H, shelf.D1L, shelf.D2H, shelf.D2L);
  }

  Serial.println("Finished.");
}

void loop() {
  //
}

 

 
