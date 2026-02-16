/*
  BiQuad Notch Filter (2nd order filter)

  An ESP32 background thread is feeding the TLV320 with a sine tone sweep 1200Hz...1700Hz.
  The TLV320DAC3101 Stereo Audio DAC has a BiQuad notch filters activated with a center
  frequency of fc=1.5kHz and a bandwidth of bw=300Hz. Therefore all frequencies near 1.5kHz 
  will get attenuated. The audio signal is output on both the speaker and headphone socket.
  
  Processing block PRB_P1 (default) contains 3 BiQuad filter blocks (A, B, C). We configure
  and use only the BiQuadA filter section. 
  
  When the serial monitor has the input focus you can enter some keysd:
   - d...disables the filter,
   - e...enables the filter, 
   - a...toggles adaptive mode

  The following additional libraries are needed:
   - Adafruit_TLV320_I2S
   - Adafruit_BusIO

  Last updated 2026-02-16, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <ESP_I2S.h>
#include "TLV320DAC3101.h"

// ESP32-S3 I2S settings
i2s_mode_t           mode  = I2S_MODE_STD;             // Philips standard
i2s_data_bit_width_t width = I2S_DATA_BIT_WIDTH_16BIT; // 16bit data/sample width
i2s_slot_mode_t      slot  = I2S_SLOT_MODE_STEREO;     // 2 slots (stereo)

// audio definitions
#define SAMPLERATE_HZ 44100        // audio sample rate (e.g. 32000, 44100, 48000)
#define FREQU_MAX     1700         // highest generated frequency
#define FREQU_MIN     1200         // lowest generated frequency
#define FREQU_DELTA   4            // Hz, frequency step
#define INTERVAL      150          // ms
#define FREQU_C       1500         // Hz, center frequency of notch filter
#define FREQU_BW      200          // Hz, -3dB bandwidth

float amplitude = ((1<<14)-1);     // amplitude of generated waveform
int32_t frequency = FREQU_MIN,     // start frequency of generated waveform
        maxSamples = (int32_t)(SAMPLERATE_HZ / 1000.0 * INTERVAL),
        fdelta = FREQU_DELTA;

// for pre-calculation of sine waveform in memory
#define WAV_SIZE      8192         // size/points of generated waveform
int16_t waveform[WAV_SIZE] = {0};

I2SClass  i2s;
TLV320DAC3101 dac;
tlv320_filter_cfg_t filter;  // keeps the filter settings

// Background task continuously feeding I2S bus with sine tone sweep
void backgroundTask(void *parameter) {
  uint16_t pos = 0, delta;
  while (true) {
    // give some visual feedback about actual frequency
    Serial.printf("frequency=%.0luHz\n", frequency);

    // generate sine tone sweep
    delta = (int16_t)(frequency * (float)WAV_SIZE / (float)SAMPLERATE_HZ);
    for (uint32_t i = 0; i < maxSamples; ++i) {
      pos = uint16_t(pos + delta) % (uint16_t)WAV_SIZE;
      int16_t sample = waveform[pos];
      // left channel, low 8 bits first
      i2s.write((uint8_t)sample);
      i2s.write((uint8_t)(sample >> 8));
      // right channel, low 8 bits first
      i2s.write((uint8_t)sample);
      i2s.write((uint8_t)(sample >> 8));
    }
    frequency += fdelta;           // set new frequency
    if (frequency <= FREQU_MIN || frequency >= FREQU_MAX) {
      fdelta = -fdelta;            // reverse direction
    }
  }
  vTaskDelete(NULL);               // will never get here  
}

// helper function to halt on critical errors
void halt(const char *message) {
  Serial.println(message);
  while (true) yield();
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("\nrunning example \"BiQuad Notch Filter\":");

  // generate a sine wave signal with defined amplitude in RAM buffer
  for (int i = 0; i < WAV_SIZE; ++i) {
    waveform[i] = int16_t(amplitude * sin(2.0 * PI * ((1.0 / WAV_SIZE) * i)));
  }
  Serial.println("Sine table generated.");

  // TLV320DAC3101 Audio DAC initialization
  pinMode(TLV_RESET, OUTPUT);
  digitalWrite(TLV_RESET, LOW);    // resets the DAC chip
  delay(100);
  digitalWrite(TLV_RESET, HIGH);

  Serial.println("Init TLV320 DAC");
  if (!dac.begin()) {
    halt("Failed to initialize codec!");
  }

  // I2S Interface Control
  if (!dac.setCodecInterface(TLV320DAC3100_FORMAT_I2S,       // Format: I2S (Philips standard)
                             TLV320DAC3100_DATA_LEN_16)) {   // Length: 16 bits
    halt("Failed to configure codec interface!");
  }

  // Clock MUX and PLL settings
  if (!dac.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) || // PLL output feeds Codec
      !dac.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK)) {    // BCLK feeds PLL input
    halt("Failed to configure codec clocks!");
  }

  if (!dac.setPLLValues(1, 2, 32, 0)) {       // Configure PLL dividers P, R, J and D
    halt("Failed to configure PLL values!");
  }

  if (!dac.setNDAC(true, 4) ||                // Configure DAC dividers NDAC, MDAC and DOSR
      !dac.setMDAC(true, 4) ||
      !dac.setDOSR(128)) {
    Serial.println("Failed to configure DAC dividers!");
  }

  if (!dac.powerPLL(true)) {                  // Power up the PLL
    halt("Failed to power up PLL!");
  }

  // setting parameters for BiQuadA filter
  filter.enabled = true;                            // enable filter
  filter.section = TLV320_FILTER_SECTION_BIQUAD_A;  // BiQuadA (2nd order filter)
  filter.type = TLV320_FILTER_TYPE_NOTCH;           // notch filter
  filter.channel = TLV320_FILTER_CHAN_ALL;          // left and right channel
  filter.fs = (float)SAMPLERATE_HZ;                 // sample rate
  filter.fc = (float)FREQU_C;                       // center frequency
  filter.bw = (float)FREQU_BW;                      // -3dB bandwidth
  // filter coefficients can be set manually or with this library as shown further down
  // filter.N0H = 0x7F,
  // filter.N0L = 0xFF,
  // etc.

  if (!dac.calculateDACFilterCoeffs(&filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  if (!dac.setDACFilter(&filter)) {
    halt("Failed to configure BiQuadA!");
  }

  // Configure DAC path - now power up both left and right DACs
  if (!dac.setDACDataPath(true, true,                      // Power up both DACs
                          TLV320_DAC_PATH_NORMAL,          // Normal left path
                          TLV320_DAC_PATH_NORMAL,          // Normal right path
                          TLV320_VOLUME_STEP_1SAMPLE)) {   // Step: 1 per sample
    halt("Failed to configure DAC data path!");
  }

  // Route DAC output to headphone
  if (!dac.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER,   // Left DAC to mixer
                                 TLV320_DAC_ROUTE_MIXER,   // Right DAC to mixer
                                 false, false, false,      // No AIN routing
                                 false)) {                 // No HPL->HPR
    halt("Failed to configure DAC routing!");
  }
                                
  // DAC Volume Control
  if (!dac.setDACVolumeControl(
        false, false, TLV320_VOL_INDEPENDENT) ||   // Unmute both channels
      !dac.setChannelVolume(false, 0) ||           // Left DAC +0dB
      !dac.setChannelVolume(true, 0)) {            // Right DAC +0dB
    halt("Failed to configure DAC volumes!");
  }

  // Headphone & Speaker Setup
  if (!dac.configureHeadphoneDriver(
        true, true,                           // Power up both drivers
        TLV320_HP_COMMON_1_50V,               // Default common mode
        false) ||                             // Don't power down on SCD
      !dac.configureHPL_PGA(0, true) ||       // Set HPL gain (0-9dB), unmute
      !dac.configureHPR_PGA(0, true) ||       // Set HPR gain (0-9dB), unmute
      !dac.setHPLVolume(true, 20) ||          // Enable and set HPL volume: -10dB
      !dac.setHPRVolume(true, 20)) {          // Enable and set HPR volume: -10dB
    halt("Failed to configure headphone outputs!");
  }

  if (!dac.enableSpeaker(true) ||                 // Disable/Enable speaker amps
      !dac.configureSPK_PGA(TLV320_SPK_GAIN_6DB,  // Set gain to 6dB
                            true) ||              // Unmute
      !dac.setSPKVolume(true, 20)) {              // Enable and set volume to -10dB
    halt("Failed to configure speaker output!");
  }
  Serial.println("TLV320 DAC config done!");

  // I2S bus initialization
  i2s.setPins(I2S_BCLK, I2S_LRCLK, I2S_DOUT);
  if (!i2s.begin(mode, (uint32_t)SAMPLERATE_HZ, width, slot)) {
    halt("Failed to initialize I2S!");
  }
  Serial.println("I2S initialization done!");

  xTaskCreate(backgroundTask, "bgTask", 4096, NULL, 1, NULL);
  delay(100);
  
  // to avoid trouble only activate adaptive mode with I2S bus active
  dac.setAdaptiveMode(true);
}

void loop() {
  char buf[10];

  // check for serial input and perform the requested action
  if (Serial.read(buf, sizeof(buf))) {
    if (*buf == 'e') {                        // enable filtering
      Serial.println("---> enable notch filter");
      filter.enabled = true;
      if (!dac.setDACFilter(&filter)) {
        halt("Failed to enable BiQuadA filter!");
      }
    }                       
    else if (*buf == 'd') {                   // disable filtering
      Serial.println("---> disable notch filter");
      filter.enabled = false;
      if (!dac.setDACFilter(&filter)) {
        halt("Failed to disable BiQuadA filter!");
      }  
    }
    else if (*buf == 'a') {                   // toggle adaptive mode
      dac.setAdaptiveMode(!dac.getAdaptiveMode());
      Serial.printf("adaptive mode is switched %s\n", dac.getAdaptiveMode() ? "on" : "off");
    }
  }
  delay(10);
}
