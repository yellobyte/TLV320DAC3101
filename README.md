# TLV320DAC3101
Arduino library for the Texas Instruments TLV320DAC3101 I2S Stereo Audio DAC. The TLV320DAC3101 chip is a low-power, highly integrated, high-performance DAC with 25 selectable digital audio processing blocks (combination of 1 x IIR, 6 x BiQuad, DRC, interpolation filter, beep generator, all user programmable) and 24-bit stereo playback. It further features a Stereo Headphone or Lineout, a 1.3W 8Ω Class-D Speaker stereo output, I2S bus audio input supporting 8kHz to 192kHz sample rates, I2C bus for configuration & control and much more.

The lib is build upon the Adafruit TLV320 I2S library and extends it with functions for audio filtering (low pass, high pass, notch, peaking EQ, bass shelf, treble shelf) using 1st order IIR and/or 2nd order BiQuad filter blocks (cascading is possible), dynamic range compression (DRC), adaptive filtering mode and **stereo speaker** output available on the TLV320DAC3101.

Integrating this library into your Arduino audio projects is easy. For detailed infos have a look at the many [examples](https://github.com/yellobyte/TLV320DAC3101/tree/main/examples) included. Further down you find code samples for utilizing IIR and BiQuad filtering in your audio projects.

## :zap: Application notes

To install the library into your **IDE** open the **Library Manager**, search for **TLV320DAC3101** and choose **install**, **Add to Project** or similar.

## :information_source: Code samples for audio processing with the TLV320DAC3101 Stereo Audio DAC

### IIR (1st order) Low Pass Filter:

The TLV320DAC3101 has an IIR (1st order) low pass filter activated on both audio channels (left & right) and therefore frequencies above the set corner frequency get slightly attenuated.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // PRB_P3 (RC10) contains IIR filtering option
  if (!dac.setDACProcessingBlock(3)) {
    halt("Failed to configure Processing Block!");
  }

  // setting parameters for an IIR low pass filter
  filter.fc = 1000;                           // Hz, -3dB corner frequency

  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_LOW_PASS,
                                     TLV320_FILTER_IIR, &filter)) {
    halt("Failed to calculate IIR filter coefficients!");
  }

  if (!dac.setDACFilter(true,                 // enable filtering
                        true,                 // on left channel
                        true,                 // and on right channel
                        TLV320_FILTER_IIR,    // using IIR filter block
                        &filter)) {           // pointer to filter settings
    halt("Failed to configure IIR filter!");
  }
  ...
}
```

### IIR (1st order) High Pass Filter:

The TLV320DAC3101 has an IIR (1st order) high pass filter activated on both audio channels (left & right) and therefore frequencies below a set corner frequency get slightly attenuated. User defined filter coefficients get applied.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // PRB_P3 (RC10) contains IIR filtering option
  if (!dac.setDACProcessingBlock(3)) {
    halt("Failed to configure Processing Block!");
  }

  // setting user defined IIR filter coefficients N0, N1, D1
  filter.N0H = 0x77;
  filter.N0L = 0x78;
  filter.N1H = 0x88;
  filter.N1L = 0x88;
  filter.D1H = 0x6E;
  filter.D1L = 0xF2;

  if (!dac.setDACFilter(true,                 // enable filtering
                        true,                 // on left channel
                        true,                 // and on right channel
                        TLV320_FILTER_IIR,    // using IIR filter block
                        &filter)) {           // pointer to filter settings
    halt("Failed to configure IIR filter!");
  }
  ...
}
```

### BiQuad (4th order) High Pass Filter:

The TLV320DAC3101 has two cascaded BiQuad (2nd order) high pass filters pro channel activated. Together they form a high pass filter of 4th order per channel, which has a much steeper filter curve than a single BiQuad filter alone. Therefore frequencies below the set corner frequency get strongly attenuated.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // setting parameters for BiQuad filter blocks
  filter.fc = 1500.0;                         // Hz, -3dB corner frequency
  filter.gain = 1.0;                          // dB, filter gain per block

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_HIGH_PASS,
                                    TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadA filter!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_B,  // using BiQuadB filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadB filter!");
  }
  ...
}
```

### BiQuad (2nd order) Notch Filter:

The TLV320DAC3101 has a single BiQuad notch filters activated with a center frequency of fc=1.5kHz and a -3dB bandwidth of bw=300Hz. Therefore all frequencies near 1.5kHz will get attenuated.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 32000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // setting parameters for the BiQuad filter block
  filter.fc = 1500.0;                            // Hz, notch center frequency
  filter.bw = 200.0  ;                           // Hz, -3dB filter bandwidth

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_NOTCH,
                                    TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadA filter!");
  }
  ...
}
```

### BiQuad (2nd order) peaking EQ Filter:

The TLV320DAC3101 has a single BiQuad EQ filter with center frequency fc=1.5kHz, bandwidth bw=200Hz and gain=+12dB activated. Therefore all frequencies near 1.5kHz will get a moderate boost.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // setting parameters for the BiQuad filter block
  filter.fc = 1500.0;                            // Hz, EQ center frequency
  filter.bw = 200.0  ;                           // Hz, -3dB filter bandwidth
  filter.gain = 12.0   ;                         // dB, EQ filter gain

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_EQ,
                                    TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadA filter!");
  }
  ...
}
```

### BiQuad (4th order) Bass Shelf Filter:

The TLV320DAC3101 has two cascaded BiQuad Bass Shelf filter blocks with fc=1.5kHz and gain=+10dB per block activated. Therefore the frequency spectrum below 800Hz will get a flat 20dB boost.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // setting parameters for BiQuad filter blocks
  filter.fc = 800.0;                             // Hz, -3dB center frequency
  filter.gain = 10.0;                            // dB, filter gain below fc per BiQuad block
                                                 // Note: setting the overall gain too high might cause
                                                 // the filter to become unstable!

  // calculate coefficients for Biquad filter blocks
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_BASS_SHELF,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadA filter!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_B,  // using BiQuadB filter block
                        &filter)) {              // pointer to filter settings
    halt("Failed to set BiQuadB filter!");
  }
  ...
}
```

### Dynamic Range Compression (DRC):

An activated DRC continuously monitors the output of the DAC. If a peaking signal is detected, the Audio DAC autonomously reduces the applied gain to avoid hard clipping. Special user settings can be given, however, below code sample would use recommended standard DRC settings.

```c
...
#include "TLV320DAC3101.h"

TLV320DAC3101 dac;

void setup()
{
  ...
  if (!dac.setDRC(true,                // enable DRC
                  true,                // on left channel and
                  true,                // on right channel
                  NULL)) {             // with recommended standard DRC settings
    halt("Failed to configure DRC!");
  }
  ...
}
```

### Using the integrated Beep Generator:

The digital signal processing block PRB_P25 can generate and forward a sine-wave to the DAC. This functionality is intended e.g. for generating key-click sounds for user feedback.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;

void setup()
{
  ...
  // Only processing block PRB_P25 (RC12) contains the Beep Generator
  if (!dac.setDACProcessingBlock(25)) {
    halt("Failed to configure processing block!");
  }

  // Beep generator settings: fBeep = 1000Hz, duration = 100ms
  if (!dac.configureBeepTone(1000.0, 100, SAMPLERATE_HZ)) {
    halt("Failed to configure beep tone!");
  }

  if (!dac.setBeepVolume(-10, -10)) {        // setting beep volume to L/R -10dB
    halt("Failed to configure beep volume!");
  }
  ...
}

void loop()
{
  ...
  dac.enableBeep(true);  // generate single beep tone
  ...
}
```