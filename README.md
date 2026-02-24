# TLV320DAC3101 Stereo Audio DAC
Arduino library for the Texas Instruments TLV320DAC3101 I2S Stereo Audio DAC. The TLV320DAC3101 chip is a low-power, highly integrated, high-performance DAC with 25 selectable digital audio processing blocks (combination of 1 x IIR, 6 x BiQuad, DRC, interpolation filter, beep generator, all user programmable) and 24-bit stereo playback. It further features a Stereo Headphone or Lineout, a 1.3W 8Ω Class-D Speaker stereo output, I2S bus audio input supporting 8kHz to 192kHz sample rates, I2C bus for configuration & control and much more.

The lib is build upon the Adafruit TLV320 I2S library and extends it with functions for audio filtering (low pass, high pass, notch, peaking EQ, bass shelf, treble shelf) using 1st order IIR and/or 2nd order BiQuad filter blocks (cascading is possible), dynamic range compression (DRC), adaptive filtering mode and **stereo speaker** output available on the TLV320DAC3101.

Integrating this library into your Arduino audio projects is easy. For detailed infos have a look at the many [examples](https://github.com/yellobyte/TLV320DAC3101/tree/main/examples) included. Further down you find code samples for utilizing IIR and BiQuad filtering in your audio projects.

## :zap: Application notes

To install the library into your **IDE** open the **Library Manager**, search for **TLV320DAC3101** and choose **install**, **Add to Project** or similar.

## :information_source: Code samples for audio processing with the TLV320DAC3101

The theory behind IIR filters of 1st, 2nd (BiQuad) or even higher orders is very complex. Calculating filter coefficients for filters of e.g. 4th order requests numerical calculations of the highest precision. To get around this, you can cascade multiple lower-order filters, such as first and second order. For example, in order to get a low pass Butterworth filter of higher order and therefore sharper filter curve you can simply cascade 2 BiQuad low pass Butterworth filters.

However, your -3dB point at the LPF corner frequency fc on a single BiQuad filter curve now turns into a -6dB point with the two BiQuads cascaded and your achieved filter curve will look slightly different from the expected one. A typical low pass Butterworth filter realized with a BiQuad (2nd order) has a Q of 0.707 (1/SQRT(2)). In order to regain your -3dB point at fc you now need to set Q of the two cascaded BiQuads differently, in this case one to Q=1/0.7654 and the other one to Q 1/1.8478.

BiQuads and esp. cascaded filter systems can get unstable very quickly by setting slightly wrong coefficients, chosing the overall filter gain too high, etc. So in order to avoid this, you could start with a filter of lower order, bigger bandwidth (notch, EQ) and lower gain, test it to be stable and then if needed decrease bandwidth, increase gain and order (one IIR block + one BiQuad block makes 3rd order, two BiQuads make 4th order, etc.).

A very good point to start from is Texas Instrument's [**COEFFICIENT-CALC — Coefficient Calculator For Digital Biquad Filters**](https://www.ti.com/tool/COEFFICIENT-CALC) with graphical user interface (GUI). It targets TI's TLV320 product series and shows the filter curves of single and/or cascaded filter blocks, lets you play with frequency, gain, bandwidth, Q and even indicates if a setting becomes unstable. It calculates all filter coefficients for you and makes it very simple to get good results quickly.

Then take the calculated filter coefficients (N0, N1 & D0 for 1st order filters and additionally N2 & D1 for 2nd order filters) and program them into a IIR/BiQuad filter block with function setDACFilter().

In case you need to change the coefficients dynamically while your program is running you can call calcDACFilterCoefficients() before calling setDACFilter().

Below examples show the general use of calcDACFilterCoefficients() and setDACFilter() on typical filters.

### Example 1: IIR (1st order) Low Pass Filter

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

  // setting parameter for low pass filter
  filter.fc = 1000.0;                         // Hz, -3dB corner frequency

  // calculate filter coefficients (TI calls them N0, N1 & D1) for an IIR (1st order) filter
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ,                // audio sample rate
                                     TLV320_FILTER_TYPE_LOW_PASS,  // filter type
                                     TLV320_FILTER_IIR,            // 1st order filter
                                     &filter)) {                   // keeps filter setting
    halt("Failed to calculate IIR filter coefficients!");
  }

  // program the calculated filter coefficients into the IIR signal processing block
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

### Example 2: IIR (1st order) High Pass Filter

The TLV320DAC3101 has an IIR (1st order) high pass filter activated only on the left audio channel. Frequencies below the set corner frequency of fc=1kHz get slightly attenuated on that channel.

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

  // Setting IIR signal processing block coefficients N0, N1, D1 manually,
  // they represent fc=1000Hz, gain=0.0dB.
  filter.N0H = 0x77;
  filter.N0L = 0x78;
  filter.N1H = 0x88;
  filter.N1L = 0x88;
  filter.D1H = 0x6E;
  filter.D1L = 0xF2;

  // program the filter coefficients into the IIR signal processing block
  if (!dac.setDACFilter(true,                 // enable filtering
                        true,                 // on left channel
                        false,                // but not on right channel
                        TLV320_FILTER_IIR,    // using IIR filter block
                        &filter)) {           // pointer to filter settings
    halt("Failed to configure IIR filter!");
  }
  ...
}
```

### Example 3: BiQuad (4th order) High Pass Filter

The TLV320DAC3101 has two cascaded BiQuad (2nd order) high pass filters pro channel activated. Together they form a high pass filter of 4th order per channel, which has a much steeper filter curve than a single BiQuad filter alone. Therefore frequencies below the set corner frequency get strongly attenuated. Q is chosen differently to keep -3dB attenuation at fc.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // setting filter parameter for first BiQuad filter block
  filter.fc = 1000.0;                           // Hz, -3dB corner frequency
  filter.Q = 1 / 0.7654;

  // calculate coefficients for first Biquad filter block
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

  // changing filter parameter for second BiQuad filter block, explanation see above
  filter.Q = 1 / 1.8478;

  // calculate coefficients for second Biquad filter block
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_HIGH_PASS,
                                    TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
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

### Example 4: BiQuad (2nd order) Notch Filter

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

  // program the calculated filter coefficients into the BiQuad signal processing block
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

### Example 5: BiQuad (2nd order) peaking EQ Filter

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
  filter.bw = 200.0;                             // Hz, EQ filter bandwidth
  filter.gain = 12.0;                            // dB, EQ filter peak gain

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_EQ,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // program the calculated filter coefficients into the BiQuad signal processing block
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

### Example 6: BiQuad (4th order) Bass Shelf Filter

The TLV320DAC3101 has two cascaded BiQuad Bass Shelf filter blocks with fc=800Hz and gain=+8dB per block activated. Therefore the frequency spectrum below 800Hz will get a constant 16dB boost.

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
  filter.fc = 800.0;                             // Hz, frequencies below 800Hz get boosted
  filter.gain = 8.0;                             // dB, filter gain below fc per BiQuad block
                                                 // Note: setting the overall gain too high might cause
                                                 // the filter to become unstable!

  // calculate coefficients for Biquad filter blocks
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_BASS_SHELF,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // program the calculated filter coefficients into the BiQuad signal processing blocks
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
### Example 7: Disabling filter block BiQuadA on both channels

```c
  ...
  if (!dac.setDACFilter(false,                   // disable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        NULL)) {                 // not needed in this case
    halt("Failed to disable BiQuadA filter!");
  }
  ...
```

### Example 8: Dynamic Range Compression (DRC)

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

### Example 9: Using the integrated Beep Generator

Only the digital signal processing block PRB_P25 can generate and forward a sine-wave to the DAC. This functionality is intended e.g. for generating key-click sounds for user feedback etc.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;

void setup()
{
  ...
  // Processing block PRB_P25 (RC12) contains a Beep Generator
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
  dac.enableBeep(true);  // generate single short beep tone of 1kHz
  ...
}
```