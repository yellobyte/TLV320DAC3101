# TLV320DAC3101 Stereo Audio DAC
Arduino library for the Texas Instruments TLV320DAC3101 I2S Stereo Audio DAC. The TLV320DAC3101 chip is a low-power, highly integrated, high-performance DAC with 25 selectable digital audio processing blocks (combination of 1 x IIR, 6 x BiQuad, DRC, interpolation filter, beep generator, all user programmable) and 24-bit stereo playback. It further features a Stereo Headphone or Lineout, a 1.3W 8Ω Class-D Speaker stereo output, I2S bus audio input supporting 8kHz to 192kHz sample rates, I2C bus for configuration & control and much more.

The lib is build upon the [Adafruit TLV320 I2S](https://github.com/adafruit/Adafruit_TLV320_I2S) library and extends it with functions for audio filtering (low pass, high pass, notch, peaking EQ, bass shelf, treble shelf) using 1st order IIR and/or 2nd order BiQuad filter blocks (cascading is possible), dynamic range compression (DRC), adaptive filtering mode and **stereo speaker** output available on the TLV320DAC3101.

Integrating this library into your Arduino audio projects is easy. For detailed infos have a look at the many [examples](https://github.com/yellobyte/TLV320DAC3101/tree/main/examples) included. Further down you find code samples for utilizing IIR and BiQuad filtering in your audio projects.

## :zap: Application notes

To install the library into your **IDE** open the **Library Manager**, search for **TLV320DAC3101** and choose **install**, **Add to Project** or similar.

## :pencil: Digital audio signal processing

The theory behind IIR filters of 1st, 2nd (BiQuad) or even higher orders is complex. Calculating filter coefficients for filters of e.g. 4th order requests numerical calculations of the highest precision. To get around this, you can cascade multiple lower-order filters, such as first and second order.  

For example, a typical Butterworth low pass filter (LPF) realized with a single BiQuad (2nd order) has a Q of **0.707** (1/SQRT(2)) and decreases at −12dB/octave resp. -40dB/decade (JFYI: -6dB/octave resp. -20dB/decade for a 1st order one). 

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_LPF_2nd_Butterworth.jpg)

To get a LPF of higher order and therefore sharper filter curve you could simply cascade 2 BiQuad Butterworth LPFs. Disappointingly, your -3dB point at the LPF corner frequency fc turns into a -6dB point with the two BiQuad Butterworths cascaded and the achieved filter curve looks slightly different from the expected one. In order to regain your -3dB point at fc you simply need to set Q of the two cascaded BiQuads differently, in this case one to Q1=1/0.7654(**1.307**) and the other one to Q2=1/1.8478(**0.541**). Such 4th order LPF would decrease at −24 dB per octave.

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_LPF_4th_diffQ.jpg)

You can go further. With 3 BiQuads cascaded you create an even more aggressive LPF (or HPF). In this case you need to set the three Qs to Q1=1/0.517638(**1.932**), Q2=1/SQRT(2)(**0.7071**) and Q3=1/1.931852(**0.518**). Demonstrated in example [LPF](https://github.com/yellobyte/TLV320DAC3101/tree/main/examples/BiQuad-Low-Pass-Filter).

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_LPF_6th_diffQ.jpg)

However, BiQuads and esp. cascaded filter systems can get unstable by setting wrong coefficients, chosing the overall filter gain too high, etc. So in order to avoid this in your special audio project, you could start with a filter of lower order, bigger bandwidth (notch, EQ) and lower gain, test it to be stable and then if needed decrease bandwidth, increase gain and order (one IIR block + one BiQuad block make 3rd order, two BiQuads make 4th order, etc.).

A very good point to start from is Texas Instrument's [**COEFFICIENT-CALC — Coefficient Calculator For Digital Biquad Filters**](https://www.ti.com/tool/COEFFICIENT-CALC) with graphical user interface (GUI). It targets TI's TLV320 product series and shows the filter curves of single and/or cascaded filter blocks, lets you play with frequency, gain, bandwidth, Q and even indicates if a setting becomes unstable. It calculates all filter coefficients for you and makes it very simple to get good results quickly.

Then take the calculated filter coefficients (N0, N1 & D1 for 1st order filters and additionally N2 & D2 for 2nd order filters) and write them into the IIR/BiQuad filter block(s) with function setDACFilter().

In case you need to calculate and change the coefficients dynamically while your program is running you can first call calcDACFilterCoefficients() before writing them with setDACFilter().

Below examples show the general use of calcDACFilterCoefficients() and setDACFilter() on typical filters.

## :information_source: Code samples for audio processing with the TLV320DAC3101

### Example 1: IIR Low Pass Filter (1st order)

The TLV320DAC3101 has an IIR (1st order) low pass filter activated on both audio channels (left & right) and therefore frequencies above the -3dB corner frequency get attenuated. Such filter decreases with -6dB/octave resp. -20dB/decade.  

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_LPF_1st_Butterworth.jpg)

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

  // set fc parameter for a low pass filter
  filter.fc = 1000.0;                         // Hz, -3dB corner frequency

  // calculate filter coefficients (TI calls them N0, N1 & D1) for an IIR (1st order) filter
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ,                // audio sample rate
                                     TLV320_FILTER_TYPE_LOW_PASS,  // filter type
                                     TLV320_FILTER_IIR,            // 1st order filter
                                     &filter)) {                   // keeps filter setting
    halt("Failed to calculate IIR filter coefficients!");
  }

  // write calculated filter coefficients into IIR signal processing block
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

### Example 2: IIR High Pass Filter (1st order)


The TLV320DAC3101 has an IIR (1st order) high pass filter activated only on the left audio channel. Frequencies below the -3dB corner frequency of fc=1kHz get attenuated on that channel.  

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_HPF_1st_Butterworth.jpg)

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

  // set IIR signal processing block coefficients N0, N1, D1 manually,
  // they represent a HPF with fc=1000Hz and no gain
  filter.N0H = 0x77;
  filter.N0L = 0x78;
  filter.N1H = 0x88;
  filter.N1L = 0x88;
  filter.D1H = 0x6E;
  filter.D1L = 0xF2;

  // write filter coefficients into IIR signal processing block
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

### Example 3: BiQuad High Pass Filter (4th order)

The TLV320DAC3101 has two cascaded BiQuad (2nd order) high pass filters pro channel activated. Together they form a high pass filter of 4th order per channel, which has a much steeper filter curve than a single BiQuad filter alone. Filter Q is chosen differently to keep -3dB attenuation at fc. Picture & explanation see above.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filterA, filterB;

void setup()
{
  ...
  // set filter parameters for BiQuad filter blocks
  filterA.fc = filterB.fc = 1000.0;              // Hz, -3dB corner frequency
  filterA.Q = 1 / 0.7654;
  filterB.Q = 1 / 1.8478;

  // calculate coefficients for the two Biquad filter blocks
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_HIGH_PASS,
                                    TLV320_FILTER_BIQUAD, &filterA) ||
      !dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_HIGH_PASS,
                                    TLV320_FILTER_BIQUAD, &filterB)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // write filter coefficients into BiQuad signal processing blocks
  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_A,  // using BiQuadA filter block
                        &filterA)) {             // pointer to filter settings
    halt("Failed to set BiQuadA filter!");
  }

  if (!dac.setDACFilter(true,                    // enable filtering
                        true,                    // on left channel
                        true,                    // and on right channel
                        TLV320_FILTER_BIQUAD_B,  // using BiQuadB filter block
                        &filterB)) {             // pointer to filter settings
    halt("Failed to set BiQuadB filter!");
  }
  ...
}
```

### Example 4: BiQuad Notch Filter (2nd order)


The TLV320DAC3101 has a single BiQuad notch filters activated with a center frequency of fc=1.5kHz and a -3dB bandwidth of bw=200Hz. Therefore all frequencies near 1.5kHz get attenuated.  

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_Notch.jpg)

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 32000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // set parameters for the BiQuad filter block
  filter.fc = 1500.0;                            // Hz, notch center frequency
  filter.bw = 200.0  ;                           // Hz, -3dB filter bandwidth

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_NOTCH,
                                    TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // write calculated filter coefficients into BiQuad signal processing block
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

### Example 5: BiQuad peaking EQ Filter (2nd order)


The TLV320DAC3101 has a single BiQuad EQ filter with center frequency fc=1.5kHz, bandwidth bw=300Hz and peak gain=+10dB activated. Therefore all frequencies near 1.5kHz will get a moderate boost. If you encounter an unstable filter then increase the bandwidth or reduce the DAC gain by some amount with _cfg.dac_gain_left/right = -x.x_ in your program.

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_EQ.jpg)

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // set parameters for the BiQuad filter block
  filter.fc = 1500.0;                            // Hz, EQ center frequency
  filter.bw = 300.0;                             // Hz, EQ filter bandwidth
  filter.gain = 10.0;                            // dB, EQ filter peak gain

  // calculate BiQuad coefficients
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_EQ,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // write calculated filter coefficients into BiQuad signal processing block
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

### Example 6: BiQuad Bass Shelf Filter (4th order)

I tested with a single BiQuad & 12dB gain and encountered no problems. Even two cascaded BiQuad blocks with fc=800Hz and gain=+8dB per block worked. Though I had to reduce the DAC gain with _cfg.dac_gain_left/right = -x.x_ by some amount to get this configuration stable. The picture below points at this problem. However, with a bass shelf filter the whole frequency spectrum below fc gets a constant boost.  

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_BassShelf_4th.jpg)

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // set parameters for the two BiQuad filter blocks
  filter.fc = 800.0;                          // Hz, frequencies below 800Hz get boosted
  filter.gain = 8.0;                          // dB, filter gain below fc per BiQuad block
                                              // Note: setting the overall gain too high might cause
                                              // the filters to become unstable!

  // calculate coefficients for Biquad filter blocks
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_BASS_SHELF,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // write calculated filter coefficients into BiQuad signal processing blocks
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

### Example 7: BiQuad Treble Shelf Filter (2nd order)

A treble shelf filter of 2nd order, realized with a single BiQuad block. The whole frequency spectrum above fc gets a constant boost resp. the whole spectrum below fc gets a constant reduction.  

![](https://github.com/yellobyte/TLV320DAC3101/raw/main/doc/TIBQ_TrebleShelf_2nd.jpg)

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 48000

TLV320DAC3101 dac;
tlv320_filter_param_t filter;

void setup()
{
  ...
  // set parameters for BiQuad filter block
  filter.fc = 800.0;                        // Hz, frequencies above 800Hz get boosted
  filter.gain = 11.0;                       // dB, constant filter gain above fc 

  // calculate coefficients for Biquad filter block
  if (!dac.calcDACFilterCoefficients(SAMPLERATE_HZ, TLV320_FILTER_TYPE_TREBLE_SHELF,
                                     TLV320_FILTER_BIQUAD, &filter)) {
    halt("Failed to calculate BiQuad filter coefficients!");
  }

  // write calculated filter coefficients into BiQuadA signal processing block
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
### Example 8: Disable filter block BiQuadA on both channels

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

### Example 9: Dynamic Range Compression (DRC)

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

### Example 10: Dynamic Range Compression (DRC) with non-standard settings

The same as above but with user defined DRC settings:

```c
...
#include "TLV320DAC3101.h"

// non-standard DRC LPF/HPF coefficients as used in example in Ch. 6.3.10.4.6
uint8_t my_drc_lpf_coeffs[6] = {0x00, 0x11, 0x00, 0x11, 0x7F, 0xDE};
uint8_t my_drc_hpf_coeffs[6] = {0x7F, 0xAB, 0x80, 0x55, 0x7F, 0x56};

TLV320DAC3101 dac;
tlv320_drc_param_t drc;

void setup()
{
  ...
  // they default to standard if not set
  //drc.threshold  = TLV320_DRC_THRESHOLD_MINUS_21DB;
  //drc.hyst       = TLV320_DRC_HYST_2DB;
  drc.hold       = TLV320_DRC_HOLD_TIME_32_SAMPLES;
  drc.attack     = TLV320_DRC_ATTACK_RATE_0_00390625DB;
  drc.decay      = TLV320_DRC_DECAY_RATE_0_000488281DB;
  drc.lpf_coeffs = my_drc_lpf_coeffs;
  drc.hpf_coeffs = my_drc_hpf_coeffs;

  if (!dac.setDRC(true,                // enable DRC
                  true,                // on left channel and
                  true,                // on right channel
                  &drc)) {             // with non-standard settings
    halt("Failed to configure DRC!");
  }
  ...
}
```

### Example 11: Using the integrated Beep Generator

Only the digital signal processing block PRB_P25 can generate and forward a sine-wave to the DAC. This functionality is intended e.g. for generating key-click sounds for user feedback etc.

```c
...
#include "TLV320DAC3101.h"

#define SAMPLERATE_HZ 44100

TLV320DAC3101 dac;

void setup()
{
  ...
  // processing block PRB_P25 (RC12) contains a Beep generator
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
