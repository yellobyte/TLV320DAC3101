/*
  Audio processing with the Texas Instruments TLV320DAC3101 Audio Stereo DAC.

  The lib is build upon the Adafruit TLV320 I2S library and extends it with routines 
  for filtering (high pass, low pass, notch, EQ etc. using IIR and/or BiQuad filters), 
  dynamic range control DRC, adaptive filtering mode and stereo speaker output 
  for the TLV320DAC3101.

  It also provides a print function which should make debugging code for the rather 
  complex TLV320DAC3101 chip a bit easier.

  Copyright (c) 2025 Thomas Jentzsch

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation 
  files (the "Software"), to deal in the Software without restriction, 
  including without limitation the rights to use, copy, modify, merge, 
  publish, distribute, sublicense, and/or sell copies of the Software, 
  and to permit persons to whom the Software is furnished to do so, 
  subject to the following conditions:

  The above copyright notice and this permission notice shall be 
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR 
  ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef TLV320DAC3101_H
#define TLV320DAC3101_H

#include <Adafruit_TLV320DAC3100.h>

// Page 0 
#define TLV320DAC3100_REG_DRC_CONTROL_1 0x44 // DRC Control 1 Register 
#define TLV320DAC3100_REG_DRC_CONTROL_2 0x45 // DRC Control 2 Register 
#define TLV320DAC3100_REG_DRC_CONTROL_3 0x46 // DRC Control 3 Register 
// Pages 8/12 DAC Coefficient-RAM A/B
#define TLV320DAC3100_REG_DAC_CRAM_CTRL 0x01 // DAC C-RAM Control Register
#define TLV320DAC3100_REG_DAC_LBQA_N0H  0x02 // DAC Left BiQuadA Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_LBQA_N0L  0x03 // DAC Left BiQuadA Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_LBQA_N1H  0x04 // DAC Left BiQuadA Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQA_N1L  0x05 // DAC Left BiQuadA Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQA_N2H  0x06 // DAC Left BiQuadA Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQA_N2L  0x07 // DAC Left BiQuadA Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_LBQA_D1H  0x08 // DAC Left BiQuadA Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQA_D1L  0x09 // DAC Left BiQuadA Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQA_D2H  0x0A // DAC Left BiQuadA Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQA_D2L  0x0B // DAC Left BiQuadA Coeff D2 (7:0)

#define TLV320DAC3100_REG_DAC_LBQB_N0H  0x0C // DAC Left BiQuadB Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_LBQB_N0L  0x0D // DAC Left BiQuadB Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_LBQB_N1H  0x0E // DAC Left BiQuadB Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQB_N1L  0x0F // DAC Left BiQuadB Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQB_N2H  0x10 // DAC Left BiQuadB Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQB_N2L  0x11 // DAC Left BiQuadB Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_LBQB_D1H  0x12 // DAC Left BiQuadB Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQB_D1L  0x13 // DAC Left BiQuadB Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQB_D2H  0x14 // DAC Left BiQuadB Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQB_D2L  0x15 // DAC Left BiQuadB Coeff D2 (7:0)

#define TLV320DAC3100_REG_DAC_LBQC_N0H  0x16 // DAC Left BiQuadC Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_LBQC_N0L  0x17 // DAC Left BiQuadC Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_LBQC_N1H  0x18 // DAC Left BiQuadC Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQC_N1L  0x19 // DAC Left BiQuadC Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQC_N2H  0x1A // DAC Left BiQuadC Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQC_N2L  0x1B // DAC Left BiQuadC Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_LBQC_D1H  0x1C // DAC Left BiQuadC Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_LBQC_D1L  0x1D // DAC Left BiQuadC Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_LBQC_D2H  0x1E // DAC Left BiQuadC Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_LBQC_D2L  0x1F // DAC Left BiQuadC Coeff D2 (7:0)

#define TLV320DAC3100_REG_DAC_RBQA_N0H  0x42 // DAC Right BiQuadA Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_RBQA_N0L  0x43 // DAC Right BiQuadA Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_RBQA_N1H  0x44 // DAC Right BiQuadA Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQA_N1L  0x45 // DAC Right BiQuadA Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQA_N2H  0x46 // DAC Right BiQuadA Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQA_N2L  0x47 // DAC Right BiQuadA Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_RBQA_D1H  0x48 // DAC Right BiQuadA Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQA_D1L  0x49 // DAC Right BiQuadA Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQA_D2H  0x4A // DAC Right BiQuadA Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQA_D2L  0x4B // DAC Right BiQuadA Coeff D2 (7:0)

#define TLV320DAC3100_REG_DAC_RBQB_N0H  0x4C // DAC Right BiQuadB Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_RBQB_N0L  0x4D // DAC Right BiQuadB Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_RBQB_N1H  0x4E // DAC Right BiQuadB Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQB_N1L  0x4F // DAC Right BiQuadB Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQB_N2H  0x50 // DAC Right BiQuadB Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQB_N2L  0x51 // DAC Right BiQuadB Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_RBQB_D1H  0x52 // DAC Right BiQuadB Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQB_D1L  0x53 // DAC Right BiQuadB Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQB_D2H  0x54 // DAC Right BiQuadB Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQB_D2L  0x55 // DAC Right BiQuadB Coeff D2 (7:0)

#define TLV320DAC3100_REG_DAC_RBQC_N0H  0x56 // DAC Right BiQuadC Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_RBQC_N0L  0x57 // DAC Right BiQuadC Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_RBQC_N1H  0x58 // DAC Right BiQuadC Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQC_N1L  0x59 // DAC Right BiQuadC Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQC_N2H  0x5A // DAC Right BiQuadC Coeff N2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQC_N2L  0x5B // DAC Right BiQuadC Coeff N2 (7:0)
#define TLV320DAC3100_REG_DAC_RBQC_D1H  0x5C // DAC Right BiQuadC Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_RBQC_D1L  0x5D // DAC Right BiQuadC Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_RBQC_D2H  0x5E // DAC Right BiQuadC Coeff D2 (15:8)
#define TLV320DAC3100_REG_DAC_RBQC_D2L  0x5F // DAC Right BiQuadC Coeff D2 (7:0)

// BQD...BQF 

// Page 9/13 DAC Coefficient-RAM
#define TLV320DAC3100_REG_DAC_LIIR_N0H   0x02 // DAC Left IIR Filter Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_LIIR_N0L   0x03 // DAC Left IIR Filter Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_LIIR_N1H   0x04 // DAC Left IIR Filter Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_LIIR_N1L   0x05 // DAC Left IIR Filter Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_LIIR_D1H   0x06 // DAC Left IIR Filter Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_LIIR_D1L   0x07 // DAC Left IIR Filter Coeff D1 (7:0)
#define TLV320DAC3100_REG_DAC_RIIR_N0H   0x08 // DAC Right IIR Filter Coeff N0 (15:8)
#define TLV320DAC3100_REG_DAC_RIIR_N0L   0x09 // DAC Right IIR Filter Coeff N0 (7:0)
#define TLV320DAC3100_REG_DAC_RIIR_N1H   0x0A // DAC Right IIR Filter Coeff N1 (15:8)
#define TLV320DAC3100_REG_DAC_RIIR_N1L   0x0B // DAC Right IIR Filter Coeff N1 (7:0)
#define TLV320DAC3100_REG_DAC_RIIR_D1H   0x0C // DAC Right IIR Filter Coeff D1 (15:8)
#define TLV320DAC3100_REG_DAC_RIIR_D1L   0x0D // DAC Right IIR Filter Coeff D1 (7:0)

// Page 9/13 DRC Coefficient-RAM
#define TLV320DAC3100_REG_DRC_HPF_N0H   0x0E // DRC High Pass Filter Coeff N0 (15:8)
#define TLV320DAC3100_REG_DRC_HPF_N0L   0x0F // DRC High Pass Filter Coeff N0 (7:0)
#define TLV320DAC3100_REG_DRC_HPF_N1H   0x10 // DRC High Pass Filter Coeff N1 (15:8)
#define TLV320DAC3100_REG_DRC_HPF_N1L   0x11 // DRC High Pass Filter Coeff N1 (7:0)
#define TLV320DAC3100_REG_DRC_HPF_D1H   0x12 // DRC High Pass Filter Coeff D1 (15:8)
#define TLV320DAC3100_REG_DRC_HPF_D1L   0x13 // DRC High Pass Filter Coeff D1 (7:0)
#define TLV320DAC3100_REG_DRC_LPF_N0H   0x14 // DRC Low Pass Filter Coeff N0 (15:8)
#define TLV320DAC3100_REG_DRC_LPF_N0L   0x15 // DRC Low Pass Filter Coeff N0 (7:0)
#define TLV320DAC3100_REG_DRC_LPF_N1H   0x16 // DRC Low Pass Filter Coeff N1 (15:8)
#define TLV320DAC3100_REG_DRC_LPF_N1L   0x17 // DRC Low Pass Filter Coeff N1 (7:0)
#define TLV320DAC3100_REG_DRC_LPF_D1H   0x18 // DRC Low Pass Filter Coeff D1 (15:8)
#define TLV320DAC3100_REG_DRC_LPF_D1L   0x19 // DRC Low Pass Filter Coeff D1 (7:0)

//
// selecting various DAC registers for serial printout (just for debugging purposes)
//
#define P0_I2S 0x0001
#define P0_PLL 0x0002
#define P0_DRC 0x0004
#define Px_BQA 0x0010
#define Px_BQB 0x0020
#define Px_BQC 0x0040
#define Px_BQD 0x0080
#define Px_BQE 0x0100
#define Px_BQF 0x0200
#define Px_IIR 0x1000
#define P9_DRC 0x2000

typedef enum {
  TLV320_DRC_THRESHOLD_MINUS_3DB  = 0b000,  //  -3dB
  TLV320_DRC_THRESHOLD_MINUS_6DB  = 0b001,  //  -6dB
  TLV320_DRC_THRESHOLD_MINUS_9DB  = 0b010,  //  -9dB
  TLV320_DRC_THRESHOLD_MINUS_12DB = 0b011,  // -12dB
  TLV320_DRC_THRESHOLD_MINUS_15DB = 0b100,  // -15dB
  TLV320_DRC_THRESHOLD_MINUS_18DB = 0b101,  // -18dB
  TLV320_DRC_THRESHOLD_MINUS_21DB = 0b110,  // -21dB
  TLV320_DRC_THRESHOLD_MINUS_24DB = 0b111   // -24dB
} tlv320_drc_threshold_t;

typedef enum {
  TLV320_DRC_HYST_0DB = 0b00,  // No hysteresis
  TLV320_DRC_HYST_1DB = 0b01,  // 1dB hysteresis
  TLV320_DRC_HYST_2DB = 0b10,  // 2dB hysteresis
  TLV320_DRC_HYST_3DB = 0b11   // 3dB hysteresis
} tlv320_drc_hyst_t;

typedef enum {
  TLV320_DRC_HOLD_TIME_DISABLED       = 0b0000,  // disabled
  TLV320_DRC_HOLD_TIME_32_SAMPLES     = 0b0001,  // 
  TLV320_DRC_HOLD_TIME_64_SAMPLES     = 0b0010,  // 
  TLV320_DRC_HOLD_TIME_128_SAMPLES    = 0b0011,  // 
  TLV320_DRC_HOLD_TIME_256_SAMPLES    = 0b0100,  // 
  TLV320_DRC_HOLD_TIME_512_SAMPLES    = 0b0101,  // 
  TLV320_DRC_HOLD_TIME_1024_SAMPLES   = 0b0110,  // 
  TLV320_DRC_HOLD_TIME_2048_SAMPLES   = 0b0111,  //
  TLV320_DRC_HOLD_TIME_4096_SAMPLES   = 0b1000,  //
  TLV320_DRC_HOLD_TIME_8192_SAMPLES   = 0b1001,  //
  TLV320_DRC_HOLD_TIME_16384_SAMPLES  = 0b1010,  //
  TLV320_DRC_HOLD_TIME_32768_SAMPLES  = 0b1011,  //
  TLV320_DRC_HOLD_TIME_65536_SAMPLES  = 0b1100,  //
  TLV320_DRC_HOLD_TIME_98304_SAMPLES  = 0b1101,  //
  TLV320_DRC_HOLD_TIME_131072_SAMPLES = 0b1110,  //
  TLV320_DRC_HOLD_TIME_163840_SAMPLES = 0b1111   //
} tlv320_drc_hold_time_t;

typedef enum {
  TLV320_DRC_ATTACK_RATE_4DB          = 0b0000,  // 4dB
  TLV320_DRC_ATTACK_RATE_2DB          = 0b0001,  // 4dB/2^1 = 2dB
  TLV320_DRC_ATTACK_RATE_1DB          = 0b0010,  // 4dB/2^2 = 1dB
  TLV320_DRC_ATTACK_RATE_0_5DB        = 0b0011,  // 4dB/2^3 = 0.5dB
  TLV320_DRC_ATTACK_RATE_0_25DB       = 0b0100,  // 4dB/2^4 = 0.25dB
  TLV320_DRC_ATTACK_RATE_0_125DB      = 0b0101,  // 4dB/2^5 = 0.125dB
  TLV320_DRC_ATTACK_RATE_0_0625DB     = 0b0110,  // 4dB/2^6 = 0.0625dB
  TLV320_DRC_ATTACK_RATE_0_03125DB    = 0b0111,  // 4dB/2^7 = 0.03125dB
  TLV320_DRC_ATTACK_RATE_0_015625DB   = 0b1000,  // 4dB/2^8 = 0.015625dB
  TLV320_DRC_ATTACK_RATE_0_0078125DB  = 0b1001,  // 4dB/2^9 = 0.0078125dB
  TLV320_DRC_ATTACK_RATE_0_00390625DB = 0b1010,  // 4dB/2^10 = 0.00390625dB
  TLV320_DRC_ATTACK_RATE_0_00195312DB = 0b1011,  // 4dB/2^11 = 0.00195312dB
  TLV320_DRC_ATTACK_RATE_0_00097656DB = 0b1100,  // 4dB/2^12 = 0.00097656dB
  TLV320_DRC_ATTACK_RATE_0_00048828DB = 0b1101,  // 4dB/2^13 = 0.00048828dB
  TLV320_DRC_ATTACK_RATE_0_00024414DB = 0b1110,  // 4dB/2^14 = 0.00024414dB
  TLV320_DRC_ATTACK_RATE_0_00012207DB = 0b1111   // 4dB/2^15 = 0.00012207dB
} tlv320_drc_attack_rate_t;

typedef enum {
  TLV320_DRC_DECAY_RATE_0_015625DB       = 0b0000,  // 0.015625dB
  TLV320_DRC_DECAY_RATE_0_0078125DB      = 0b0001,  // 0.015625dB/2^1 = 0.0078125dB
  TLV320_DRC_DECAY_RATE_0_00390625DB     = 0b0010,  // 0.015625dB/2^2 = 0.00390625dB
  TLV320_DRC_DECAY_RATE_0_00195312DB     = 0b0011,  // 0.015625dB/2^3 = 0.001953125dB
  TLV320_DRC_DECAY_RATE_0_000976562DB    = 0b0100,  // 0.015625dB/2^4 = 0.0009765625dB
  TLV320_DRC_DECAY_RATE_0_000488281DB    = 0b0101,  // 0.015625dB/2^5 = 0.00048828125dB
  TLV320_DRC_DECAY_RATE_0_000244140DB    = 0b0110,  // 0.015625dB/2^6 = 0.000244140625dB
  TLV320_DRC_DECAY_RATE_0_000122070DB    = 0b0111,  // 0.015625dB/2^7 = 0.0001220703125dB
  TLV320_DRC_DECAY_RATE_0_0000610351DB   = 0b1000,  // 0.015625dB/2^8 = 0.00006103515625dB
  TLV320_DRC_DECAY_RATE_0_0000305175DB   = 0b1001,  // 0.015625dB/2^9 = 0.000030517578125dB
  TLV320_DRC_DECAY_RATE_0_0000152587DB   = 0b1010,  // 0.015625dB/2^10 = 0.0000152587890625dB
  TLV320_DRC_DECAY_RATE_0_00000762939DB  = 0b1011,  // 0.015625dB/2^11 = 0.00000762939453125dB
  TLV320_DRC_DECAY_RATE_0_00000381469DB  = 0b1100,  // 0.015625dB/2^12 = 0.000003814697265625dB
  TLV320_DRC_DECAY_RATE_0_00000190734DB  = 0b1101,  // 0.015625dB/2^13 = 0.0000019073486328125dB
  TLV320_DRC_DECAY_RATE_0_000000953674DB = 0b1110,  // 0.015625dB/2^14 = 0.00000095367431640625dB
  TLV320_DRC_DECAY_RATE_0_000000476837DB = 0b1111   // 0.015625dB/2^15 = 0.000000476837158203125dB
} tlv320_drc_decay_rate_t;

typedef enum {
  TLV320_FILTER_CHAN_ALL = 0,          // filter settings apply to left and right channel
  TLV320_FILTER_CHAN_LEFT,             // filter settings apply to left channel only
  TLV320_FILTER_CHAN_RIGHT,            // filter settings apply to right channel only
} tlv320_filter_channel_t;

typedef enum {
  TLV320_FILTER_SECTION_BIQUAD_A = 0,  // BIQUAD_A section (2nd order filter)
  TLV320_FILTER_SECTION_BIQUAD_B,      // BIQUAD_B section (2nd order filter)
  TLV320_FILTER_SECTION_BIQUAD_C,      // BIQUAD_C section (2nd order filter)
  TLV320_FILTER_SECTION_BIQUAD_D,      // BIQUAD_D section (2nd order filter)
  TLV320_FILTER_SECTION_BIQUAD_E,      // BIQUAD_E section (2nd order filter)
  TLV320_FILTER_SECTION_BIQUAD_F,      // BIQUAD_F section (2nd order filter)
  TLV320_FILTER_SECTION_IIR,           // IIR section (1st order filter)
} tlv320_filter_section_t;

typedef enum {
  TLV320_FILTER_TYPE_LOW_PASS = 0,     // low pass filter (LPF)
  TLV320_FILTER_TYPE_HIGH_PASS,        // high pass filter (HPF)
  TLV320_FILTER_TYPE_NOTCH,            // notch filter
  TLV320_FILTER_TYPE_EQ,               // peaking EQ filter
  TLV320_FILTER_TYPE_BASS_SHELF,       // bass shelf filter
  TLV320_FILTER_TYPE_TREBLE_SHELF      // treble shelf filter
} tlv320_filter_type_t;

typedef struct 
{
  bool enabled { true };                    // enable/disable filter
  tlv320_filter_section_t section;          // filter section: IIR, BiQuadA, BiQuadB, etc.
  tlv320_filter_channel_t channel           // settings apply to left, right or both channels,
               { TLV320_FILTER_CHAN_ALL };  // default is left AND right
  tlv320_filter_type_t    type;             // filter type: low pass, high pass, notch, etc.
  float fs     { 44100 };                   // sample frequency (e.g. 16000, 32000, 44100(default), 48000)
  float fc     { 0 };                       // -3dB corner frequency (LPF, HPF) or center frequency (notch, EQ)
  float bw     { 0.0};                      // Hz, bandwidth (only notch and EQ)
  float gain   { 0.0 };                     // dB, filter gain (default is 0)
  float Q      { 0.0 };                     // filter quality factor (can be given instead of bandwidth)
  float S      { 1.0 };                     // shelfing factor for bass/treble shelf filter
  uint8_t N0H  { 0x7F }, N0L { 0xFF },      // filter coefficients, default is linear filter
          N1H  { 0 },    N1L { 0 }, 
          N2H  { 0 },    N2L { 0 },
          D1H  { 0 },    D1L { 0 }, 
          D2H  { 0 },    D2L { 0 };
} tlv320_filter_cfg_t;

class TLV320DAC3101 : public Adafruit_TLV320DAC3100 {
public:
  TLV320DAC3101() : Adafruit_TLV320DAC3100() {};   // Constructor
  bool getRegisterValue(uint8_t page, uint8_t registr, uint8_t *value);
  bool getRegisterValue(uint8_t page, uint8_t registr, uint16_t *value);
  bool enableSpeaker(bool en);
  bool configureSPK_PGA(tlv320_spk_gain_t gain, bool unmute);
  
  bool setSPKVolume(bool route_enabled, uint8_t gain);
  bool setDRC(bool enabled, 
    tlv320_drc_threshold_t threshold = TLV320_DRC_THRESHOLD_MINUS_24DB, 
    tlv320_drc_hyst_t hysteresis = TLV320_DRC_HYST_2DB,
    tlv320_drc_hold_time_t hold_time = TLV320_DRC_HOLD_TIME_DISABLED, 
    tlv320_drc_attack_rate_t attack_rate = TLV320_DRC_ATTACK_RATE_0_00195312DB,
    tlv320_drc_decay_rate_t decay_rate = TLV320_DRC_DECAY_RATE_0_000244140DB,
    uint8_t *hpf_buf = NULL, uint8_t hpf_buf_length = 0,
    uint8_t *lpf_buf = NULL, uint8_t lpf_buf_length = 0);
    bool powerOnDAC(bool left_dac_on, bool right_dac_on);
    bool calculateDACFilterCoeffs(tlv320_filter_cfg_t *filter_cfg);
    bool setDACFilter(tlv320_filter_cfg_t *filter_cfg);
    bool setAdaptiveMode(bool enabled);
    bool getAdaptiveMode();
    // just for debugging purposes
    void printRegisterSettings(const char *s = "", uint16_t select = (uint16_t)0xFFFF);
    
private:
  bool setPage(uint8_t page);
  uint32_t float2Hex(double floatN, int bits = 16);
  double calculateBWoctave(double fc, double bw);
  bool normalizeForA0(double *a0, double *a1, double *a2,
                      double *b0, double *b1, double *b2);
  bool refactorB(double *b0, double *b1, double *b2);

  Adafruit_I2CDevice *i2c_dev = new Adafruit_I2CDevice(TLV320DAC3100_I2CADDR_DEFAULT, &Wire);
};

#endif