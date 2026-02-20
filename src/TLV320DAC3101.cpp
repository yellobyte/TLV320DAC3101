/*
  Audio processing with the Texas Instruments TLV320DAC3101 Audio Stereo DAC.

  The lib is build upon the Adafruit TLV320 I2S library and extends it with routines
  for filtering (low/high pass, notch, EQ & shelf using IIR and/or BiQuad filters),
  dynamic range compression DRC, adaptive filtering mode and stereo speaker output
  for the TLV320DAC3101.

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

#include "TLV320DAC3101.h"

bool TLV320DAC3101::setPage(uint8_t page)
{
  Adafruit_BusIO_Register page_reg(i2c_dev, TLV320DAC3100_REG_PAGE_SELECT);

  return page_reg.write(page);
}

bool TLV320DAC3101::getRegisterValue(uint8_t page, uint8_t registr, uint8_t *value)
{
  if (!setPage(page)) return false;
  Adafruit_BusIO_Register reg(i2c_dev, registr);

  return reg.read(value);
}

bool TLV320DAC3101::getRegisterValue(uint8_t page, uint8_t registr, uint16_t *value)
{
  if (!setPage(page)) return false;
  Adafruit_BusIO_Register reg(i2c_dev, registr);

  return reg.read(value);
}

bool TLV320DAC3101::enableSpeaker(bool en)
{
  // left channel
  if (!Adafruit_TLV320DAC3100::enableSpeaker(en)) return false;

  // right channel
  if (!setPage(1)) return false;
  Adafruit_BusIO_Register spk_ampR = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_AMP);
  Adafruit_BusIO_RegisterBits spk_enR = Adafruit_BusIO_RegisterBits(&spk_ampR, 1, 6);

  return spk_enR.write(en);
}

bool TLV320DAC3101::configureSPK_PGA(tlv320_spk_gain_t gain, bool unmute)
{
  // left channel
  if (!Adafruit_TLV320DAC3100::configureSPK_PGA(gain, unmute)) return false;

  // right channel
  if (!setPage(1)) return false;
  Adafruit_BusIO_Register spkR = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_DRIVER + 1);
  Adafruit_BusIO_RegisterBits gain_bitsR = Adafruit_BusIO_RegisterBits(&spkR, 2, 3);
  Adafruit_BusIO_RegisterBits muteR = Adafruit_BusIO_RegisterBits(&spkR, 1, 2);
  if (!gain_bitsR.write(gain)) return false;

  return muteR.write(unmute);
}

bool TLV320DAC3101::setSPKVolume(bool route_enabled, uint8_t gain)
{
  // left channel
  if (!Adafruit_TLV320DAC3100::setSPKVolume(route_enabled, gain)) return false;

  // right channel
  if (!setPage(1)) return false;
  Adafruit_BusIO_Register spk_volR = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_SPK_VOL + 1);
  Adafruit_BusIO_RegisterBits routeR = Adafruit_BusIO_RegisterBits(&spk_volR, 1, 7);
  Adafruit_BusIO_RegisterBits volR = Adafruit_BusIO_RegisterBits(&spk_volR, 7, 0);
  if (gain > 0x7F) gain = 0x7F;
  if (!routeR.write(route_enabled)) return false;

  return volR.write(gain);
}

bool TLV320DAC3101::setDRC(bool enable, bool left_channel, bool right_channel,
                           tlv320_drc_param_t *drc_param)
{
  // default coefficients for DRC filter (LPF & HPF)
  uint8_t drc_lpf_coeffs_default[6] = {0x00, 0x11, 0x00, 0x11, 0x7F, 0xDE};
  uint8_t drc_hpf_coeffs_default[6] = {0x7F, 0xF7, 0x80, 0x09, 0x7F, 0xEF};

  uint8_t *drc_lpf_coeffs, *drc_hpf_coeffs;
  bool adaptive_mode, left_dac_on_orig, left_dac_on, right_dac_on_orig, right_dac_on,
       left_mute_orig = false, right_mute_orig = false;
  float left_vol_orig = 0, right_vol_orig = 0;

  Adafruit_BusIO_Register drc1 = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_CONTROL_1);
  Adafruit_BusIO_RegisterBits drc1_enable_l = Adafruit_BusIO_RegisterBits(&drc1, 1, 6);
  Adafruit_BusIO_RegisterBits drc1_enable_r = Adafruit_BusIO_RegisterBits(&drc1, 1, 5);
  Adafruit_BusIO_RegisterBits drc1_thres = Adafruit_BusIO_RegisterBits(&drc1, 3, 2);
  Adafruit_BusIO_RegisterBits drc1_hyst = Adafruit_BusIO_RegisterBits(&drc1, 2, 0);

  Adafruit_BusIO_Register drc2 = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_CONTROL_2);
  Adafruit_BusIO_RegisterBits drc2_hold = Adafruit_BusIO_RegisterBits(&drc2, 4, 3);

  Adafruit_BusIO_Register drc3 = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_CONTROL_3);
  Adafruit_BusIO_RegisterBits drc3_attack = Adafruit_BusIO_RegisterBits(&drc3, 4, 4);
  Adafruit_BusIO_RegisterBits drc3_decay = Adafruit_BusIO_RegisterBits(&drc3, 4, 0);

  Adafruit_BusIO_Register flag2_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG2);
  Adafruit_BusIO_RegisterBits lpga_bit(&flag2_reg, 1, 4);
  Adafruit_BusIO_RegisterBits rpga_bit(&flag2_reg, 1, 0);

  Adafruit_BusIO_Register vol_ctrl = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_VOL_CTRL);
  Adafruit_BusIO_RegisterBits left_mute_bit = Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 3);
  Adafruit_BusIO_RegisterBits right_mute_bit = Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 2);

  Adafruit_BusIO_Register cram_ctrl_reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_CRAM_CTRL);
  Adafruit_BusIO_RegisterBits cram_adaptive = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 2);
  Adafruit_BusIO_RegisterBits cram_buffer_switch = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 0);

  Adafruit_BusIO_Register dac_path = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_DATAPATH);
  Adafruit_BusIO_RegisterBits left_power = Adafruit_BusIO_RegisterBits(&dac_path, 1, 7);
  Adafruit_BusIO_RegisterBits right_power = Adafruit_BusIO_RegisterBits(&dac_path, 1, 6);

  if (!setPage(0)) return false;

  if (!drc1_enable_l.write(enable)) return false;
  if (!drc1_enable_r.write(enable)) return false;

  if (!enable) return true;             // no reason to continue here, DRC has been disabled

  if (drc_param != NULL) {
    // possibly non-standard parameters given
    if (!drc1_thres.write(drc_param->threshold)) return false;
    if (!drc1_hyst.write(drc_param->hyst)) return false;
    if (!drc2_hold.write(drc_param->hold)) return false;
    if (!drc3_attack.write(drc_param->attack)) return false;
    if (!drc3_decay.write(drc_param->decay)) return false;

    drc_lpf_coeffs = (drc_param->lpf_coeffs != NULL) ? drc_param->lpf_coeffs : drc_lpf_coeffs_default;
    drc_hpf_coeffs = (drc_param->hpf_coeffs != NULL) ? drc_param->hpf_coeffs : drc_hpf_coeffs_default;
  }
  else {
    // use recommended settings, Ch. 6.3.10.4.2...5
    if (!drc1_thres.write(TLV320_DRC_THRESHOLD_MINUS_24DB)) return false;
    if (!drc1_hyst.write(TLV320_DRC_HYST_3DB)) return false;
    if (!drc2_hold.write(TLV320_DRC_HOLD_TIME_DISABLED)) return false;
    if (!drc3_attack.write(TLV320_DRC_ATTACK_RATE_0_00195312DB)) return false;
    if (!drc3_decay.write(TLV320_DRC_DECAY_RATE_0_000244140DB)) return false;

    drc_lpf_coeffs = drc_lpf_coeffs_default;
    drc_hpf_coeffs = drc_hpf_coeffs_default;
  }

  if (!setPage(8)) return false;
  adaptive_mode = cram_adaptive.read();

  if (!setPage(0)) return false;
  left_dac_on_orig = left_dac_on = left_power.read();
  right_dac_on_orig = right_dac_on = right_power.read();

  if (!adaptive_mode) {
    // ramp down procedure as recommended in Ch. 6.3.10.9, figure 6-18
    if (left_dac_on && left_channel) {
      left_mute_orig = left_mute_bit.read();
      left_vol_orig = getChannelVolume(false);
      if (!setChannelVolume(false, -63.5)) return false;
      while (!lpga_bit.read()) {};
      if (!left_mute_bit.write(true)) return false;
      delay(20);
      if (!left_power.write(false)) return false;  // power down left DAC to get access
    }
    if (right_dac_on && right_channel) {
      right_mute_orig = right_mute_bit.read();
      right_vol_orig = getChannelVolume(true);
      if (!setChannelVolume(true, -63.5)) return false;
      while (!rpga_bit.read()) {};
      if (!right_mute_bit.write(true)) return false;
      delay(20);
      if (!right_power.write(false)) return false; // power down right DAC to get access
    }
  }

  // setting DRC LPF/HPF coefficients
  if (!setPage(9)) return false;    // write to C-RAM Buffer A  (A or B in adaptive mode)

  Adafruit_BusIO_Register drc_lpf = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_LPF_N0H);
  if (!(drc_lpf.write(drc_lpf_coeffs, 6))) return false;

  Adafruit_BusIO_Register drc_hpf = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_HPF_N0H);
  if (!(drc_hpf.write(drc_hpf_coeffs, 6))) return false;

  if (adaptive_mode) {
    if (!setPage(8)) return false;

    // triggering a switch between buffer A and B
    if (!cram_buffer_switch.write(true)) return false;

    // Note: if the I2S audio bus is not active (externally or internally) or both(!) DACs are
    // powered down then switching will not happen and we are stuck. The timer is our bailout.
    uint32_t start = millis();

    while (true) {
      if (!cram_buffer_switch.read()) break;
      if ((millis() - start) > 20) return false;
    };
  }

  if (!setPage(13)) return false;    // write to C-RAM Buffer B  (A or B in adaptive mode)

  drc_lpf = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_LPF_N0H);
  if (!(drc_lpf.write(drc_lpf_coeffs, 6))) return false;

  drc_hpf = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_HPF_N0H);
  if (!(drc_hpf.write(drc_hpf_coeffs, 6))) return false;

  if (!adaptive_mode) {
    // ramp up procedure as recommended in Ch. 6.3.10.9, figure 6-18
    if (!setPage(0)) return false;

    if (left_dac_on_orig && left_channel) {
      if (!left_power.write(true)) return false;  // power up left DAC
      delay(20);
      if (!left_mute_bit.write(left_mute_orig)) return false;
      if (!setChannelVolume(false, left_vol_orig)) return false;
      while (!lpga_bit.read()) {};
    }

    if (right_dac_on_orig && right_channel) {
      if (!right_power.write(true)) return false; // power up right DAC
      delay(20);
      if (!right_mute_bit.write(right_mute_orig)) return false;
      if (!setChannelVolume(true, right_vol_orig)) return false;
      while (!rpga_bit.read()) {};
    }
  }

  return true;
}

bool TLV320DAC3101::powerOnDAC(bool left_dac_on, bool right_dac_on)
{
  if (!setPage(0)) return false;

  Adafruit_BusIO_Register dac = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_DATAPATH);
  Adafruit_BusIO_RegisterBits left_power = Adafruit_BusIO_RegisterBits(&dac, 1, 7);
  Adafruit_BusIO_RegisterBits right_power = Adafruit_BusIO_RegisterBits(&dac, 1, 6);
  if (!left_power.write(left_dac_on)) return false;
  if (!right_power.write(right_dac_on)) return false;

  return true;
}

#ifdef _DEBUG_
__attribute__((optimize("O0")))
#endif
bool TLV320DAC3101::calcDACFilterCoefficients(float sample_frequency, tlv320_filter_type_t type,
                                              tlv320_filter_t filter, tlv320_filter_param_t *param)
{
  double alpha, a0, a1, a2, b0, b1, b2, factor, w0;
  uint16_t N0, N1, N2, D1, D2;

  if (sample_frequency <= 0 || sample_frequency > 96000 ||
      param == NULL || param->fc <= 0 || param->fc > (sample_frequency / 2) - 100) {
    return false;
  }

  w0 = (2 * M_PI * param->fc) / sample_frequency;

  if (filter == TLV320_FILTER_IIR) {
    //
    // IIR filter (1st order, only needs coefficients N0, N1, D1)
    //
    double K = tan(w0 / 2.0);
    alpha = 1 + K;
    a1 = (1 - K) / alpha;
    a2 = b2 = 0;                // always 0 for 1st order
    if (fabs(a1) >= 1) {
#ifdef _DEBUG_
      Serial.printf("error: abs(a1):%f >= 1.0", fabs(a1));
#endif
      return false;
    }

    switch (type) {
      case TLV320_FILTER_TYPE_LOW_PASS:
        b0 = pow(10, param->gain / 20) * (K / alpha);
        b1 = b0;
        break;
      case TLV320_FILTER_TYPE_HIGH_PASS:
        b0 = pow(10, param->gain / 20) * (1 / alpha);
        b1 = -1 * b0;
        break;
      default:
        return false;
    }

    // scale coeffs so that none of b0, b1 is greater than 1
    factor = fmax(fabs(b0), fabs(b1));
    if (factor > 1) {
      b0 = b0 / factor;
      b1 = b1 / factor;
    }

    N0 = float2Hex(b0);
    N1 = float2Hex(b1);
    D1 = float2Hex(a1);

    param->N0H = N0 >> 8;
    param->N0L = N0 & 0xFF;
    param->N1H = N1 >> 8;
    param->N1L = N1 & 0xFF;
    param->N2H = param->N2L = 0;        // N2 always = 0 for 1st order
    param->D1H = D1 >> 8;
    param->D1L = D1 & 0xFF;
    param->D2H = param->D2L = 0;        // D2 always = 0 for 1st order

    return true;
  }
  else {
    //
    // BiQuad filter (2nd order, needs coefficients N0, N1, N2, D1 and D2)
    //
    double a, d, k, s, A, F, H, Q, denominator, bw, gd, gn, tanB, wc;
    double cosW = cos(w0);
    double sinW = sin(w0);

    if (param->gain < -12.0 || param->gain > 12.0) return false;

    switch (type) {
      case TLV320_FILTER_TYPE_LOW_PASS:
        Q = 0.707;                    // Q is a constant for 2nd order Butterworth LPF
        wc = 2 * M_PI * param->fc;
        k = wc / tan(M_PI * param->fc / sample_frequency);

        a0 = k * k + wc * wc + wc * k / Q;
        a1 = -2 * k * k + 2 * wc * wc;
        a2 = -wc * k / Q + k * k + wc * wc;
        b0 = pow(10, param->gain / 20) * wc * wc;
        b1 = pow(10, param->gain / 20) * 2 * wc * wc;
        b2 = pow(10, param->gain / 20) * wc * wc;

        normalizeForA0(&a0, &a1, &a2, &b0, &b1, &b2);
        break;

      case TLV320_FILTER_TYPE_HIGH_PASS:
        Q = 0.707;                    // Q is a constant for 2nd order Butterworth HPF
        wc = 2 * M_PI * param->fc;
        k = wc / tan(M_PI * param->fc / sample_frequency);

        a0 = k * k + wc * wc + wc * k / Q;
        a1 = -2 * k * k + 2 * wc * wc;
        a2 = -wc * k / Q + k * k + wc * wc;
        b0 = pow(10, param->gain / 20) * k * k;
        b1 = pow(10, param->gain / 20) * -2 * k * k;
        b2 = pow(10, param->gain / 20) * k * k;

        normalizeForA0(&a0, &a1, &a2, &b0, &b1, &b2);
        break;

      case TLV320_FILTER_TYPE_NOTCH:
        if (param->bw <= 0 && param->Q <= 0) return false;

        // prefer bandwidth over Q
        if (param->bw > 0) {
          bw = param->bw;
        }
        else {
          bw = 2 * param->fc * sinh((sinW / w0) * asinh(1 / (2 * param->Q)));
        }
        if (bw > ((sample_frequency / 2) - 100)) return false;

        tanB = tan(M_PI * bw / sample_frequency);
        A = pow(10, param->gain / 20);
        a = ((1 - tanB) / (1 + tanB));
        d = -cosW;

        // calculate notch coeffs, already a0 normalized
        b0 = (1 + a) * A / 2;
        b1 = d * (1 + abs(a)) * A;
        b2 = (1 + a) * A / 2;
        a0 = 1;
        a1 = b1 / A;
        a2 = a;
        break;

      case TLV320_FILTER_TYPE_EQ:
        if (param->bw <= 0 && param->Q <= 0) return false;

        // prefer bandwidth over Q
        if (param->bw > 0) {
          bw = param->bw;
        }
        else {
          bw = 2 * param->fc * sinh((sinW / w0) * asinh(1 / (2 * param->Q)));
        }
        if (bw > ((sample_frequency / 2) - 100)) return false;

        tanB = tan(M_PI * bw / sample_frequency);
        A = pow(10, param->gain / 20);
        H = A - 1;
        a = (A < 1) ? ((tanB - A) / (tanB + A)) : ((tanB - 1) / (tanB + 1));
        d = -cosW;

        // calculate EQ coeffs, already a0 normalized
        b0 = 1 + (1 + a) * H / 2;
        b1 = d * (1 + abs(a));
        b2 = (-a - (1 + a) * H / 2);
        a0 = 1;
        a1 = b1;
        a2 = -a;
        break;

      case TLV320_FILTER_TYPE_BASS_SHELF:
        A = pow(10, (param->gain / 20));
        s = sqrt(2) / 2;

        if (param->gain > -6 && param->gain < 6) {
          F = sqrt(A);
        }
        else if (A > 1) {
          F = A / sqrt(2);
        }
        else {
          F = A * sqrt(2);
        }

        if (param->gain == 0) {
          gd = 1;
        }
        else {
          gd = pow((F*F - 1) / (A*A - F*F), 0.25);
        }

        gn = sqrt(A) * gd;
        a = tan(M_PI * (param->fc / sample_frequency - 1.0 / 4.0));
        denominator = 2 * s * gd + 1 - 2 * s * gd * a*a + gd*gd * a*a +
                      2 * gd*gd * a + a*a + gd*gd - 2 * a;

        b0 = -(-1 - gn*gn * a*a - a*a - 2 * gn*gn * a - gn*gn - 2 * s * gn +
             2 * s * gn * a*a + 2 * a) / denominator;
        b1 = -(2 - 4 * a - 4 * gn*gn * a - 2 * gn*gn * a*a - 2 * gn*gn +
             2 * a*a) / denominator;
        b2 = (1 + 2 * s * gn * a*a - 2 * a + gn*gn - 2 * s * gn + 2 * gn*gn * a +
             a*a + gn*gn * a*a) / denominator;
        a0 = 1;
        a1 = (-2 + 2 * gd*gd * a*a + 4 * gd*gd * a - 2 * a*a + 2 * gd*gd +
             4 * a) / denominator;
        a2 = (gd*gd * a*a - 2 * a + 1 + 2 * gd*gd * a - 2 * s * gd + a*a +
             2 * s * gd * a*a + gd*gd) / denominator;
        normalizeForA0(&a0, &a1, &a2, &b0, &b1, &b2);

        if (b1 > b0) {
          refactorB(&b0, &b1, &b2);
        }
        break;

      case TLV320_FILTER_TYPE_TREBLE_SHELF:
        A = pow(10, (param->gain / 20));
        s = sqrt(2) / 2;

        if (param->gain > -6 && param->gain < 6) {
          F = sqrt(A);
        }
        else if (A > 1) {
          F = A / sqrt(2);
        }
        else {
          F = A * sqrt(2);
        }

        if (param->gain == 0) {
          gd = 1;
        }
        else {
          gd = pow((F*F - 1) / (A*A - F*F), 0.25);
        }

        gn = sqrt(A) * gd;
        a = tan(M_PI * (param->fc / sample_frequency - 1.0 / 4.0));
        denominator = 1 + gd*gd + 2*s*gd - 2*s*gd*a*a + gd*gd*a*a - 2*gd*gd*a + a*a + 2*a;

        b0 = (gn*gn*a*a + 2*s*gn - 2*gn*gn*a + 1 - 2*s*gn*a*a + a*a + gn*gn + 2*a) / denominator;
        b1 = (2 - 2*gn*gn*a*a + 4*gn*gn*a + 4*a - 2*gn*gn + 2*a*a) / denominator;
        b2 = (1 + 2*s*gn*a*a - 2*s*gn + 2*a + a*a + gn*gn - 2*gn*gn*a + gn*gn*a*a) / denominator;
        a0 = 1;
        a1 = (2 - 2*gd*gd*a*a + 4*gd*gd*a + 2*a*a - 2*gd*gd + 4*a) / denominator;
        a2 = (1 - 2*gd*gd*a + 2*a + gd*gd - 2*s*gd + a*a + gd*gd*a*a + 2*s*gd*a*a) / denominator;
        normalizeForA0(&a0, &a1, &a2, &b0, &b1, &b2);

        if (b1 > b0) {
          refactorB(&b0, &b1, &b2);
        }
        break;

      default:
        return false;
    }

    // scale coeffs so that none of b0, b1, b2 is greater than 1
    factor = fmax(fabs(b0), fmax(fabs(b1 / 2), fabs(b2)));
    if (factor > 1) {
      b0 = b0 / factor;
      b1 = b1 / factor;
      b2 = b2 / factor;
    }

    // convert continuous values to quantized 16bit values
    N0 = (uint16_t)float2Hex(b0);
    N1 = (uint16_t)float2Hex(b1 / 2);
    N2 = (uint16_t)float2Hex(b2);
    D1 = (uint16_t)float2Hex(a1 / -2);
    D2 = (uint16_t)float2Hex(a2 * -1);

    param->N0H = N0 >> 8;
    param->N0L = N0 & 0xFF;
    param->N1H = N1 >> 8;
    param->N1L = N1 & 0xFF;
    param->N2H = N2 >> 8;
    param->N2L = N2 & 0xFF;
    param->D1H = D1 >> 8;
    param->D1L = D1 & 0xFF;
    param->D2H = D2 >> 8;
    param->D2L = D2 & 0xFF;

    return true;
  }
  return false;
}

bool TLV320DAC3101::setAdaptiveMode(bool enabled)
{
  if (!setPage(8)) return false;

  Adafruit_BusIO_Register cram_ctrl_reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_CRAM_CTRL);
  Adafruit_BusIO_RegisterBits cram_adaptive = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 2);
  if (!cram_adaptive.write(enabled)) return false;

  return true;
}

bool TLV320DAC3101::getAdaptiveMode()
{
  if (!setPage(8)) return false;

  Adafruit_BusIO_Register cram_ctrl_reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_CRAM_CTRL);
  Adafruit_BusIO_RegisterBits cram_adaptive = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 2);

  return cram_adaptive.read();
}

#ifdef _DEBUG_
__attribute__((optimize("O0")))
#endif
bool TLV320DAC3101::setDACFilter(bool enabled, bool left_channel, bool right_channel,
                                 tlv320_filter_t filter, tlv320_filter_param_t *param)
{
  // default coefficients for setting a filter back to linear (default)
  uint8_t defaultCoeffs[10] = { 0x7F, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

  uint16_t addr;
  uint8_t *coeffs;
  bool adaptive_mode, left_dac_on_orig, left_dac_on, right_dac_on_orig, right_dac_on,
       left_mute_orig = false, right_mute_orig = false;
  float left_vol_orig = 0, right_vol_orig = 0;

  coeffs = (enabled && param != NULL) ? &(param->N0H) : defaultCoeffs;

  Adafruit_BusIO_Register flag2_reg(i2c_dev, TLV320DAC3100_REG_DAC_FLAG2);
  Adafruit_BusIO_RegisterBits lpga_bit(&flag2_reg, 1, 4);
  Adafruit_BusIO_RegisterBits rpga_bit(&flag2_reg, 1, 0);

  Adafruit_BusIO_Register vol_ctrl = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_VOL_CTRL);
  Adafruit_BusIO_RegisterBits left_mute_bit = Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 3);
  Adafruit_BusIO_RegisterBits right_mute_bit = Adafruit_BusIO_RegisterBits(&vol_ctrl, 1, 2);

  Adafruit_BusIO_Register cram_ctrl_reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_CRAM_CTRL);
  Adafruit_BusIO_RegisterBits cram_adaptive = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 2);
  Adafruit_BusIO_RegisterBits cram_buffer_switch = Adafruit_BusIO_RegisterBits(&cram_ctrl_reg, 1, 0);

  Adafruit_BusIO_Register dac_path = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_DATAPATH);
  Adafruit_BusIO_RegisterBits left_power = Adafruit_BusIO_RegisterBits(&dac_path, 1, 7);
  Adafruit_BusIO_RegisterBits right_power = Adafruit_BusIO_RegisterBits(&dac_path, 1, 6);

  if (!setPage(8)) return false;
  adaptive_mode = cram_adaptive.read();

  if (!setPage(0)) return false;
  left_dac_on_orig = left_dac_on = left_power.read();
  right_dac_on_orig = right_dac_on = right_power.read();

  if (!adaptive_mode) {
    // ramp down procedure as recommended in Ch. 6.3.10.9, figure 6-18
    if (left_dac_on && left_channel) {
      left_mute_orig = left_mute_bit.read();
      left_vol_orig = getChannelVolume(false);
      if (!setChannelVolume(false, -63.5)) return false;
      while (!lpga_bit.read()) {};
      if (!left_mute_bit.write(true)) return false;
      delay(20);
      if (!left_power.write(false)) return false;  // power down left DAC to get access
    }
    if (right_dac_on && right_channel) {
      right_mute_orig = right_mute_bit.read();
      right_vol_orig = getChannelVolume(true);
      if (!setChannelVolume(true, -63.5)) return false;
      while (!rpga_bit.read()) {};
      if (!right_mute_bit.write(true)) return false;
      delay(20);
      if (!right_power.write(false)) return false; // power down right DAC to get access
    }
  }

  if (filter == TLV320_FILTER_IIR) {
    // IIR (1st order filter)
    if (!setPage(9)) return false;   // write to C-RAM Buffer A  (A or B in adaptive mode)

    if (left_channel) {
      Adafruit_BusIO_Register dac_iir_l = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_N0H);
      if (!dac_iir_l.write(coeffs, 4)) return false;      // N0, N1
        dac_iir_l = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_D1H);
      if (!dac_iir_l.write(coeffs + 6, 2)) return false;  // D1
    }
    if (right_channel) {
      Adafruit_BusIO_Register dac_iir_r = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_N0H);
      if (!dac_iir_r.write(coeffs, 4)) return false;      // N0, N1
      dac_iir_r = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_D1H);
      if (!dac_iir_r.write(coeffs + 6, 2)) return false;  // D1
    }
  }
  else {
    // BiQuad (2nd order filter)
    if (!setPage(8)) return false;   // write to C-RAM Buffer A  (A or B in adaptive mode)

    if (left_channel) {
      addr = filter * 10 + 2;
      Adafruit_BusIO_Register dac_biquad_l = Adafruit_BusIO_Register(i2c_dev, addr);
      if (!dac_biquad_l.write(coeffs, 10)) return false;
    }
    if (right_channel) {
      addr = filter * 10 + 66;
      Adafruit_BusIO_Register dac_biquad_r = Adafruit_BusIO_Register(i2c_dev, addr);
      if (!dac_biquad_r.write(coeffs, 10)) return false;
    }
  }

  if (adaptive_mode) {
    if (!setPage(8)) return false;

    // we trigger a switch between buffer A and B
    if (!cram_buffer_switch.write(true)) return false;

    // Note: if the I2S audio bus is not active (externally or internally) or both(!) DACs are
    // powered down then switching will not happen and we are stuck. The timer is our bailout.
    uint32_t start = millis();

    while (true) {
      if (!cram_buffer_switch.read()) break;
      if ((millis() - start) > 20) return false;
    };
  }

  if (filter == TLV320_FILTER_IIR) {
    // IIR (1st order filter)
    if (!setPage(13)) return false;  // write to C-RAM Buffer B (A or B in adaptive mode)

    if (left_channel) {
      Adafruit_BusIO_Register dac_iir_l = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_N0H);
      if (!dac_iir_l.write(coeffs, 4)) return false;      // N0, N1
        dac_iir_l = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_D1H);
      if (!dac_iir_l.write(coeffs + 6, 2)) return false;  // D1
    }

    if (right_channel) {
      Adafruit_BusIO_Register dac_iir_r = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_N0H);
      if (!dac_iir_r.write(coeffs, 4)) return false;      // N0, N1
      dac_iir_r = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_D1H);
      if (!dac_iir_r.write(coeffs + 6, 2)) return false;  // D1
    }
  }
  else {
    // BiQuad (2nd order filter)
    if (!setPage(12)) return false;  // write to C-RAM Buffer B (A or B in adaptive mode)

    if (left_channel) {
      addr = filter * 10 + 2;
      Adafruit_BusIO_Register dac_biquad_l = Adafruit_BusIO_Register(i2c_dev, addr);
      if (!dac_biquad_l.write(coeffs, 10)) return false;
    }

    if (right_channel) {
      addr = filter * 10 + 66;
      Adafruit_BusIO_Register dac_biquad_r = Adafruit_BusIO_Register(i2c_dev, addr);
      if (!dac_biquad_r.write(coeffs, 10)) return false;
    }
  }

  if (!adaptive_mode) {
    // ramp up procedure as recommended in Ch. 6.3.10.9, figure 6-18
    if (!setPage(0)) return false;

    if (left_dac_on_orig && left_channel) {
      if (!left_power.write(true)) return false;  // power up left DAC
      delay(20);
      if (!left_mute_bit.write(left_mute_orig)) return false;
      if (!setChannelVolume(false, left_vol_orig)) return false;
      while (!lpga_bit.read()) {};
    }

    if (right_dac_on_orig && right_channel) {
      if (!right_power.write(true)) return false; // power up right DAC
      delay(20);
      if (!right_mute_bit.write(right_mute_orig)) return false;
      if (!setChannelVolume(true, right_vol_orig)) return false;
      while (!rpga_bit.read()) {};
    }
  }

  return true;
}

uint32_t TLV320DAC3101::float2Hex(double floatN, int bits)
{
  int range = ((1 << (bits - 1)) - 1);
  uint32_t hexVal = (floatN >= 0) ? (uint32_t)floor(floatN * range) :
                                    (uint32_t)ceil(floatN * range);

  return hexVal;
}

double TLV320DAC3101::calculateBWoctave(double fc, double bw)
{
  // bw=f2-f1, f1=(fc)^2/f2, f2=(fc)^2/f1, f2=f1+bw, fc=√(f1×f2)

  double f1 = 0.5 * (sqrt((bw * bw) + (4 * fc * fc)) - bw);
  double f2 = f1 + bw;

  return log(f2 / f1) / log(2);

}

bool TLV320DAC3101::normalizeForA0(double *a0, double *a1, double *a2,
                                   double *b0, double *b1, double *b2)
{
  if (a0 == NULL || a1 == NULL || a2 == NULL ||
      b0 == NULL || b1 == NULL || b2 == NULL || *a0 == 0.0) {
    return false;
  }

  // Normalize so that a0 = 1
  *a1 = *a1 / *a0;
  *a2 = *a2 / *a0;
  *b0 = *b0 / *a0;
  *b1 = *b1 / *a0;
  *b2 = *b2 / *a0;
  *a0 = 1; // = a0 / a0;

  return true;
}

bool TLV320DAC3101::refactorB(double *b0, double *b1, double *b2)
{
  if (b0 == NULL || b1 == NULL || b2 == NULL) {
    return false;
  }

  double factor = fmax(fabs(*b0), fmax(fabs(*b1), fabs(*b2)));
  if (factor > 1) {
    *b0 = *b0 / factor;
    *b1 = *b1 / factor;
    *b2 = *b2 / factor;
  }

  return true;
}

#ifdef _DEBUG_
//
// Only for debugging purposes. Print various DAC register values.
//
__attribute__((optimize("O0")))
void TLV320DAC3101::printRegisterSettings(const char *s, uint16_t select)
{
  uint8_t u8a, u8b, u8c, u8d, u8e, u8f, u8g, u8h, u8i;
  uint8_t buf[10];

  Serial.printf("TLV320 Register Settings [%s]:\n", s);
  if (select & P0_I2S) {
    if (getRegisterValue(0, TLV320DAC3100_REG_CODEC_IF_CTRL1, &u8a)) {
      Serial.printf("P0:I2S(0x%02x)=0x%02x(D7:D6=%u,D5:D4=%u,D3=%u,D2=%u)\n",
                      TLV320DAC3100_REG_CODEC_IF_CTRL1, u8a, (u8a>>6)&0x03, (u8a>>4)&0x03, (u8a>>3)&0x01, (u8a>>2)&0x01);
    }
  }

  if (select & P0_PLL) {
    if (getRegisterValue(0, TLV320DAC3100_REG_CLOCK_MUX1, &u8a) &&
        getRegisterValue(0, TLV320DAC3100_REG_PLL_PROG_PR, &u8b) &&
        getRegisterValue(0, TLV320DAC3100_REG_PLL_PROG_J, &u8c) &&
        getRegisterValue(0, TLV320DAC3100_REG_PLL_PROG_D_MSB, &u8d) &&
        getRegisterValue(0, TLV320DAC3100_REG_PLL_PROG_D_LSB, &u8e) &&
        getRegisterValue(0, TLV320DAC3100_REG_NDAC, &u8f) &&
        getRegisterValue(0, TLV320DAC3100_REG_MDAC, &u8g) &&
        getRegisterValue(0, TLV320DAC3100_REG_DOSR_MSB, &u8h) &&
        getRegisterValue(0, TLV320DAC3100_REG_DOSR_LSB, &u8i)) {
      Serial.printf("P0:PLL(0x%02x...0x%02x): PLL_CLKIN=%u,CODEC_CLKIN=%u,PLLup=%u,P=%u,R=%u,J=%u,D=%u,"
                    "NDACen=%u,NDAC=%u,MDACen=%u,MDAC=%u,DOSR=%u\n",
                    TLV320DAC3100_REG_CLOCK_MUX1, TLV320DAC3100_REG_DOSR_LSB,
                    (u8a>>2)&0x03, u8a&0x03, (u8b>>7)&0x01, (u8b>>4)&0x07, u8b&0x0F, u8c&0x3F, (u8d&0x3F)<<8 | u8e,
                    (u8f>>7)&0x01, u8f&0x7F, (u8g>>7)&0x01, u8g&0x7F, (u8h&0x03)<<8 | u8i);
    }
  }

  if (select & P0_DRC) {
  // DRC Control Register
    if (getRegisterValue(0, TLV320DAC3100_REG_DRC_CONTROL_1, &u8a) &&
        getRegisterValue(0, TLV320DAC3100_REG_DRC_CONTROL_2, &u8b) &&
        getRegisterValue(0, TLV320DAC3100_REG_DRC_CONTROL_3, &u8c)) {
      Serial.printf("P0:DRC_CTL1(0x%02x)=0x%02x(D6=%u,D5=%u,D4:D2=%u,D1:D0=%u), DRC_CTL2(0x%02x)=0x%02x(D6:D3=%u), "
                    "DRC_CTL3(0x%02x)=0x%02x(D7:D4=%u,D3:D0=%u)\n",
                    TLV320DAC3100_REG_DRC_CONTROL_1, u8a, (u8a>>6)&0x01, (u8a>>5)&0x01, (u8a>>2)&0x07, (u8a&0x03),
                    TLV320DAC3100_REG_DRC_CONTROL_2, u8b, (u8b>>3)&0x0F,
                    TLV320DAC3100_REG_DRC_CONTROL_3, u8c, (u8c>>4)&0x0F, (u8c&0x0F));
    }
  }

  if (select & (Px_BQA | Px_BQB | Px_BQC | Px_BQD | Px_BQE | Px_BQF | Px_IIR | Px_DRC)) {
    // C-RAM access mode control
    if (getRegisterValue(8, TLV320DAC3100_REG_DAC_CRAM_CTRL, &u8a)) {
      Serial.printf("P8:C-RAM-Ctrl(0x%02x)=0x%02x(D2=%u,D1=%u,D0=%u)\n",
                    TLV320DAC3100_REG_DAC_CRAM_CTRL, u8a, (u8a>>2)&0x01, (u8a>>1)&0x01, u8a&0x01);
    }
    // Coefficient-RAM buffer is only accessible if DACs are powered down or in adaptive mode
    if ((getRegisterValue(0, TLV320DAC3100_REG_DAC_DATAPATH, &u8a) && (u8a & 0xC0)) &&    // DACs powered up and
        (getRegisterValue(8, TLV320DAC3100_REG_DAC_CRAM_CTRL, &u8b) && !(u8b & 0x04))) {  // no adaptive mode
      Serial.println("No access to C-RAM buffer: DACs are powered up and adaptive mode is disabled.");
      return;
    }
  }

  if (select & Px_BQA) {
    if (!setPage(8)) return;  // BiQuad C-RAM Buffer A

    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQA_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:LBQA:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQA_N0H, TLV320DAC3100_REG_DAC_LBQA_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQA_N1H, TLV320DAC3100_REG_DAC_LBQA_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQA_N2H, TLV320DAC3100_REG_DAC_LBQA_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQA_D1H, TLV320DAC3100_REG_DAC_LBQA_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQA_D2H, TLV320DAC3100_REG_DAC_LBQA_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQA_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:RBQA:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQA_N0H, TLV320DAC3100_REG_DAC_RBQA_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQA_N1H, TLV320DAC3100_REG_DAC_RBQA_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQA_N2H, TLV320DAC3100_REG_DAC_RBQA_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQA_D1H, TLV320DAC3100_REG_DAC_RBQA_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQA_D2H, TLV320DAC3100_REG_DAC_RBQA_D2L, buf[8], buf[9]);
    }

    if (!setPage(12)) return;  // BiQuad C-RAM Buffer B

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQA_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:LBQA:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQA_N0H, TLV320DAC3100_REG_DAC_LBQA_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQA_N1H, TLV320DAC3100_REG_DAC_LBQA_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQA_N2H, TLV320DAC3100_REG_DAC_LBQA_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQA_D1H, TLV320DAC3100_REG_DAC_LBQA_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQA_D2H, TLV320DAC3100_REG_DAC_LBQA_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQA_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:RBQA:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQA_N0H, TLV320DAC3100_REG_DAC_RBQA_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQA_N1H, TLV320DAC3100_REG_DAC_RBQA_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQA_N2H, TLV320DAC3100_REG_DAC_RBQA_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQA_D1H, TLV320DAC3100_REG_DAC_RBQA_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQA_D2H, TLV320DAC3100_REG_DAC_RBQA_D2L, buf[8], buf[9]);
    }
  }

  if (select & Px_BQB) {
    if (!setPage(8)) return;  // BiQuad C-RAM Buffer A

    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQB_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:LBQB:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQB_N0H, TLV320DAC3100_REG_DAC_LBQB_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQB_N1H, TLV320DAC3100_REG_DAC_LBQB_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQB_N2H, TLV320DAC3100_REG_DAC_LBQB_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQB_D1H, TLV320DAC3100_REG_DAC_LBQB_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQB_D2H, TLV320DAC3100_REG_DAC_LBQB_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQB_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:RBQB:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQB_N0H, TLV320DAC3100_REG_DAC_RBQB_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQB_N1H, TLV320DAC3100_REG_DAC_RBQB_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQB_N2H, TLV320DAC3100_REG_DAC_RBQB_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQB_D1H, TLV320DAC3100_REG_DAC_RBQB_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQB_D2H, TLV320DAC3100_REG_DAC_RBQB_D2L, buf[8], buf[9]);
    }

    if (!setPage(12)) return;  // BiQuad C-RAM Buffer B

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQB_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:LBQB:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQB_N0H, TLV320DAC3100_REG_DAC_LBQB_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQB_N1H, TLV320DAC3100_REG_DAC_LBQB_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQB_N2H, TLV320DAC3100_REG_DAC_LBQB_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQB_D1H, TLV320DAC3100_REG_DAC_LBQB_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQB_D2H, TLV320DAC3100_REG_DAC_LBQB_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQB_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:RBQB:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQB_N0H, TLV320DAC3100_REG_DAC_RBQB_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQB_N1H, TLV320DAC3100_REG_DAC_RBQB_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQB_N2H, TLV320DAC3100_REG_DAC_RBQB_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQB_D1H, TLV320DAC3100_REG_DAC_RBQB_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQB_D2H, TLV320DAC3100_REG_DAC_RBQB_D2L, buf[8], buf[9]);
    }
  }

  if (select & Px_BQC) {
    if (!setPage(8)) return;  // BiQuad C-RAM Buffer A

    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQC_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:LBQC:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQC_N0H, TLV320DAC3100_REG_DAC_LBQC_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQC_N1H, TLV320DAC3100_REG_DAC_LBQC_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQC_N2H, TLV320DAC3100_REG_DAC_LBQC_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQC_D1H, TLV320DAC3100_REG_DAC_LBQC_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQC_D2H, TLV320DAC3100_REG_DAC_LBQC_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQC_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P8:RAM-A:RBQC:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQC_N0H, TLV320DAC3100_REG_DAC_RBQC_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQC_N1H, TLV320DAC3100_REG_DAC_RBQC_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQC_N2H, TLV320DAC3100_REG_DAC_RBQC_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQC_D1H, TLV320DAC3100_REG_DAC_RBQC_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQC_D2H, TLV320DAC3100_REG_DAC_RBQC_D2L, buf[8], buf[9]);
    }

    if (!setPage(12)) return;  // BiQuad C-RAM Buffer B

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LBQC_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:LBQC:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LBQC_N0H, TLV320DAC3100_REG_DAC_LBQC_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LBQC_N1H, TLV320DAC3100_REG_DAC_LBQC_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LBQC_N2H, TLV320DAC3100_REG_DAC_LBQC_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_LBQC_D1H, TLV320DAC3100_REG_DAC_LBQC_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_LBQC_D2H, TLV320DAC3100_REG_DAC_LBQC_D2L, buf[8], buf[9]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RBQC_N0H);
    if (reg.read(buf, 10)) {
      Serial.printf("P12:RAM-B:RBQC:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "N2H/L(0x%02x/%02x)=0x%02x/%02x, D1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D2H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RBQC_N0H, TLV320DAC3100_REG_DAC_RBQC_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RBQC_N1H, TLV320DAC3100_REG_DAC_RBQC_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RBQC_N2H, TLV320DAC3100_REG_DAC_RBQC_N2L, buf[4], buf[5],
                    TLV320DAC3100_REG_DAC_RBQC_D1H, TLV320DAC3100_REG_DAC_RBQC_D1L, buf[6], buf[7],
                    TLV320DAC3100_REG_DAC_RBQC_D2H, TLV320DAC3100_REG_DAC_RBQC_D2L, buf[8], buf[9]);
    }
  }

  // tbd
  // if (select & (Px_BQD)) {
  // }
  // ....

  if (select & Px_DRC) {
    if (!setPage(9)) return;  // DRC C-RAM Buffer A

    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_LPF_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P9:DRC_LPF N0H/L(0x%02x/0x%02x)=0x%02x/0x%02x, N1H/L(0x%02x/0x%02x)=0x%02x/0x%02x, "
                    "D1H/L(0x%02x/0x%02x)=0x%02x/0x%02x\n",
                    TLV320DAC3100_REG_DRC_LPF_N0H, TLV320DAC3100_REG_DRC_LPF_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DRC_LPF_N1H, TLV320DAC3100_REG_DRC_LPF_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DRC_LPF_D1H, TLV320DAC3100_REG_DRC_LPF_D1L, buf[4], buf[5]);
    }
    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_HPF_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P9:DRC_HPF N0H/L(0x%02x/0x%02x)=0x%02x/0x%02x, N1H/L(0x%02x/0x%02x)=0x%02x/0x%02x, "
                    "D1H/L(0x%02x/0x%02x)=0x%02x/0x%02x\n",
                    TLV320DAC3100_REG_DRC_HPF_N0H, TLV320DAC3100_REG_DRC_HPF_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DRC_HPF_N1H, TLV320DAC3100_REG_DRC_HPF_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DRC_HPF_D1H, TLV320DAC3100_REG_DRC_HPF_D1L, buf[4], buf[5]);
    }


    if (!setPage(13)) return;  // DRC C-RAM Buffer B

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_LPF_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P13:DRC_LPF N0H/L(0x%02x/0x%02x)=0x%02x/0x%02x, N1H/L(0x%02x/0x%02x)=0x%02x/0x%02x, "
                    "D1H/L(0x%02x/0x%02x)=0x%02x/0x%02x\n",
                    TLV320DAC3100_REG_DRC_LPF_N0H, TLV320DAC3100_REG_DRC_LPF_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DRC_LPF_N1H, TLV320DAC3100_REG_DRC_LPF_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DRC_LPF_D1H, TLV320DAC3100_REG_DRC_LPF_D1L, buf[4], buf[5]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DRC_HPF_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P13:DRC_HPF N0H/L(0x%02x/0x%02x)=0x%02x/0x%02x, N1H/L(0x%02x/0x%02x)=0x%02x/0x%02x, "
                    "D1H/L(0x%02x/0x%02x)=0x%02x/0x%02x\n",
                    TLV320DAC3100_REG_DRC_HPF_N0H, TLV320DAC3100_REG_DRC_HPF_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DRC_HPF_N1H, TLV320DAC3100_REG_DRC_HPF_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DRC_HPF_D1H, TLV320DAC3100_REG_DRC_HPF_D1L, buf[4], buf[5]);
    }

  }

  if (select & Px_IIR) {
    if (!setPage(9)) return;  // IIR C-RAM Buffer A

    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P9:RAM-A:LIIR:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D1H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LIIR_N0H, TLV320DAC3100_REG_DAC_LIIR_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LIIR_N1H, TLV320DAC3100_REG_DAC_LIIR_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LIIR_D1H, TLV320DAC3100_REG_DAC_LIIR_D1L, buf[4], buf[5]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P9:RAM-A:RIIR:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D1H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RIIR_N0H, TLV320DAC3100_REG_DAC_RIIR_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RIIR_N1H, TLV320DAC3100_REG_DAC_RIIR_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RIIR_D1H, TLV320DAC3100_REG_DAC_RIIR_D1L, buf[4], buf[5]);
    }

    if (!setPage(13)) return;  // IIR C-RAM Buffer B

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_LIIR_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P13:RAM-B:LIIR:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D1H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_LIIR_N0H, TLV320DAC3100_REG_DAC_LIIR_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_LIIR_N1H, TLV320DAC3100_REG_DAC_LIIR_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_LIIR_D1H, TLV320DAC3100_REG_DAC_LIIR_D1L, buf[4], buf[5]);
    }

    reg = Adafruit_BusIO_Register(i2c_dev, TLV320DAC3100_REG_DAC_RIIR_N0H);
    if (reg.read(buf, 6)) {
      Serial.printf("P13:RAM-B:RIIR:N0H/L(0x%02x/%02x)=0x%02x/%02x, N1H/L(0x%02x/%02x)=0x%02x/%02x, "
                    "D1H/L(0x%02x/%02x)=0x%02x/%02x\n",
                    TLV320DAC3100_REG_DAC_RIIR_N0H, TLV320DAC3100_REG_DAC_RIIR_N0L, buf[0], buf[1],
                    TLV320DAC3100_REG_DAC_RIIR_N1H, TLV320DAC3100_REG_DAC_RIIR_N1L, buf[2], buf[3],
                    TLV320DAC3100_REG_DAC_RIIR_D1H, TLV320DAC3100_REG_DAC_RIIR_D1L, buf[4], buf[5]);
    }
  }
}
#endif

