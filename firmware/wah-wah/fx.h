/* SPDX-License-Identifier: Apache-2.0 */
/*
 * VIBE FM - the automatic guitar effects engine.
 *
 * Chain (mono guitar in -> stereo out):
 *   noise gate -> compressor -> AUTO WAH -> drive -> 3-band EQ
 *   -> chorus / flanger / phaser -> ping-pong delay -> reverb -> LESLIE
 *
 * The wah is a self-driving filter: six controllers (LFO sweep, pre-gain
 * envelope, touch, talk, expression pedal / MIDI CC) feed three filter
 * voicings (CryBaby, Vox, Bass). The Leslie is a clean-room dual-rotor model:
 * independent horn/drum motors, an 800 Hz crossover, geometric Doppler with
 * three baffle taps per rotor and a stereo mic pair.
 *
 * Control flow publishes one coefficient snapshot through a seqlock. The audio
 * flow copies it and touches no shared parameter. All filter, delay and rotor
 * state is audio-only. No heap, no libc, no host calls from here.
 */
#ifndef VF_FX_H
#define VF_FX_H

#include "vibe.h"

/* Every value is scaled by 1000 (e.g. 2500 = 2.5 Hz, -70000 = -70 dB). */
typedef struct {
  int wah_mode, wah_speed, wah_depth, wah_res;
  int wah_low, wah_high, wah_attack, wah_release, wah_mix;
  int les_on, les_speed, les_ramp, les_balance, les_depth, les_drive;
  int drive, gate, comp;
  int eq_bass, eq_mid, eq_treble;
  int mod_type, mod_rate, mod_depth, mod_mix;
  int delay_mix, delay_ms, delay_fb;
  int reverb_mix, reverb_room;
  int master;
  /* Appended after the original ids: old DAW automation keeps its indices. */
  int wah_voicing, wah_shape, wah_sens, wah_hold, wah_drive;
} vf_fx_params;

typedef struct { float b0, b1, b2, a1, a2; } vf_bq;

typedef struct {
  int wah_mode, wah_voicing, wah_shape, les_on, les_speed, mod_type, gate_on, comp_on, eq_on;
  int delay_len, delay_on, reverb_on;
  float wah_inc, wah_depth, wah_q, wah_low, wah_high, wah_att, wah_rel, wah_mix;
  float wah_sens, wah_hold, wah_drive, wah_smooth, wah_ratio;
  float les_hz_h, les_hz_d;
  float les_horn_up, les_horn_down, les_horn_stop;
  float les_drum_up, les_drum_down, les_drum_stop;
  float les_wet_blend, les_balance, les_depth, les_drive;
  float gate_thr, gate_close, gate_att, gate_rel, gate_hold, gate_gain_att, gate_gain_rel;
  float comp_thr, comp_ratio, comp_knee, comp_att, comp_rel, comp_makeup;
  float drive_gain;
  vf_bq eq[3];
  float mod_inc, mod_base, mod_depth, mod_mix, mod_fb;
  float delay_fb, delay_mix;
  float rev_fb[4], rev_damp, rev_mix;
  float master;
} vf_coeff;

/* Control flow, once. */
void vf_fx_init(float sample_rate);
/* Control flow: publish a new snapshot. */
void vf_fx_prepare(const vf_fx_params *p);
/* Either flow: take a consistent copy. */
void vf_fx_copy(vf_coeff *dst);
/* Audio flow only: mono input, stereo output. */
void vf_fx_process(const float *in, float *out_l, float *out_r, int n, const vf_coeff *c);
/* Control flow: clear all effect memory. */
void vf_fx_reset(void);
/* Live readouts (value x100), written by the audio flow for the display. */
int vf_fx_wah_hz100(void);
int vf_fx_rotor_hz100(void);
/* Control flow: heel..toe from the expression jack or MIDI CC (0..1000). */
void vf_fx_set_pedal(int pos1000);

#endif
