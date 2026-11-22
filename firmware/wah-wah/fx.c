/* SPDX-License-Identifier: Apache-2.0 */
/*
 * VIBE FM effects engine. See fx.h for the signal path.
 *
 * The published snapshot is guarded by an even/odd generation counter. The
 * audio thread retries until it copies between two equal even generations.
 */
#include "fx.h"
#include "vibe_dsp.h"

#define VF_MOD_BUFSZ  4096
#define VF_DELAY_BUFSZ 65536
#define VF_ROTOR_BUFSZ 4096
#define VF_COMB_SZ    4096
#define VF_AP_SZ      512

/* Clean-room rotary geometry, in metres and metres/second. */
#define VF_LES_HORN_R 0.19f
#define VF_LES_DRUM_R 0.22f
#define VF_LES_MIC_X  0.24f
#define VF_LES_MIC_Y  0.42f
#define VF_LES_C      343.0f
#define VF_LES_TAPS   3

static float fx_sr = 48000.0f;
static int fx_gen;
static vf_coeff fx_cur;
static int wah_hz100, rotor_hz100;
static int expr_pos;                       /* control flow -> audio flow, 0..1000 */

/* ---- audio-only state ------------------------------------------------------- */
static vd_svf wah_svf, wah_formant;
static float wah_phase, wah_env, wah_follow, wah_touch, wah_hold_cnt;
static float wah_cut, wah_q_sm;
static int wah_arm;
static float gate_env, gate_gain, gate_hold_cnt;
static int gate_open;
static float comp_env;
static float eq_z[3][2];
static float mod_buf[VF_MOD_BUFSZ], mod_phase, ph_z[4];
static int mod_w;
static float delay_buf[VF_DELAY_BUFSZ];
static int delay_w;
static float comb_buf[4][VF_COMB_SZ], comb_lp[4];
static int comb_i[4];
static float ap_buf[2][VF_AP_SZ];
static int ap_i[2];
typedef struct { float phase, hz, target_hz, coef; } vf_les_rotor;
static vd_svf les_svf;
static vd_svf_c les_c;
static float les_hbuf[VF_ROTOR_BUFSZ], les_dbuf[VF_ROTOR_BUFSZ];
static int les_w;
static vf_les_rotor les_horn, les_drum;
static float les_horn_lp, les_drum_tone;
static float les_horn_hp_coef, les_drum_tone_coef;
static float les_base_delay, les_dist_scale, les_center_d;
static float les_wet;

/* Three taps spread across each rotor's baffle/reflection pattern. */
static const float les_tap_sin[VF_LES_TAPS]  = { -0.482f, 0.0f, 0.482f };
static const float les_tap_cos[VF_LES_TAPS]  = {  0.876f, 1.0f, 0.876f };
static const float les_tap_gain[VF_LES_TAPS] = {  0.22f, 0.56f, 0.22f };

static float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static float q1000(int v) { return (float)v * 0.001f; }
static int qmode(int v, int max) { return (int)(clampf(q1000(v), 0.0f, (float)max) + 0.5f); }
static float db2lin(float db) { return vd_exp2(db * 0.1660964f); }
static float tc(float ms, float sr) { return 1.0f - vd_exp2(-1.442695f / (ms * 0.001f * sr)); }
/* vd_coef() returns the pole (a retainer); a one-pole blend is 1 - pole. */
static float blend(float seconds, float sr) { return 1.0f - vd_coef(seconds, sr); }
/* Rotor Ramp scales the BAR-FLY motor times; 1.5 s is the baseline. */
static float les_motor_blend(float base, float scale)
{
  float t = clampf(base * scale, 0.01f, 12.0f);
  return blend(t, fx_sr);
}

/* RBJ cookbook; vd_sin() takes phase in turns, so sin(w + .25) is cos(w). */
static void bq_set(vf_bq *c, int type, float hz, float q, float db, float sr)
{
  float w = hz / sr, cw = vd_sin(w + 0.25f), sw = vd_sin(w);
  float A = vd_exp2(db * 0.0830482f), al = sw / (2.0f * q);
  float sq = 2.0f * vd_sqrt(A) * al;
  float b0, b1, b2, a0, a1, a2;
  if (type == 1) {
    b0 = A * ((A + 1) - (A - 1) * cw + sq);
    b1 = 2 * A * ((A - 1) - (A + 1) * cw);
    b2 = A * ((A + 1) - (A - 1) * cw - sq);
    a0 = (A + 1) + (A - 1) * cw + sq;
    a1 = -2 * ((A - 1) + (A + 1) * cw);
    a2 = (A + 1) + (A - 1) * cw - sq;
  } else if (type == 2) {
    b0 = 1 + al * A; b1 = -2 * cw; b2 = 1 - al * A;
    a0 = 1 + al / A; a1 = -2 * cw; a2 = 1 - al / A;
  } else {
    b0 = A * ((A + 1) + (A - 1) * cw + sq);
    b1 = -2 * A * ((A - 1) - (A + 1) * cw);
    b2 = A * ((A + 1) + (A - 1) * cw - sq);
    a0 = (A + 1) - (A - 1) * cw + sq;
    a1 = 2 * ((A - 1) + (A + 1) * cw);
    a2 = (A + 1) - (A - 1) * cw - sq;
  }
  c->b0 = b0 / a0; c->b1 = b1 / a0; c->b2 = b2 / a0;
  c->a1 = a1 / a0; c->a2 = a2 / a0;
}

void vf_fx_init(float sample_rate)
{
  fx_sr = sample_rate > 1000.0f ? sample_rate : 48000.0f;
  __atomic_store_n(&fx_gen, 0, __ATOMIC_RELEASE);
  __atomic_store_n(&expr_pos, 500, __ATOMIC_RELEASE);
  vd_svf_set(&les_c, 800.0f, 0.7071f, fx_sr);
  /* Retention coefficients: ~140 Hz horn DC block, ~2.6 kHz drum voicing. */
  les_horn_hp_coef = vd_coef(1.0f / (VD_TAU * 140.0f), fx_sr);
  les_drum_tone_coef = vd_coef(1.0f / (VD_TAU * 2600.0f), fx_sr);
  les_base_delay = 0.004f * fx_sr;
  les_dist_scale = fx_sr / VF_LES_C;
  les_center_d = vd_sqrt(VF_LES_MIC_X * VF_LES_MIC_X + VF_LES_MIC_Y * VF_LES_MIC_Y);
  vf_fx_reset();
}

void vf_fx_set_pedal(int pos1000)
{
  int v = pos1000 < 0 ? 0 : (pos1000 > 1000 ? 1000 : pos1000);
  __atomic_store_n(&expr_pos, v, __ATOMIC_RELEASE);
}

void vf_fx_reset(void)
{
  int i, k;
  wah_svf.ic1 = wah_svf.ic2 = 0.0f;
  wah_formant.ic1 = wah_formant.ic2 = 0.0f;
  wah_phase = wah_env = wah_follow = wah_touch = wah_hold_cnt = 0.0f;
  wah_cut = 1000.0f;
  wah_q_sm = 5.0f;
  wah_arm = 0;
  gate_env = comp_env = 0.0f;
  gate_gain = 1.0f;
  gate_open = 0;
  gate_hold_cnt = 0.0f;
  for (k = 0; k < 3; k++) eq_z[k][0] = eq_z[k][1] = 0.0f;
  mod_phase = 0.0f;
  mod_w = 0;
  for (i = 0; i < VF_MOD_BUFSZ; i++) mod_buf[i] = 0.0f;
  for (k = 0; k < 4; k++) ph_z[k] = 0.0f;
  delay_w = 0;
  for (i = 0; i < VF_DELAY_BUFSZ; i++) delay_buf[i] = 0.0f;
  for (k = 0; k < 4; k++) {
    comb_lp[k] = 0.0f;
    comb_i[k] = 0;
    for (i = 0; i < VF_COMB_SZ; i++) comb_buf[k][i] = 0.0f;
  }
  for (k = 0; k < 2; k++) {
    ap_i[k] = 0;
    for (i = 0; i < VF_AP_SZ; i++) ap_buf[k][i] = 0.0f;
  }
  les_svf.ic1 = les_svf.ic2 = 0.0f;
  les_horn_lp = les_drum_tone = 0.0f;
  les_w = 0;
  les_horn.phase = les_drum.phase = 0.0f;
  les_horn.hz = les_drum.hz = 0.0f;
  les_horn.target_hz = les_drum.target_hz = 0.0f;
  les_horn.coef = les_drum.coef = 0.0f;
  les_wet = 0.0f;
  for (i = 0; i < VF_ROTOR_BUFSZ; i++) les_hbuf[i] = les_dbuf[i] = 0.0f;
  __atomic_store_n(&wah_hz100, 0, __ATOMIC_RELEASE);
  __atomic_store_n(&rotor_hz100, 0, __ATOMIC_RELEASE);
}

void vf_fx_prepare(const vf_fx_params *p)
{
  vf_coeff *c = &fx_cur;
  static const float EQ_HZ[3] = { 150.0f, 1000.0f, 6000.0f };
  static const float EQ_Q[3]  = { 0.7071f, 0.7f, 0.7071f };
  float amt, mix, db;
  int k, mt;

  if (!p) return;
  __atomic_fetch_add(&fx_gen, 1, __ATOMIC_RELEASE);

  c->master = clampf(q1000(p->master), 0.0f, 1.0f);

  /* ---- automatic wah ------------------------------------------------------ */
  c->wah_mode = qmode(p->wah_mode, 5);
  c->wah_voicing = qmode(p->wah_voicing, 2);
  c->wah_shape = qmode(p->wah_shape, 3);
  c->wah_inc = q1000(p->wah_speed) / fx_sr;
  c->wah_depth = clampf(q1000(p->wah_depth), 0.0f, 1.0f);
  c->wah_q = clampf(q1000(p->wah_res), 0.5f, 16.0f);
  c->wah_low = clampf(q1000(p->wah_low), 80.0f, 4000.0f);
  c->wah_high = clampf(q1000(p->wah_high), c->wah_low + 50.0f, 12000.0f);
  c->wah_ratio = vd_log2(c->wah_high / c->wah_low);
  c->wah_att = blend(clampf(q1000(p->wah_attack) * 0.001f, 0.0005f, 0.5f), fx_sr);
  c->wah_rel = blend(clampf(q1000(p->wah_release) * 0.001f, 0.005f, 2.0f), fx_sr);
  c->wah_sens = clampf(q1000(p->wah_sens), 0.0f, 1.0f);
  c->wah_hold = (float)(int)(clampf(q1000(p->wah_hold), 0.0f, 500.0f) * 0.001f * fx_sr);
  c->wah_drive = clampf(q1000(p->wah_drive), 0.0f, 1.0f);
  c->wah_mix = clampf(q1000(p->wah_mix), 0.0f, 1.0f);
  /* ~2.5 ms log-domain dezipper: the pot sweep never clicks or zippers. */
  c->wah_smooth = blend(0.0025f, fx_sr);

  /* ---- Leslie: independent horn/drum motors and geometric Doppler --------- */
  {
    float ramp = clampf(q1000(p->les_ramp), 0.05f, 10.0f) / 1.5f;
    c->les_on = p->les_on ? 1 : 0;
    c->les_speed = qmode(p->les_speed, 2);
    if (c->les_on && c->les_speed == 2) {
      c->les_hz_h = 7.0f; c->les_hz_d = 6.0f;
    } else if (c->les_on && c->les_speed == 1) {
      c->les_hz_h = 0.90f; c->les_hz_d = 0.80f;
    } else {
      c->les_hz_h = c->les_hz_d = 0.0f;
    }
    /* Spin-up, slow/brake and stop have separate time constants per motor. */
    c->les_horn_up   = les_motor_blend(0.22f, ramp);
    c->les_horn_down = les_motor_blend(0.38f, ramp);
    c->les_horn_stop = les_motor_blend(0.55f, ramp);
    c->les_drum_up   = les_motor_blend(1.10f, ramp);
    c->les_drum_down = les_motor_blend(0.65f, ramp);
    c->les_drum_stop = les_motor_blend(1.40f, ramp);
    c->les_wet_blend = blend(0.025f, fx_sr);
    c->les_balance = clampf(q1000(p->les_balance), 0.0f, 1.0f);
    c->les_depth = 0.12f + 0.88f * clampf(q1000(p->les_depth), 0.0f, 1.0f);
    c->les_drive = clampf(q1000(p->les_drive), 0.0f, 1.0f);
  }

  /* ---- gate and compressor ------------------------------------------------ */
  db = q1000(p->gate);
  c->gate_on = db > -79.9f;
  if (c->gate_on) {
    c->gate_thr = db2lin(db);
    c->gate_close = c->gate_thr * 0.6f;
    c->gate_att = tc(2.0f, fx_sr);
    c->gate_rel = tc(120.0f, fx_sr);
    c->gate_hold = (float)(int)(fx_sr * 0.08f);
    c->gate_gain_att = tc(1.0f, fx_sr);
    c->gate_gain_rel = tc(60.0f, fx_sr);
  }

  amt = clampf(q1000(p->comp), 0.0f, 1.0f);
  c->comp_on = amt > 0.001f;
  if (c->comp_on) {
    c->comp_thr = -6.0f - 30.0f * amt;
    c->comp_ratio = 2.0f + 8.0f * amt;
    c->comp_knee = 6.0f;
    c->comp_att = tc(5.0f, fx_sr);
    c->comp_rel = tc(150.0f, fx_sr);
    c->comp_makeup = db2lin(0.5f * (1.0f - 1.0f / c->comp_ratio) * (-c->comp_thr));
  }

  c->drive_gain = 1.0f + 24.0f * clampf(q1000(p->drive), 0.0f, 1.0f);

  c->eq_on = (p->eq_bass != 0 || p->eq_mid != 0 || p->eq_treble != 0);
  if (c->eq_on) {
    float e[3] = { q1000(p->eq_bass), q1000(p->eq_mid), q1000(p->eq_treble) };
    for (k = 0; k < 3; k++)
      bq_set(&c->eq[k], k == 1 ? 2 : (k == 0 ? 1 : 3), EQ_HZ[k], EQ_Q[k], e[k], fx_sr);
  }

  /* ---- modulation --------------------------------------------------------- */
  mt = qmode(p->mod_type, 3);
  c->mod_type = mt;
  if (mt) {
    float rate = 0.05f + clampf(q1000(p->mod_rate), 0.0f, 1.0f) * 7.95f;
    float depth = clampf(q1000(p->mod_depth), 0.0f, 1.0f);
    c->mod_inc = rate / fx_sr;
    c->mod_mix = clampf(q1000(p->mod_mix), 0.0f, 1.0f);
    c->mod_depth = depth;
    if (mt == 1) {           /* chorus: 12..24 ms */
      c->mod_base = 0.018f * fx_sr;
      c->mod_depth = 0.006f * fx_sr * depth;
      c->mod_fb = 0.0f;
    } else if (mt == 2) {    /* flanger: 0.5..3.5 ms, a little feedback */
      c->mod_base = 0.003f * fx_sr;
      c->mod_depth = 0.003f * fx_sr * depth;
      c->mod_fb = 0.6f * depth;
    } else {                 /* phaser: four modulated allpass stages */
      c->mod_base = 0.0f;
      c->mod_fb = 0.0f;
    }
    c->mod_base = clampf(c->mod_base, 0.0f, (float)VF_MOD_BUFSZ - 8.0f);
    c->mod_depth = clampf(c->mod_depth, 0.0f, (float)VF_MOD_BUFSZ - 8.0f - c->mod_base);
  }

  /* ---- delay -------------------------------------------------------------- */
  mix = clampf(q1000(p->delay_mix), 0.0f, 1.0f);
  c->delay_mix = mix;
  c->delay_len = (int)(clampf(q1000(p->delay_ms), 0.0f, 1400.0f) * 0.001f * fx_sr + 0.5f);
  if (c->delay_len < 1) c->delay_len = 1;
  if (c->delay_len > VF_DELAY_BUFSZ - 2) c->delay_len = VF_DELAY_BUFSZ - 2;
  c->delay_fb = clampf(q1000(p->delay_fb), 0.0f, 0.92f);
  c->delay_on = mix > 0.001f;

  /* ---- reverb ------------------------------------------------------------- */
  mix = clampf(q1000(p->reverb_mix), 0.0f, 1.0f);
  c->rev_mix = mix;
  c->reverb_on = mix > 0.001f;
  if (c->reverb_on) {
    static const float DECAY[4] = { 0.98f, 0.96f, 0.94f, 0.92f };
    float room = clampf(q1000(p->reverb_room), 0.0f, 1.0f);
    float fb = 0.70f + 0.25f * room;
    c->rev_damp = 0.15f + 0.55f * room;
    for (k = 0; k < 4; k++) c->rev_fb[k] = fb * DECAY[k];
  }

  __atomic_fetch_add(&fx_gen, 1, __ATOMIC_RELEASE);
}

void vf_fx_copy(vf_coeff *dst)
{
  int g1, g2;
  if (!dst) return;
  for (;;) {
    g1 = __atomic_load_n(&fx_gen, __ATOMIC_ACQUIRE);
    if (g1 & 1) continue;
    *dst = fx_cur;
    g2 = __atomic_load_n(&fx_gen, __ATOMIC_ACQUIRE);
    if (g1 == g2) return;
  }
}

int vf_fx_wah_hz100(void) { return __atomic_load_n(&wah_hz100, __ATOMIC_ACQUIRE); }
int vf_fx_rotor_hz100(void) { return __atomic_load_n(&rotor_hz100, __ATOMIC_ACQUIRE); }

static inline float bq_tick(const vf_bq *c, float *z, float x)
{
  float y = c->b0 * x + z[0];
  z[0] = c->b1 * x - c->a1 * y + z[1];
  z[1] = c->b2 * x - c->a2 * y;
  return y;
}

/* Fractional read from a rotor delay line. */
static float les_line_read(const float *buf, float back)
{
  int base, r;
  float frac, a, b;
  if (back < 0.0f) back = 0.0f;
  if (back > (float)VF_ROTOR_BUFSZ - 2.0f) back = (float)VF_ROTOR_BUFSZ - 2.0f;
  base = (int)back;
  r = les_w - base;
  frac = back - (float)base;
  while (r < 0) r += VF_ROTOR_BUFSZ;
  while (r >= VF_ROTOR_BUFSZ) r -= VF_ROTOR_BUFSZ;
  b = buf[r];
  r--; if (r < 0) r += VF_ROTOR_BUFSZ;
  a = buf[r];
  return a * frac + b * (1.0f - frac);
}

/* Separate spin-up, brake and stop blends for the two motors. */
static void les_motor_set(vf_les_rotor *q, float target,
                          float up, float down, float stop)
{
  q->target_hz = target;
  if (target <= 0.0001f) q->coef = stop;
  else if (target > q->hz + 0.001f) q->coef = up;
  else if (target < q->hz - 0.001f) q->coef = down;
  else q->coef = up;              /* at target: either blend leaves hz unchanged */
}

static void les_motor_tick(vf_les_rotor *q, float sr)
{
  q->hz += (q->target_hz - q->hz) * q->coef;
  if (q->target_hz <= 0.0001f && q->hz < 0.002f) q->hz = 0.0f;
  if (q->hz < 0.0f) q->hz = 0.0f;
  q->phase += q->hz / sr;
  if (q->phase >= 1.0f) q->phase -= 1.0f;
}

/* One rotor heard by one microphone: three baffle taps with geometry-based
 * path length, Doppler and inverse-distance/directivity gain. */
static float les_rotor_read(const float *buf, float phase, float radius,
                            float mic_x, float depth)
{
  float s = vd_sin(phase), c = vd_sin(phase + 0.25f);
  float out = 0.0f;
  int k;
  for (k = 0; k < VF_LES_TAPS; k++) {
    float ts = s * les_tap_cos[k] + c * les_tap_sin[k];
    float tc = c * les_tap_cos[k] - s * les_tap_sin[k];
    float dx = mic_x - radius * tc;
    float dy = VF_LES_MIC_Y - radius * ts;
    float d = vd_sqrt(dx * dx + dy * dy);
    float path, tap, dot, dir;
    if (d < 0.001f) d = 0.001f;
    path = (d - les_center_d) * les_dist_scale * depth;
    tap = les_line_read(buf, les_base_delay + path);
    dot = (c * dx + s * dy) / d;
    dir = 0.50f + 0.50f * dot;
    if (dir < 0.08f) dir = 0.08f;
    out += tap * les_tap_gain[k] * dir * (les_center_d / d);
  }
  return out;
}

static float safe(float v)
{
  if (!(v > -1.0e20f && v < 1.0e20f)) return 0.0f;
  return v;
}

void vf_fx_process(const float *in, float *out_l, float *out_r, int n, const vf_coeff *c)
{
  int i, k;
  if (!out_l || !out_r || !c || n <= 0) return;

  for (i = 0; i < n; i++) {
    float x = in ? in[i] : 0.0f;
    float l, r;
    vd_svf_out wo, co, fo;
    float a, target;

    /* ---- gate ------------------------------------------------------------ */
    if (c->gate_on) {
      a = vd_absf(x);
      gate_env += (a - gate_env) * (a > gate_env ? c->gate_att : c->gate_rel);
      if (!gate_open && gate_env > c->gate_thr) {
        gate_open = 1;
        gate_hold_cnt = c->gate_hold;
      }
      if (gate_open) {
        if (gate_env > c->gate_close) gate_hold_cnt = c->gate_hold;
        else if (gate_hold_cnt > 0.0f) gate_hold_cnt -= 1.0f;
        else gate_open = 0;
      }
      target = gate_open ? 1.0f : 0.0f;
      gate_gain += (target - gate_gain) * (target > gate_gain ? c->gate_gain_att : c->gate_gain_rel);
      x *= gate_gain;
    } else {
      gate_env = gate_gain = 0.0f;
      gate_gain = 1.0f;
      gate_open = 0;
      gate_hold_cnt = 0.0f;
    }

    /* ---- compressor ------------------------------------------------------ */
    if (c->comp_on) {
      float db, over, gr;
      a = vd_absf(x);
      comp_env += (a - comp_env) * (a > comp_env ? c->comp_att : c->comp_rel);
      db = 3.0103f * vd_log2(comp_env + 1e-9f);
      over = db - c->comp_thr;
      if (over <= -c->comp_knee * 0.5f) gr = 0.0f;
      else if (over < c->comp_knee * 0.5f) {
        float t = over + c->comp_knee * 0.5f;
        gr = (1.0f - 1.0f / c->comp_ratio) * t * t / (2.0f * c->comp_knee);
      } else gr = (1.0f - 1.0f / c->comp_ratio) * over;
      x *= db2lin(-gr) * c->comp_makeup;
    } else comp_env = 0.0f;

    /* ---- automatic wah --------------------------------------------------- */
    {
      float pos, p, cut_t, q, bp, lp, wet;
      int mode = c->wah_mode;
      a = vd_absf(x);

      if (mode == 1 || mode == 4) {
        /* LFO: sine / triangle / saw / square, with an excursion control. */
        float s;
        wah_phase += c->wah_inc * (mode == 4 ? 1.7f : 1.0f);
        if (wah_phase >= 1.0f) wah_phase -= 1.0f;
        if (c->wah_shape == 1) s = 1.0f - vd_absf(2.0f * wah_phase - 1.0f);
        else if (c->wah_shape == 2) s = wah_phase;
        else if (c->wah_shape == 3) s = wah_phase < 0.5f ? 0.0f : 1.0f;
        else s = 0.5f + 0.5f * vd_sin(wah_phase);
        pos = 0.5f + (s - 0.5f) * c->wah_depth;
      } else if (mode == 2 || mode == 3) {
        /* Peak detector in dB, so quiet and loud players both get a sweep. */
        float db, u;
        wah_env += (a - wah_env) * (a > wah_env ? c->wah_att : c->wah_rel);
        db = 3.0103f * vd_log2(wah_env + 1.0e-7f);
        u = clampf((db - (-54.0f + 24.0f * c->wah_sens)) / 42.0f, 0.0f, 1.0f);
        if (mode == 2) {
          /* Auto wah: fast to the note, hold, then fall back to the heel. */
          if (u > wah_follow) {
            wah_follow += (u - wah_follow) * c->wah_att;
            wah_hold_cnt = c->wah_hold;
          } else if (wah_hold_cnt > 0.0f) wah_hold_cnt -= 1.0f;
          else wah_follow += (u - wah_follow) * c->wah_rel;
          pos = 0.06f + 0.94f * wah_follow;
        } else {
          /* Touch wah: one toe kick per picked note, then decay. */
          if (!wah_arm && u > 0.30f && u > wah_follow + 0.10f) {
            wah_touch = 1.0f;
            wah_arm = 1;
          }
          if (u < 0.12f) wah_arm = 0;
          wah_follow += (u - wah_follow) * c->wah_rel;
          wah_touch += (0.0f - wah_touch) * c->wah_rel;
          pos = 0.06f + 0.94f * wah_touch;
        }
      } else if (mode == 5) {
        float target = (float)__atomic_load_n(&expr_pos, __ATOMIC_ACQUIRE) * 0.001f;
        wah_follow += (target - wah_follow) * c->wah_att;
        pos = wah_follow;
      } else {
        pos = 0.5f;
        wah_phase = wah_env = wah_follow = wah_touch = wah_hold_cnt = 0.0f;
      }
      pos = clampf(pos, 0.0f, 1.0f);

      /* Pot curve and resonance: heel is resonant, toe opens up. */
      if (c->wah_voicing == 0) p = pos * pos * (3.0f - 2.0f * pos);
      else if (c->wah_voicing == 2) p = pos * (0.6f + 0.4f * pos);
      else p = pos;
      cut_t = clampf(c->wah_low * vd_exp2(p * c->wah_ratio), 40.0f, fx_sr * 0.45f);
      q = clampf(c->wah_q * (c->wah_voicing == 1 ? (0.85f + 0.25f * pos)
                                                : (1.5f - 0.7f * pos)), 0.6f, 18.0f);
      wah_cut += (cut_t - wah_cut) * c->wah_smooth;
      wah_q_sm += (q - wah_q_sm) * c->wah_smooth;
      __atomic_store_n(&wah_hz100, (int)(wah_cut * 100.0f), __ATOMIC_RELEASE);

      wo = vd_svf_run(&wah_svf, x, wah_cut, wah_q_sm, fx_sr);
      bp = wo.bp / wah_q_sm;                /* the TPT SVF peaks at Q */
      lp = wo.lp / wah_q_sm;
      if (mode == 4) {
        /* Talk: a second, higher formant rides on the swept band. */
        float f2 = clampf(wah_cut * 1.8f + 120.0f, 40.0f, fx_sr * 0.45f);
        float q2 = vd_maxf(wah_q_sm * 0.7f, 0.6f);
        fo = vd_svf_run(&wah_formant, x, f2, q2, fx_sr);
        wet = bp * (1.35f + 0.55f * p) + (fo.bp / q2) * 0.75f;
      } else {
        wet = bp * (c->wah_voicing == 1 ? (1.15f + 0.65f * p) : (1.5f + 0.8f * p));
        wet += lp * (c->wah_voicing == 1 ? 0.30f : 0.12f);
      }
      if (mode != 0) x = x * (1.0f - c->wah_mix) + wet * c->wah_mix;
      x = vd_tanh(x * (1.0f + 5.0f * c->wah_drive));
    }

    /* ---- drive ----------------------------------------------------------- */
    x = vd_tanh(x * c->drive_gain);

    /* ---- EQ -------------------------------------------------------------- */
    if (c->eq_on) {
      for (k = 0; k < 3; k++) x = bq_tick(&c->eq[k], eq_z[k], x);
    } else {
      for (k = 0; k < 3; k++) eq_z[k][0] = eq_z[k][1] = 0.0f;
    }

    /* ---- chorus / flanger / phaser --------------------------------------- */
    if (c->mod_type) {
      mod_phase += c->mod_inc;
      if (mod_phase >= 1.0f) mod_phase -= 1.0f;
      if (c->mod_type == 3) {
        float y = x, cc = 0.55f * vd_sin(mod_phase), t;
        for (k = 0; k < 4; k++) {
          t = cc * y + ph_z[k];
          ph_z[k] = y - cc * t;
          y = t;
        }
        x = x * (1.0f - c->mod_mix) + y * c->mod_mix;
      } else {
        int rr = mod_w - (int)(c->mod_base + c->mod_depth * vd_sin(mod_phase));
        float d;
        if (rr < 0) rr += VF_MOD_BUFSZ;
        d = mod_buf[rr];
        mod_buf[mod_w] = x + d * c->mod_fb;
        mod_w++;
        if (mod_w >= VF_MOD_BUFSZ) mod_w = 0;
        x = x * (1.0f - c->mod_mix) + d * c->mod_mix;
      }
    } else {
      mod_phase = 0.0f;
      for (k = 0; k < 4; k++) ph_z[k] = 0.0f;
    }

    /* ---- ping-pong style delay ------------------------------------------- */
    if (c->delay_on) {
      int rr = delay_w - c->delay_len;
      float d;
      if (rr < 0) rr += VF_DELAY_BUFSZ;
      d = delay_buf[rr];
      delay_buf[delay_w] = x + d * c->delay_fb;
      delay_w++;
      if (delay_w >= VF_DELAY_BUFSZ) delay_w = 0;
      x = x * (1.0f - c->delay_mix) + d * c->delay_mix;
    }

    /* ---- reverb ---------------------------------------------------------- */
    if (c->reverb_on) {
      float rv = 0.0f;
      for (k = 0; k < 4; k++) {
        float y = comb_buf[k][comb_i[k]];
        comb_lp[k] = y * (1.0f - c->rev_damp) + comb_lp[k] * c->rev_damp;
        comb_buf[k][comb_i[k]] = x + comb_lp[k] * c->rev_fb[k];
        comb_i[k]++;
        if (comb_i[k] >= VF_COMB_SZ) comb_i[k] = 0;
        rv += y;
      }
      rv *= 0.25f;
      for (k = 0; k < 2; k++) {
        float y = ap_buf[k][ap_i[k]];
        ap_buf[k][ap_i[k]] = rv + y * 0.5f;
        ap_i[k]++;
        if (ap_i[k] >= VF_AP_SZ) ap_i[k] = 0;
        rv = y - rv;
      }
      x = x * (1.0f - c->rev_mix) + rv * c->rev_mix;
    }

    /* ---- Leslie: dual-rotor model with stereo mics ------------------------ */
    les_motor_set(&les_horn, c->les_hz_h, c->les_horn_up, c->les_horn_down, c->les_horn_stop);
    les_motor_set(&les_drum, c->les_hz_d, c->les_drum_up, c->les_drum_down, c->les_drum_stop);
    /* Motors keep coasting while the wet mix fades out. */
    les_motor_tick(&les_horn, fx_sr);
    les_motor_tick(&les_drum, fx_sr);

    if (!c->les_on && les_wet <= 0.0005f) {
      l = r = x;
    } else {
      float horn_gain = 0.55f + 0.90f * c->les_balance;
      float drum_gain = 0.55f + 0.90f * (1.0f - c->les_balance);
      float horn, drum, lpn, hl, hr, bl, br, wet_l, wet_r;

      co = vd_svf_tick(&les_svf, &les_c, x);
      lpn = les_horn_lp + (co.hp - les_horn_lp) * les_horn_hp_coef;
      horn = co.hp - lpn;
      les_horn_lp = lpn;
      les_drum_tone += (co.lp - les_drum_tone) * les_drum_tone_coef;
      drum = les_drum_tone;

      if (c->les_drive > 0.0f)
        horn = vd_tanh(horn * (1.0f + 5.0f * c->les_drive));

      les_hbuf[les_w] = horn;
      les_dbuf[les_w] = drum;
      les_w++;
      if (les_w >= VF_ROTOR_BUFSZ) les_w = 0;

      hl = les_rotor_read(les_hbuf, les_horn.phase, VF_LES_HORN_R,
                          -VF_LES_MIC_X, c->les_depth) * horn_gain;
      hr = les_rotor_read(les_hbuf, les_horn.phase, VF_LES_HORN_R,
                           VF_LES_MIC_X, c->les_depth) * horn_gain;
      bl = les_rotor_read(les_dbuf, les_drum.phase, VF_LES_DRUM_R,
                          -VF_LES_MIC_X, c->les_depth) * drum_gain;
      br = les_rotor_read(les_dbuf, les_drum.phase, VF_LES_DRUM_R,
                           VF_LES_MIC_X, c->les_depth) * drum_gain;

      wet_l = (hl + bl) * 0.90f + x * 0.045f;
      wet_r = (hr + br) * 0.90f + x * 0.045f;
      les_wet += ((c->les_on ? 1.0f : 0.0f) - les_wet) * c->les_wet_blend;
      if (les_wet > 1.0f) les_wet = 1.0f;
      l = x + (wet_l - x) * les_wet;
      r = x + (wet_r - x) * les_wet;
    }

    l = vd_tanh(l * c->master);
    r = vd_tanh(r * c->master);
    out_l[i] = safe(l);
    out_r[i] = safe(r);
  }
  __atomic_store_n(&rotor_hz100, (int)(les_horn.hz * 100.0f + 0.5f), __ATOMIC_RELEASE);
}
