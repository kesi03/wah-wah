/* SPDX-License-Identifier: Apache-2.0 */
/*
 * VIBE FM - automatic funk guitar effects.
 *
 * This is an EFFECT, not a synth: IN 1 is the guitar signal. For a quick demo,
 * the 25 keys pluck a tiny Karplus-Strong guitar INSIDE the firmware, so the wah
 * and the Leslie can be heard without plugging anything in. The riff is off
 * until you press the DEMO button. The keys are only a showcase - live input
 * runs through the very same chain.
 *
 * Control flow: parameters, presets, keys, display, editor.
 * Audio flow: note queue, demo guitar, coefficient snapshot, fx_process().
 */
#include "vibe.h"
#include "vibe_gfx.h"
#include "vibe_dsp.h"
#include "fx.h"

VIBE_PARALLEL_AUDIO

#define ED_CHUNK 4096
#define QN 64
#define KS_VOICES 8
#define KS_BUFSZ 2048

enum { UI_WAH, UI_FX, UI_TONE, UI_PRESETS, UI_N };

/* DAW / Editor parameter ids - stable, never rename. */
enum {
  P_WAH_MODE = 0, P_WAH_SPEED, P_WAH_DEPTH, P_WAH_RES, P_WAH_LOW, P_WAH_HIGH,
  P_WAH_ATTACK, P_WAH_RELEASE, P_WAH_MIX,
  P_LESLIE_ON, P_LESLIE_SPEED, P_LESLIE_RAMP, P_LESLIE_BALANCE, P_LESLIE_DEPTH,
  P_LESLIE_DRIVE,
  P_DRIVE, P_GATE, P_COMP,
  P_EQ_BASS, P_EQ_MID, P_EQ_TREBLE,
  P_MOD_TYPE, P_MOD_RATE, P_MOD_DEPTH, P_MOD_MIX,
  P_DELAY_MIX, P_DELAY_MS, P_DELAY_FB,
  P_REVERB_MIX, P_REVERB_ROOM,
  P_MASTER, P_DEMO_RIFF, P_DEMO_BPM,
  /* appended: the ids above keep their index for saved DAW automation */
  P_WAH_VOICING, P_WAH_SHAPE, P_WAH_SENS, P_WAH_HOLD, P_WAH_DRIVE,
  P_COUNT
};

/* Warm 70s rack: espresso, burnt orange, mustard, avocado, cream. */
#define C_BG     0x140D0A
#define C_PANEL  0x241812
#define C_PANEL2 0x35231A
#define C_ACC    0xFF8C1A
#define C_GOLD   0xF2C14E
#define C_AVO    0x8FA44B
#define C_CREAM  0xF3E5C8
#define C_FG     0xFFF3DC
#define C_MUTED  0xB39478
#define C_DIM    0x6E5847
#define C_GREEN  0x9BCB63
#define C_RED    0xD95B3F

static float sr = 48000.0f;
static int g_params[P_COUNT];                 /* value x1000, atomic, single writer */
static float ui_vals[P_COUNT];                /* control-flow copy for the display */
static int ui_mode = UI_WAH, help, preset = 0, wah_last_mode = 1;
static char status[96];
static int draw_tick;
static char ed_status[192];
static int ed_len, ed_pending;

/* ---- note queue: control flow writes, audio flow reads ---------------------- */
static int q_note[QN], q_vel[QN], q_on[QN], q_head, q_tail;

/* ---- demo guitar: audio-only Karplus-Strong voices -------------------------- */
static float ks_buf[KS_VOICES][KS_BUFSZ];
static int ks_len[KS_VOICES], ks_idx[KS_VOICES], ks_age[KS_VOICES];
static float ks_amp[KS_VOICES], ks_damp[KS_VOICES];
static int riff_clock, riff_step;
static float mono[VIBE_MAX_BLOCK];
static const signed char RIFF[16] = {
   0, 0, 3, 0,  7, 3, 0, -2,  0, 3, 0, 7,  5, 3, 0, -2
};

static const char *const WAH_MODES[] = { "OFF", "SWEEP", "ENVELOPE", "TOUCH", "TALK", "PEDAL" };
static const char *const WAH_VOICINGS[] = { "CRYBABY", "VOX", "BASS" };
static const char *const WAH_SHAPES[] = { "SINE", "TRIANGLE", "SAW", "SQUARE" };
static const char *const LES_SPEEDS[] = { "STOP", "SLOW", "FAST" };
static const char *const ON_OFF[] = { "OFF", "ON" };
static const char *const MOD_KINDS[] = { "OFF", "CHORUS", "FLANGER", "PHASER" };

#define PV(id,name,min,max,def,unit) { id, name, min, max, def, unit, 0, 0 }
static const vibe_param PARAMS[P_COUNT] = {
  /* A - WAH */
  { "wah_mode", "Wah Mode", 0.0f, 5.0f, 1.0f, "", 6, WAH_MODES },
  PV("wah_speed", "Wah Speed", 0.1f, 10.0f, 2.4f, "Hz"),
  PV("wah_depth", "Wah Depth", 0.0f, 1.0f, 0.9f, ""),
  PV("wah_res", "Wah Resonance", 0.7f, 12.0f, 5.5f, ""),
  PV("wah_low", "Wah Low", 100.0f, 1500.0f, 330.0f, "Hz"),
  PV("wah_high", "Wah High", 800.0f, 7000.0f, 3400.0f, "Hz"),
  PV("wah_attack", "Env Attack", 1.0f, 200.0f, 12.0f, "ms"),
  PV("wah_release", "Env Release", 20.0f, 1000.0f, 180.0f, "ms"),
  PV("wah_mix", "Wah Mix", 0.0f, 1.0f, 0.9f, ""),
  /* B - LESLIE */
  { "leslie_on", "Leslie", 0.0f, 1.0f, 0.0f, "", 2, ON_OFF },
  { "leslie_speed", "Rotor Speed", 0.0f, 2.0f, 1.0f, "", 3, LES_SPEEDS },
  PV("leslie_ramp", "Rotor Ramp", 0.05f, 10.0f, 1.5f, "s"),
  PV("leslie_balance", "Horn / Bass", 0.0f, 1.0f, 0.55f, ""),
  PV("leslie_depth", "Rotor Depth", 0.0f, 1.0f, 0.75f, ""),
  PV("leslie_drive", "Horn Drive", 0.0f, 1.0f, 0.25f, ""),
  /* C - DRIVE */
  PV("drive", "Drive", 0.0f, 1.0f, 0.22f, ""),
  PV("gate", "Gate Threshold", -80.0f, -10.0f, -70.0f, "dB"),
  PV("comp", "Compressor", 0.0f, 1.0f, 0.3f, ""),
  /* D - TONE */
  PV("eq_bass", "Bass", -18.0f, 18.0f, 1.5f, "dB"),
  PV("eq_mid", "Middle", -18.0f, 18.0f, -2.5f, "dB"),
  PV("eq_treble", "Treble", -18.0f, 18.0f, 3.0f, "dB"),
  /* E - SPACE */
  { "mod_type", "Mod Type", 0.0f, 3.0f, 0.0f, "", 4, MOD_KINDS },
  PV("mod_rate", "Mod Rate", 0.0f, 1.0f, 0.35f, ""),
  PV("mod_depth", "Mod Depth", 0.0f, 1.0f, 0.6f, ""),
  PV("mod_mix", "Mod Mix", 0.0f, 1.0f, 0.25f, ""),
  PV("delay_mix", "Delay Mix", 0.0f, 1.0f, 0.0f, ""),
  PV("delay_ms", "Delay Time", 0.0f, 1400.0f, 375.0f, "ms"),
  PV("delay_fb", "Delay Feedback", 0.0f, 0.9f, 0.35f, ""),
  PV("reverb_mix", "Reverb Mix", 0.0f, 1.0f, 0.08f, ""),
  PV("reverb_room", "Reverb Room", 0.0f, 1.0f, 0.45f, ""),
  /* F - SYSTEM */
  PV("master", "Master", 0.0f, 1.0f, 0.8f, ""),
  { "demo_riff", "Demo Riff", 0.0f, 1.0f, 0.0f, "", 2, ON_OFF },
  PV("demo_bpm", "Demo Tempo", 60.0f, 180.0f, 104.0f, "bpm"),
  /* the new wah controls live on the same tab as the original nine */
  { "wah_voicing", "Wah Voicing", 0.0f, 2.0f, 0.0f, "", 3, WAH_VOICINGS },
  { "wah_shape", "Sweep Shape", 0.0f, 3.0f, 0.0f, "", 4, WAH_SHAPES },
  PV("wah_sens", "Wah Sensitivity", 0.0f, 1.0f, 0.55f, ""),
  PV("wah_hold", "Wah Hold", 0.0f, 500.0f, 120.0f, "ms"),
  PV("wah_drive", "Wah Drive", 0.0f, 1.0f, 0.2f, "")
};
#undef PV

static const char *const GROUPS[P_COUNT] = {
  "A · WAH", "A · WAH", "A · WAH", "A · WAH", "A · WAH", "A · WAH",
  "A · WAH", "A · WAH", "A · WAH",
  "B · LESLIE", "B · LESLIE", "B · LESLIE", "B · LESLIE", "B · LESLIE", "B · LESLIE",
  "C · DRIVE", "C · DRIVE", "C · DRIVE",
  "D · TONE", "D · TONE", "D · TONE",
  "E · SPACE", "E · SPACE", "E · SPACE", "E · SPACE", "E · SPACE", "E · SPACE",
  "E · SPACE", "E · SPACE", "E · SPACE",
  "F · SYSTEM", "F · SYSTEM", "F · SYSTEM",
  "A · WAH", "A · WAH", "A · WAH", "A · WAH", "A · WAH"
};

static const vibe_card CARDS[] = {
  { "WAH", 11, "Sweep, envelope, touch, talk or a real pedal" },
  { "LESLIE", 6, "Dual-rotor stereo Leslie with independent motors" },
  { "DRIVE", 4, "Gate, compressor and grit" },
  { "TONE", 4, "Three-band EQ" },
  { "SPACE", 4, "Modulation, delay and reverb" },
  { "SYSTEM", 12, "Level and the built-in demo" }
};

typedef struct {
  const char *name;
  int n;
  const int id[16];
  const float val[16];
} vf_preset;

static const vf_preset PRESETS[] = {
  { "FUNK WAH", 15,
    { P_WAH_MODE, P_WAH_SPEED, P_WAH_DEPTH, P_WAH_RES, P_WAH_LOW, P_WAH_HIGH,
      P_WAH_VOICING, P_WAH_SHAPE, P_WAH_SENS, P_WAH_HOLD, P_WAH_DRIVE,
      P_LESLIE_ON, P_DRIVE, P_EQ_MID, P_REVERB_MIX },
    { 1.0f, 2.4f, 0.9f, 5.5f, 330.0f, 3400.0f,
      0.0f, 0.0f, 0.55f, 120.0f, 0.20f, 0.0f, 0.22f, -2.5f, 0.08f } },
  { "ROTARY", 9,
    { P_WAH_MODE, P_LESLIE_ON, P_LESLIE_SPEED, P_LESLIE_BALANCE, P_LESLIE_DEPTH,
      P_LESLIE_DRIVE, P_DRIVE, P_EQ_MID, P_REVERB_MIX },
    { 0.0f, 1.0f, 1.0f, 0.5f, 0.85f, 0.3f, 0.12f, -1.0f, 0.15f } },
  { "WAH + ROTARY", 13,
    { P_WAH_MODE, P_WAH_SPEED, P_WAH_DEPTH, P_WAH_VOICING, P_WAH_SENS,
      P_LESLIE_ON, P_LESLIE_SPEED, P_LESLIE_BALANCE, P_LESLIE_DEPTH,
      P_DRIVE, P_WAH_DRIVE, P_DELAY_MIX, P_REVERB_MIX },
    { 1.0f, 1.8f, 0.75f, 1.0f, 0.45f, 1.0f, 1.0f, 0.55f, 0.65f,
      0.20f, 0.12f, 0.12f, 0.12f } },
  { "FUNK DUB", 15,
    { P_WAH_MODE, P_WAH_SPEED, P_WAH_DEPTH, P_WAH_SHAPE, P_WAH_RES,
      P_WAH_VOICING, P_WAH_SENS, P_LESLIE_ON, P_MOD_TYPE, P_MOD_RATE, P_MOD_MIX,
      P_DELAY_MIX, P_DELAY_MS, P_DELAY_FB, P_REVERB_MIX },
    { 1.0f, 1.2f, 0.6f, 2.0f, 4.2f, 0.0f, 0.5f, 0.0f, 2.0f, 0.35f, 0.35f,
      0.35f, 380.0f, 0.48f, 0.22f } },
  { "CLEAN", 9,
    { P_WAH_MODE, P_LESLIE_ON, P_DRIVE, P_COMP, P_EQ_BASS, P_EQ_MID, P_EQ_TREBLE,
      P_MOD_TYPE, P_REVERB_MIX },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f } },
  { "TALK BOX", 12,
    { P_WAH_MODE, P_WAH_SPEED, P_WAH_DEPTH, P_WAH_RES, P_WAH_LOW, P_WAH_HIGH,
      P_WAH_VOICING, P_WAH_SHAPE, P_WAH_DRIVE, P_LESLIE_ON, P_DELAY_MIX, P_REVERB_MIX },
    { 4.0f, 2.2f, 0.85f, 7.0f, 450.0f, 2600.0f,
      1.0f, 2.0f, 0.25f, 0.0f, 0.12f, 0.12f } },
  { "BASS WAH", 12,
    { P_WAH_MODE, P_WAH_DEPTH, P_WAH_RES, P_WAH_LOW, P_WAH_HIGH,
      P_WAH_VOICING, P_WAH_SENS, P_WAH_HOLD, P_WAH_DRIVE, P_LESLIE_ON,
      P_EQ_BASS, P_DRIVE },
    { 2.0f, 0.9f, 6.0f, 130.0f, 1400.0f,
      2.0f, 0.6f, 180.0f, 0.15f, 0.0f, 2.0f, 0.20f } }
};
#define NPRESET ((int)(sizeof PRESETS / sizeof PRESETS[0]))

static const int ENC_PARAM[UI_N][4] = {
  { P_WAH_SPEED, P_WAH_DEPTH, P_WAH_RES, P_WAH_SENS },
  { P_DRIVE, P_MOD_MIX, P_DELAY_MIX, P_REVERB_MIX },
  { P_EQ_BASS, P_EQ_MID, P_EQ_TREBLE, P_MASTER },
  { P_WAH_SPEED, P_WAH_DEPTH, P_LESLIE_SPEED, P_DEMO_BPM }
};

static const char *const HELP[] = {
  "KEYS", "pluck the demo guitar (showcase only)",
  "DEMO RIFF", "hands-free funk riff: press F6 / K6 / PLAY to start or stop",
  "IN 1", "your real guitar runs through the same FX",
  "F1 / K5", "next factory preset",
  "F2", "wah mode: off / sweep / env / touch / talk / pedal",
  "K2", "wah on / off (remembers the last mode)",
  "PEDAL / CC4", "expression pedal drives the wah in PEDAL mode (CC11 / CC1 too)",
  "F3", "rotor: stop / slow / fast",
  "F4 / F5", "FX page / TONE page",
  "F6 / K6", "demo riff on / off (off at startup)",
  "K1", "reset all controls",
  "K2 / K3", "wah on / off, Leslie on / off",
  "K4", "Leslie slow / fast",
  "KNOBS 1-4", "the four controls of the open page",
  "DATA / ENTER", "FOCUS: walk a control, press to edit",
  "RETURN / HOME", "close help, back to main page",
  "REC / STOP", "panic / demo off", "PLAY", "demo on"
};

static int aget(int *p) { return __atomic_load_n(p, __ATOMIC_ACQUIRE); }
static void aset(int *p, int v) { __atomic_store_n(p, v, __ATOMIC_RELEASE); }
static int scaled(float v) { return (int)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f)); }
static float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

static void fx_sync(void);
static void set_param(int id, float v);
static void nudge(int id, int d);
static void apply_preset(int p);
static void store_state(void);
static void panic(void);

/* ---- state publication ------------------------------------------------------ */

static void fx_sync(void)
{
  vf_fx_params p;
  p.wah_mode       = aget(&g_params[P_WAH_MODE]);
  p.wah_speed      = aget(&g_params[P_WAH_SPEED]);
  p.wah_depth      = aget(&g_params[P_WAH_DEPTH]);
  p.wah_res        = aget(&g_params[P_WAH_RES]);
  p.wah_low        = aget(&g_params[P_WAH_LOW]);
  p.wah_high       = aget(&g_params[P_WAH_HIGH]);
  p.wah_attack     = aget(&g_params[P_WAH_ATTACK]);
  p.wah_release    = aget(&g_params[P_WAH_RELEASE]);
  p.wah_mix        = aget(&g_params[P_WAH_MIX]);
  p.wah_voicing    = aget(&g_params[P_WAH_VOICING]);
  p.wah_shape      = aget(&g_params[P_WAH_SHAPE]);
  p.wah_sens       = aget(&g_params[P_WAH_SENS]);
  p.wah_hold       = aget(&g_params[P_WAH_HOLD]);
  p.wah_drive      = aget(&g_params[P_WAH_DRIVE]);
  p.les_on         = aget(&g_params[P_LESLIE_ON]);
  p.les_speed      = aget(&g_params[P_LESLIE_SPEED]);
  p.les_ramp       = aget(&g_params[P_LESLIE_RAMP]);
  p.les_balance    = aget(&g_params[P_LESLIE_BALANCE]);
  p.les_depth      = aget(&g_params[P_LESLIE_DEPTH]);
  p.les_drive      = aget(&g_params[P_LESLIE_DRIVE]);
  p.drive          = aget(&g_params[P_DRIVE]);
  p.gate           = aget(&g_params[P_GATE]);
  p.comp           = aget(&g_params[P_COMP]);
  p.eq_bass        = aget(&g_params[P_EQ_BASS]);
  p.eq_mid         = aget(&g_params[P_EQ_MID]);
  p.eq_treble      = aget(&g_params[P_EQ_TREBLE]);
  p.mod_type       = aget(&g_params[P_MOD_TYPE]);
  p.mod_rate       = aget(&g_params[P_MOD_RATE]);
  p.mod_depth      = aget(&g_params[P_MOD_DEPTH]);
  p.mod_mix        = aget(&g_params[P_MOD_MIX]);
  p.delay_mix      = aget(&g_params[P_DELAY_MIX]);
  p.delay_ms       = aget(&g_params[P_DELAY_MS]);
  p.delay_fb       = aget(&g_params[P_DELAY_FB]);
  p.reverb_mix     = aget(&g_params[P_REVERB_MIX]);
  p.reverb_room    = aget(&g_params[P_REVERB_ROOM]);
  p.master         = aget(&g_params[P_MASTER]);
  vf_fx_prepare(&p);
}

static void set_param(int id, float v)
{
  if (id < 0 || id >= P_COUNT) return;
  v = clampf(v, PARAMS[id].min, PARAMS[id].max);
  if (PARAMS[id].steps > 1) {
    float st = (PARAMS[id].max - PARAMS[id].min) / (float)(PARAMS[id].steps - 1);
    v = PARAMS[id].min + st * (float)(int)( (v - PARAMS[id].min) / st + 0.5f );
  }
  ui_vals[id] = v;
  aset(&g_params[id], scaled(v));
  if (id == P_WAH_MODE && v > 0.5f) wah_last_mode = (int)(v + 0.5f);
  vibe_param_set(id, v);
  fx_sync();
}

static void nudge(int id, int d)
{
  float v;
  if (id < 0 || id >= P_COUNT) return;
  v = ui_vals[id];
  if (PARAMS[id].steps > 1) v += (float)d;
  else {
    float span = PARAMS[id].max - PARAMS[id].min;
    v += (float)d * (span > 0.0f ? span / 100.0f : 0.01f);
  }
  set_param(id, v);
}

static void apply_preset(int p)
{
  int i;
  if (p < 0) p = 0;
  if (p >= NPRESET) p = NPRESET - 1;
  preset = p;
  for (i = 0; i < PRESETS[p].n; i++) set_param(PRESETS[p].id[i], PRESETS[p].val[i]);
  status[0] = 0;
  {
    char t[32];
    int n = 0;
    const char *s = PRESETS[p].name;
    while (s[n] && n < (int)sizeof t - 1) { t[n] = s[n]; n++; }
    t[n] = 0;
    n = 0;
    while (t[n] && n < 60) { status[n] = t[n]; n++; }
    status[n] = 0;
  }
}

static void store_state(void)
{
  int32_t buf[P_COUNT], i;
  for (i = 0; i < P_COUNT; i++) buf[i] = (int32_t)aget(&g_params[i]);
  vibe_store_write("state", buf, (int)sizeof buf);
}

static void load_state(void)
{
  int32_t buf[P_COUNT];
  int i, count, n = vibe_store_read("state", buf, (int)sizeof buf);
  /* rigs saved before the extra wah controls still load; new ones keep defaults */
  if (n < 34 * (int)sizeof(int32_t)) return;
  count = n / (int)sizeof(int32_t);
  if (count > P_COUNT) count = P_COUNT;
  for (i = 0; i < count; i++) {
    float v = (float)buf[i] * 0.001f;
    if (v < PARAMS[i].min || v > PARAMS[i].max) return;
  }
  for (i = 0; i < count; i++) {
    ui_vals[i] = (float)buf[i] * 0.001f;
    aset(&g_params[i], (int)buf[i]);
  }
}

/* ---- demo guitar (audio flow only) ----------------------------------------- */

static void ks_trigger(int note, float vel)
{
  float f, per;
  int v = 0, oldest = 0x7fffffff, i;
  for (i = 0; i < KS_VOICES; i++) {
    if (ks_amp[i] <= 0.0001f) { v = i; break; }
    if (ks_age[i] < oldest) { oldest = ks_age[i]; v = i; }
  }
  f = vd_mtof((float)note);
  per = sr / (f > 20.0f ? f : 20.0f);
  ks_len[v] = (int)(per + 0.5f);
  if (ks_len[v] < 8) ks_len[v] = 8;
  if (ks_len[v] > KS_BUFSZ - 2) ks_len[v] = KS_BUFSZ - 2;
  for (i = 0; i < ks_len[v]; i++)
    ks_buf[v][i] = (float)(int32_t)vd_rand() * (1.0f / 2147483648.0f);
  /* a soft pick: smooth the first few samples */
  for (i = 0; i < 4 && i + 1 < ks_len[v]; i++)
    ks_buf[v][i] = 0.5f * (ks_buf[v][i] + ks_buf[v][i + 1]);
  ks_idx[v] = 0;
  ks_age[v] = 0;
  ks_amp[v] = clampf(vel, 0.0f, 1.0f) * 0.6f;
  ks_damp[v] = 0.994f + 0.004f * clampf((127.0f - (float)note) / 127.0f, 0.0f, 1.0f);
}

static float ks_render(void)
{
  int v;
  float sum = 0.0f;
  for (v = 0; v < KS_VOICES; v++) {
    float y, nx;
    int next;
    if (ks_amp[v] <= 0.0001f) continue;
    y = ks_buf[v][ks_idx[v]];
    next = ks_idx[v] + 1;
    if (next >= ks_len[v]) next = 0;
    nx = ks_buf[v][next];
    ks_buf[v][ks_idx[v]] = 0.5f * (y + nx) * ks_damp[v];
    ks_idx[v] = next;
    sum += y * ks_amp[v];
    ks_age[v]++;
    if (ks_age[v] > (int)(sr * 4.0f)) ks_amp[v] = 0.0f;
  }
  return vd_tanh(sum);
}

static void panic(void)
{
  int i;
  for (i = 0; i < KS_VOICES; i++) ks_amp[i] = 0.0f;
  riff_clock = riff_step = 0;
  q_head = q_tail = 0;
  vf_fx_reset();
}

/* ---- events ---------------------------------------------------------------- */

static void note_on(int note, float vel)
{
  int h, nh;
  if (vel <= 0.0f) return;                 /* the demo strings ring out */
  h = q_head;
  nh = (h + 1) % QN;
  if (nh == q_tail) return;
  q_note[h] = note;
  q_vel[h] = scaled(vel);
  q_on[h] = 1;
  VIBE_RELEASE();
  q_head = nh;
}

static void cycle(int id)
{
  float v = ui_vals[id] + 1.0f;
  float max = PARAMS[id].max;
  if (v > max) v = PARAMS[id].min;
  set_param(id, v);
}

static void next_preset(int d)
{
  int p = preset + d;
  while (p < 0) p += NPRESET;
  while (p >= NPRESET) p -= NPRESET;
  apply_preset(p);
}

static void handle_button(int id)
{
  switch (id) {
    case VB_F1: next_preset(1); break;
    case VB_F2: cycle(P_WAH_MODE); break;
    case VB_F3: cycle(P_LESLIE_SPEED); break;
    case VB_F4: ui_mode = UI_FX; break;
    case VB_F5: ui_mode = UI_TONE; break;
    case VB_F6: set_param(P_DEMO_RIFF, ui_vals[P_DEMO_RIFF] > 0.5f ? 0.0f : 1.0f); break;
    case VB_K1: { int i; for (i = 0; i < P_COUNT; i++) set_param(i, PARAMS[i].def); } break;
    case VB_K2: set_param(P_WAH_MODE, ui_vals[P_WAH_MODE] > 0.5f ? 0.0f : (float)wah_last_mode); break;
    case VB_K3: set_param(P_LESLIE_ON, ui_vals[P_LESLIE_ON] > 0.5f ? 0.0f : 1.0f); break;
    case VB_K4: set_param(P_LESLIE_SPEED, ui_vals[P_LESLIE_SPEED] > 1.5f ? 1.0f : 2.0f); break;
    case VB_K5: next_preset(1); break;
    case VB_K6: set_param(P_DEMO_RIFF, ui_vals[P_DEMO_RIFF] > 0.5f ? 0.0f : 1.0f); break;
    case VB_REC: panic(); break;
    case VB_STOP: set_param(P_DEMO_RIFF, 0.0f); break;
    case VB_PLAY: set_param(P_DEMO_RIFF, 1.0f); break;
    case VB_HOME: help = !help; break;
    case VB_RETURN: if (help) help = 0; else ui_mode = UI_WAH; break;
    default: break;
  }
}

static void handle_editor(const uint8_t *b, int n)
{
  if (!b || n < 1) return;
  if (b[0] == 0x20) { ed_pending = 1; }
  else if (b[0] == 0x21) store_state();
  else if (b[0] == 0x22) apply_preset(0);
  else if (b[0] == 0x23) panic();
}

static void append_txt(char *d, int *n, const char *s, int max)
{
  while (*s && *n < max - 1) { d[*n] = *s; (*n)++; s++; }
  d[*n] = 0;
}

static void build_status(void)
{
  int n = 0;
  char t[24];
  ed_status[0] = 0;
  append_txt(ed_status, &n, PRESETS[preset].name, (int)sizeof ed_status);
  append_txt(ed_status, &n, " | wah ", (int)sizeof ed_status);
  append_txt(ed_status, &n, vg_itoa(vf_fx_wah_hz100() / 100, t), (int)sizeof ed_status);
  append_txt(ed_status, &n, " Hz | rotor ", (int)sizeof ed_status);
  append_txt(ed_status, &n, vg_ftoa((float)vf_fx_rotor_hz100() * 0.01f, 1, t), (int)sizeof ed_status);
  append_txt(ed_status, &n, " Hz | ", (int)sizeof ed_status);
  append_txt(ed_status, &n, aget(&g_params[P_DEMO_RIFF]) ? "riff on" : "riff off",
             (int)sizeof ed_status);
  ed_len = n;
  ed_pending = 1;
}

void vibe_on_event(const vibe_event *e)
{
  int id = -1, r;
  if (!e) return;
  r = vf_event(e, &id);
  if (r == VF_PRESS) {
    if (id >= 100 && id < 100 + NPRESET) apply_preset(id - 100);
    else handle_button(id);
    return;
  }
  if (r == VF_CHANGED) {
    if (id >= 0 && id < P_COUNT) set_param(id, ui_vals[id]);
    return;
  }
  if (r == VF_MOVED) return;

  if (e->type == VIBE_EV_NOTE) {
    if (e->down) note_on((int)e->id, e->value);
    return;
  }
  if (e->type == VIBE_EV_PEDAL) {
    if (e->id == VIBE_PEDAL_EXPR)
      vf_fx_set_pedal((int)(clampf(e->value, 0.0f, 1.0f) * 1000.0f + 0.5f));
    else if (e->id == VIBE_PEDAL_SWITCH && e->down)
      set_param(P_WAH_MODE, ui_vals[P_WAH_MODE] > 0.5f ? 0.0f : 5.0f);
    return;
  }
  if (e->type == VIBE_EV_MIDI) {
    /* CC4 foot controller, CC11 expression, CC1 mod wheel -> wah pedal. */
    if ((e->midi[0] & 0xF0) == 0xB0) {
      int cc = e->midi[1];
      if (cc == 4 || cc == 11 || cc == 1)
        vf_fx_set_pedal((int)(clampf((float)e->midi[2] / 127.0f, 0.0f, 1.0f) * 1000.0f + 0.5f));
    }
    return;
  }
  if (e->type == VIBE_EV_KEY) {
    if (e->down) note_on(48 + (int)e->id, e->value > 0.0f ? e->value : 0.8f);
    return;
  }
  if (e->type == VIBE_EV_PARAM) {
    int i = (int)e->id;
    if (i >= 0 && i < P_COUNT) {
      ui_vals[i] = e->value;
      aset(&g_params[i], scaled(e->value));
      fx_sync();
    }
    return;
  }
  if (e->type == VIBE_EV_EDITOR_DATA) {
    handle_editor(vibe_editor_data(), (int)e->value);
    return;
  }
  if (e->type == VIBE_EV_SAVE) { store_state(); return; }
  if (e->type == VIBE_EV_BUTTON) {
    if (e->down) handle_button((int)e->id);
    return;
  }
  if (e->type == VIBE_EV_ENCODER) {
    int d = e->value > 0.0f ? 1 : (e->value < 0.0f ? -1 : 0);
    if (!d) return;
    if (e->id == VE_A) { ui_mode = (ui_mode + 1) % UI_N; return; }
    if (e->id == VE_B) { next_preset(d); return; }
    if (e->id == VE_DATA) { nudge(ENC_PARAM[ui_mode][0], d); return; }
    if (e->id >= VE_1 && e->id <= VE_4) nudge(ENC_PARAM[ui_mode][e->id - VE_1], d);
  }
}

/* ---- audio ----------------------------------------------------------------- */

void vibe_on_audio(const float *in, float *out_l, float *out_r, int frames)
{
  int n = frames > VIBE_MAX_BLOCK ? VIBE_MAX_BLOCK : frames;
  int i, t, riff_on, riff_period;
  vf_coeff fc;
  if (n < 0) n = 0;

  /* drain the key/editor note queue */
  VIBE_ACQUIRE();
  t = q_tail;
  while (t != q_head) {
    if (q_on[t] && q_vel[t] > 0) ks_trigger(q_note[t], (float)q_vel[t] * 0.001f);
    t = (t + 1) % QN;
  }
  q_tail = t;

  /* hands-free demo riff: clock it per SAMPLE, not per audio block */
  {
    int bpm = aget(&g_params[P_DEMO_BPM]) / 1000;
    if (bpm < 60) bpm = 60;
    if (bpm > 180) bpm = 180;
    riff_period = (int)(sr * 15.0f / (float)bpm);   /* a sixteenth note */
    if (riff_period < 1) riff_period = 1;
    riff_on = aget(&g_params[P_DEMO_RIFF]);
  }

  for (i = 0; i < n; i++) {
    if (riff_on) {
      riff_clock++;
      if (riff_clock >= riff_period) {
        riff_clock -= riff_period;
        ks_trigger(40 + (int)RIFF[riff_step], (riff_step % 4 == 0) ? 0.95f : 0.62f);
        riff_step = (riff_step + 1) & 15;
      }
    }
    mono[i] = (in ? in[i] : 0.0f) + ks_render();
  }

  vf_fx_copy(&fc);
  vf_fx_process(mono, out_l, out_r, n, &fc);
  for (i = n; i < frames; i++) { out_l[i] = 0.0f; out_r[i] = 0.0f; }
}

/* ---- drawing --------------------------------------------------------------- */

static void draw_afro(float cx, float cy, float r)
{
  int i;
  for (i = 0; i < 18; i++) {
    float a = (float)i / 18.0f;
    float x = vd_sin(a), y = vd_sin(a + 0.25f);
    vg_circle(cx + x * r * 0.72f, cy - y * r * 0.66f - r * 0.12f,
              r * 0.44f, i & 1 ? C_ACC : C_GOLD);
  }
  vg_circle(cx, cy + r * 0.22f, r * 0.46f, C_CREAM);
  vg_circle(cx - r * 0.16f, cy + r * 0.20f, r * 0.045f, C_BG);
  vg_circle(cx + r * 0.16f, cy + r * 0.20f, r * 0.045f, C_BG);
  vg_line(cx - r * 0.14f, cy + r * 0.40f, cx + r * 0.14f, cy + r * 0.40f, 1.5f, C_BG);
}

static void draw_bar(int x, int y, int w, float v, uint32_t col)
{
  vg_round(x, y, w, 8, 4, C_PANEL2);
  if (v > 0.0f) vg_round(x, y, (int)(w * clampf(v, 0.0f, 1.0f)), 8, 4, col);
}

static void draw_bipolar(int x, int y, int w, float v, uint32_t col)
{
  int cx = x + w / 2, ex = x + (int)(w * clampf((v + 1.0f) * 0.5f, 0.0f, 1.0f));
  vg_round(x, y, w, 8, 4, C_PANEL2);
  vg_line((float)cx, (float)y + 1.0f, (float)cx, (float)y + 7.0f, 1.0f, C_DIM);
  if (ex > cx) vg_round(cx, y, ex - cx, 8, 4, col);
  else if (ex < cx) vg_round(ex, y, cx - ex, 8, 4, col);
}

static void fmt_hz(char *d, int n, int hz100)
{
  int hz = hz100 / 100;
  int k = 0;
  d[k++] = (char)('0' + (hz / 1000) % 10);
  d[k++] = (char)('0' + (hz / 100) % 10);
  d[k++] = (char)('0' + (hz / 10) % 10);
  d[k++] = (char)('0' + hz % 10);
  d[k] = 0;
  (void)n;
}

static void draw_wah(void)
{
  char t[24];
  int mode = (int)ui_vals[P_WAH_MODE];
  int voice = (int)ui_vals[P_WAH_VOICING];
  int shape = (int)ui_vals[P_WAH_SHAPE];
  int wah = vf_fx_wah_hz100();
  float norm;
  int lo = (int)ui_vals[P_WAH_LOW], hi = (int)ui_vals[P_WAH_HIGH];

  vg_round(8, 54, 224, 190, 8, C_PANEL);
  vg_round(248, 54, 224, 190, 8, C_PANEL);
  vg_text("AUTO WAH", 18, 62, VG_BOLD, C_ACC, VG_LEFT);

  vg_text("MODE", 18, 82, VG_SMALL, C_MUTED, VG_LEFT);
  vg_text(WAH_MODES[mode < 0 || mode > 5 ? 0 : mode], 66, 81, VG_BOLD, C_FG, VG_LEFT);
  vg_text("VOICE", 18, 98, VG_SMALL, C_MUTED, VG_LEFT);
  vg_text(WAH_VOICINGS[voice < 0 || voice > 2 ? 0 : voice], 62, 97, VG_BOLD, C_GOLD, VG_LEFT);

  vg_text("SPEED", 18, 114, VG_SMALL, C_MUTED, VG_LEFT);
  vg_ftoa(ui_vals[P_WAH_SPEED], 1, t);
  vg_text(t, 210, 113, VG_BOLD, C_GOLD, VG_RIGHT);
  draw_bar(18, 130, 204, (ui_vals[P_WAH_SPEED] - PARAMS[P_WAH_SPEED].min) /
           (PARAMS[P_WAH_SPEED].max - PARAMS[P_WAH_SPEED].min), C_ACC);
  vf_value(P_WAH_SPEED, 18, 116, 204, 24, &ui_vals[P_WAH_SPEED],
           PARAMS[P_WAH_SPEED].min, PARAMS[P_WAH_SPEED].max, 0.1f);

  vg_text("SHAPE", 18, 144, VG_SMALL, C_MUTED, VG_LEFT);
  vg_text(WAH_SHAPES[shape < 0 || shape > 3 ? 0 : shape], 62, 143, VG_BOLD, C_FG, VG_LEFT);

  vg_text("DEPTH", 18, 160, VG_SMALL, C_MUTED, VG_LEFT);
  vg_ftoa(ui_vals[P_WAH_DEPTH], 2, t);
  vg_text(t, 210, 159, VG_BOLD, C_FG, VG_RIGHT);
  draw_bar(18, 176, 204, ui_vals[P_WAH_DEPTH], C_AVO);
  vf_value(P_WAH_DEPTH, 18, 162, 204, 24, &ui_vals[P_WAH_DEPTH], 0.0f, 1.0f, 0.01f);

  vg_text("RES", 18, 190, VG_SMALL, C_MUTED, VG_LEFT);
  vg_ftoa(ui_vals[P_WAH_RES], 1, t);
  vg_text(t, 60, 189, VG_BOLD, C_GOLD, VG_RIGHT);
  vg_text("SENS", 104, 190, VG_SMALL, C_MUTED, VG_LEFT);
  vg_ftoa(ui_vals[P_WAH_SENS], 2, t);
  vg_text(t, 210, 189, VG_BOLD, C_GOLD, VG_RIGHT);
  vf_value(P_WAH_RES, 18, 186, 72, 22, &ui_vals[P_WAH_RES], 0.7f, 12.0f, 0.1f);
  vf_value(P_WAH_SENS, 104, 186, 112, 22, &ui_vals[P_WAH_SENS], 0.0f, 1.0f, 0.01f);

  vg_text("SWEEP", 18, 205, VG_SMALL, C_MUTED, VG_LEFT);
  draw_bar(18, 220, 204, 1.0f, C_PANEL2);
  norm = hi > lo ? (float)(wah - lo * 100) / (float)((hi - lo) * 100) : 0.0f;
  norm = clampf(norm, 0.0f, 1.0f);
  vg_circle(18.0f + 204.0f * norm, 224.0f, 5.0f, C_GOLD);
  fmt_hz(t, (int)sizeof t, wah);
  vg_text(t, 18, 226, VG_BIG, C_GOLD, VG_LEFT);
  vg_text("Hz", 82, 234, VG_SMALL, C_MUTED, VG_LEFT);
  vg_ftoa(ui_vals[P_WAH_HOLD], 0, t);
  vg_text(t, 210, 226, VG_BOLD, C_FG, VG_RIGHT);
  vg_text("HOLD ms", 160, 235, VG_SMALL, C_DIM, VG_LEFT);
  vf_value(P_WAH_HOLD, 152, 222, 70, 20, &ui_vals[P_WAH_HOLD], 0.0f, 500.0f, 10.0f);

  /* Leslie */
  vg_text("LESLIE", 258, 62, VG_BOLD, C_ACC, VG_LEFT);
  {
    int ls = (int)ui_vals[P_LESLIE_SPEED];
    int rot = vf_fx_rotor_hz100();
    vg_text(ui_vals[P_LESLIE_ON] > 0.5f ? "ON" : "OFF", 440, 62, VG_BOLD,
            ui_vals[P_LESLIE_ON] > 0.5f ? C_GREEN : C_RED, VG_RIGHT);
    vg_text("ROTOR", 258, 84, VG_SMALL, C_MUTED, VG_LEFT);
    vg_text(LES_SPEEDS[ls < 0 || ls > 2 ? 0 : ls], 330, 83, VG_BOLD, C_FG, VG_LEFT);
    vg_ftoa((float)rot * 0.01f, 1, t);
    vg_text(t, 458, 83, VG_BOLD, C_GOLD, VG_RIGHT);
    vg_text("Hz", 462, 84, VG_SMALL, C_MUTED, VG_LEFT);

    vg_circle(282.0f, 120.0f, 12.0f, C_PANEL2);
    vg_circle(282.0f, 120.0f, 5.0f, C_GOLD);
    vg_circle(318.0f, 120.0f, 12.0f, C_PANEL2);
    vg_circle(318.0f, 120.0f, 5.0f, C_AVO);
    vg_text("HORN", 262, 138, VG_SMALL, C_MUTED, VG_LEFT);
    vg_text("BASS", 298, 138, VG_SMALL, C_MUTED, VG_LEFT);

    vg_text("BALANCE", 342, 108, VG_SMALL, C_MUTED, VG_LEFT);
    vg_ftoa(ui_vals[P_LESLIE_BALANCE], 2, t);
    vg_text(t, 458, 107, VG_BOLD, C_FG, VG_RIGHT);
    draw_bar(342, 124, 110, ui_vals[P_LESLIE_BALANCE], C_GOLD);
    vf_value(P_LESLIE_BALANCE, 342, 116, 110, 26, &ui_vals[P_LESLIE_BALANCE], 0.0f, 1.0f, 0.01f);

    vg_text("DEPTH", 258, 154, VG_SMALL, C_MUTED, VG_LEFT);
    draw_bar(258, 170, 194, ui_vals[P_LESLIE_DEPTH], C_AVO);
    vf_value(P_LESLIE_DEPTH, 258, 162, 194, 26, &ui_vals[P_LESLIE_DEPTH], 0.0f, 1.0f, 0.01f);

    vg_text("RAMP", 258, 190, VG_SMALL, C_MUTED, VG_LEFT);
    vg_ftoa(ui_vals[P_LESLIE_RAMP], 1, t);
    vg_text(t, 452, 189, VG_BOLD, C_FG, VG_RIGHT);
    vg_text("F3 slow / fast", 258, 216, VG_SMALL, C_DIM, VG_LEFT);
    vg_text("PEDAL: CC4 / CC11 / EXPR", 258, 230, VG_SMALL, C_DIM, VG_LEFT);
  }
}

static void draw_fx(void)
{
  static const int ids[6] = { P_DRIVE, P_MOD_MIX, P_DELAY_MIX, P_REVERB_MIX, P_COMP, P_GATE };
  static const char *const names[6] = { "DRIVE", "MOD MIX", "DELAY MIX", "REVERB", "COMP", "GATE" };
  int i;
  vg_text("EFFECT RACK", 16, 58, VG_BIG, C_ACC, VG_LEFT);
  for (i = 0; i < 6; i++) {
    int y = 84 + i * 30;
    float v = ui_vals[ids[i]];
    float norm = (v - PARAMS[ids[i]].min) / (PARAMS[ids[i]].max - PARAMS[ids[i]].min);
    char t[24];
    vg_text(names[i], 16, y, VG_SMALL, C_MUTED, VG_LEFT);
    draw_bar(116, y + 3, 260, norm, i == 4 ? C_AVO : C_ACC);
    vg_ftoa(v, 2, t);
    vg_text(t, 462, y, VG_BOLD, C_GOLD, VG_RIGHT);
    vf_value(ids[i], 116, y - 2, 260, 22, &ui_vals[ids[i]],
             PARAMS[ids[i]].min, PARAMS[ids[i]].max, 0.01f);
  }
  vg_text(MOD_KINDS[(int)ui_vals[P_MOD_TYPE]], 16, 246, VG_SMALL, C_DIM, VG_LEFT);
}

static void draw_tone(void)
{
  static const int ids[3] = { P_EQ_BASS, P_EQ_MID, P_EQ_TREBLE };
  static const char *const names[3] = { "BASS", "MIDDLE", "TREBLE" };
  int i;
  vg_text("TONE", 16, 58, VG_BIG, C_ACC, VG_LEFT);
  for (i = 0; i < 3; i++) {
    int x = 30 + i * 140;
    float v = ui_vals[ids[i]] / 18.0f;
    vg_round(x, 88, 56, 168, 10, C_PANEL);
    vg_rect(x + 26, 96, 4, 152, C_PANEL2);
    draw_bipolar(x + 12, 96, 32, v, i == 1 ? C_AVO : C_GOLD);
    vg_text(names[i], x + 28, 266, VG_SMALL, C_MUTED, VG_CENTER);
    {
      char t[24];
      vg_ftoa(ui_vals[ids[i]], 1, t);
      vg_text(t, x + 28, 96, VG_SMALL, C_FG, VG_CENTER);
    }
    vf_value(ids[i], x, 88, 56, 168, &ui_vals[ids[i]], -18.0f, 18.0f, 0.5f);
  }
  vg_text("MASTER", 460, 90, VG_SMALL, C_MUTED, VG_RIGHT);
  draw_bar(404, 106, 56, ui_vals[P_MASTER], C_ACC);
  vf_value(P_MASTER, 404, 100, 56, 22, &ui_vals[P_MASTER], 0.0f, 1.0f, 0.01f);
}

static void draw_presets(void)
{
  int i;
  vg_text("FACTORY PRESETS", 16, 58, VG_BIG, C_ACC, VG_LEFT);
  for (i = 0; i < NPRESET; i++) {
    int y = 88 + i * 28;
    vg_round(16, y, 448, 26, 6, i == preset ? C_PANEL2 : C_PANEL);
    vg_text(PRESETS[i].name, 30, y + 6, VG_MED, i == preset ? C_GOLD : C_FG, VG_LEFT);
    if (i == preset) vg_circle(448.0f, (float)y + 13.0f, 5.0f, C_ACC);
    vf_button(100 + i, 16, y, 448, 26);
  }
}

void vibe_on_draw(uint16_t *fb)
{
  static const char *const KEYS[6] = { "PRESET", "WAH", "LESLIE", "FX", "TONE", "DEMO" };
  int i;
  char t[24];
  vg_begin(fb);
  if (help) { vg_controls("VIBE FM - AUTOMATIC FX", HELP, (int)(sizeof HELP / sizeof HELP[0]) / 2); return; }

  vg_clear(C_BG);
  vg_rect(0, 0, 480, 48, C_PANEL);
  draw_afro(26.0f, 24.0f, 18.0f);
  vg_text("VIBE·FM", 56, 10, VG_BIG, C_ACC, VG_LEFT);
  vg_text("AUTOMATIC FUNK EFFECTS", 57, 34, VG_SMALL, C_MUTED, VG_LEFT);
  vg_text(PRESETS[preset].name, 464, 14, VG_BOLD, C_GOLD, VG_RIGHT);

  vf_begin();
  if (ui_mode == UI_WAH) draw_wah();
  else if (ui_mode == UI_FX) draw_fx();
  else if (ui_mode == UI_TONE) draw_tone();
  else draw_presets();

  vg_rect(0, 252, 480, 30, C_PANEL2);
  vg_ftoa(ui_vals[P_DEMO_BPM], 0, t);
  vg_text("RIFF", 12, 259, VG_SMALL, aget(&g_params[P_DEMO_RIFF]) ? C_GREEN : C_DIM, VG_LEFT);
  vg_text(t, 50, 259, VG_SMALL, C_FG, VG_LEFT);
  vg_text("BPM   KEYS = DEMO GUITAR   IN 1 = LIVE", 96, 259, VG_SMALL, C_MUTED, VG_LEFT);
  vg_text(status, 12, 274, VG_SMALL, C_GOLD, VG_LEFT);
  vf_end(C_ACC);

  vg_rect(0, 288, 480, 32, C_PANEL);
  for (i = 0; i < 6; i++) {
    int x = 8 + i * 78;
    vg_round(x, 292, 70, 24, 4, C_PANEL2);
    vg_text(KEYS[i], x + 35, 298, VG_BOLD, C_ACC, VG_CENTER);
  }
  vibe_label(VB_F1, KEYS[0]); vibe_label(VB_F2, KEYS[1]); vibe_label(VB_F3, KEYS[2]);
  vibe_label(VB_F4, KEYS[3]); vibe_label(VB_F5, KEYS[4]); vibe_label(VB_F6, KEYS[5]);

  {
    static const char *const PAGES[4] = { "AUTO WAH", "EFFECT RACK", "TONE", "PRESETS" };
    char v1[12], v2[12], v3[12], v4[12];
    const int *p1 = &ENC_PARAM[ui_mode][0];
    vibe_link_page(PAGES[ui_mode]);
    vg_ftoa(ui_vals[*p1], 2, v1);
    vg_ftoa(ui_vals[ENC_PARAM[ui_mode][1]], 2, v2);
    vg_ftoa(ui_vals[ENC_PARAM[ui_mode][2]], 2, v3);
    vg_ftoa(ui_vals[ENC_PARAM[ui_mode][3]], 2, v4);
    vibe_link_encoder(VE_1, PARAMS[*p1].name, v1);
    vibe_link_encoder(VE_2, PARAMS[ENC_PARAM[ui_mode][1]].name, v2);
    vibe_link_encoder(VE_3, PARAMS[ENC_PARAM[ui_mode][2]].name, v3);
    vibe_link_encoder(VE_4, PARAMS[ENC_PARAM[ui_mode][3]].name, v4);
  }

  draw_tick++;
  if ((draw_tick % 8) == 0) build_status();
  if (ed_pending) {
    if (vibe_editor_post((const uint8_t *)ed_status, ed_len) >= 0) ed_pending = 0;
  }
}

/* ---- init ------------------------------------------------------------------ */

void vibe_on_init(float sample_rate)
{
  int i;
  sr = sample_rate > 1000.0f ? sample_rate : 48000.0f;

  for (i = 0; i < P_COUNT; i++) {
    ui_vals[i] = PARAMS[i].def;
    aset(&g_params[i], scaled(PARAMS[i].def));
  }
  apply_preset(0);
  load_state();   /* a saved rig wins over the factory default */
  set_param(P_DEMO_RIFF, 0.0f);   /* never auto-start the riff, even from a saved rig */
  wah_last_mode = ui_vals[P_WAH_MODE] > 0.5f ? (int)(ui_vals[P_WAH_MODE] + 0.5f) : 1;

  vibe_params(PARAMS, P_COUNT);
  vibe_param_groups(GROUPS);
  vibe_editor_cards(CARDS, (int)(sizeof CARDS / sizeof CARDS[0]));
  vibe_editor_accent(0xFF8C1A);
  vibe_editor_tab_color("A", 0xFF8C1A);
  vibe_editor_tab_color("B", 0xF2C14E);
  vibe_editor_tab_color("C", 0xD95B3F);
  vibe_editor_tab_color("D", 0x8FA44B);
  vibe_editor_tab_color("E", 0x5AB6FF);
  vibe_param_help(P_WAH_MODE, "SWEEP: LFO. ENVELOPE: follows level. TOUCH: kick per note. TALK: two formants. PEDAL: CC4 / CC11 / expression jack.");
  vibe_param_help(P_WAH_VOICING, "CRYBABY is tight and nasal, VOX is broad and vocal, BASS sweeps low with a sharper resonant peak.");
  vibe_param_help(P_WAH_SENS, "Envelope / touch sensitivity: low = only strong picking opens the wah.");
  vibe_param_help(P_LESLIE_ON, "Dual-rotor Leslie: horn and drum have separate motors and a stereo mic pair.");
  vibe_param_help(P_LESLIE_SPEED, "STOP / SLOW / FAST. Horn ~0.9/7 Hz, drum ~0.8/6 Hz, with spin-up and brake.");
  vibe_param_help(P_LESLIE_RAMP, "Scales the motor spin-up and brake times. 1.5 s is the clean-room baseline.");
  vibe_param_help(P_DEMO_RIFF, "Off at startup. Press F6 / K6 / PLAY to start or stop the hands-free funk riff. Keys also pluck a demo guitar.");

  vibe_notes(VIBE_NOTES);
  vibe_request(VIBE_REQ_AUDIO_INPUT);
  vf_mode(VF_ON_HOLD);
  vf_clicks(1);
  vf_set_focus(P_WAH_SPEED);

  vf_fx_init(sr);
  fx_sync();
  for (i = 0; i < KS_VOICES; i++) { ks_amp[i] = 0.0f; ks_age[i] = 0; }
  q_head = q_tail = 0;
  riff_clock = riff_step = 0;
  ui_mode = UI_WAH;
  help = 0;
  status[0] = 0;
  ed_len = ed_pending = 0;
}
