/* SPDX-License-Identifier: GPL-3.0-or-later
 * Tests of the A380 flight-control laws and steering (a380x_fctl.h, docs/FLIGHT_CONTROLS.md).
 *
 * Unit checks (modes, AP bypass, direct law, steering schedule) and a closed-loop bench: a
 * hand-written heavy-jet airframe (point-mass longitudinal + short period, linear roll / Dutch roll,
 * a simple speed-holding thrust loop). It is NOT X-Plane's flight model; every closed-loop case is
 * re-flown with half and double control power, so the port gains are checked for robustness rather
 * than tuned to one model.
 */
#include "a380x_fctl.h"
#include <assert.h>
#include <stdio.h>

static int failures, checks;
#define CHECK(cond, ...) do { ++checks; if (!(cond)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

#define D2R 0.017453292519943295
#define R2D 57.29577951308232
#define G0 9.80665

/* ---------------------------------------------------------------- bench airframe */
typedef struct {
    double V, gam, theta, q, h, alpha;      /* m/s, rad */
    double phi, p, r, beta, psi;
    double ths_deg, thrust;                 /* stabiliser, N */
    double nz;
    double m, pitch_power, roll_power;      /* kg, control-power multipliers */
    int flaps, athr;
    double v_target;
} Bench;

static void bench_init(Bench *b, double kias, int flaps, double pitch_power, double roll_power)
{
    memset(b, 0, sizeof(*b));
    b->V = kias * 0.514444; b->m = 450000; b->flaps = flaps;
    b->pitch_power = pitch_power; b->roll_power = roll_power;
    b->h = 3000; b->athr = 1; b->v_target = b->V;
    b->ths_deg = 0; b->nz = 1;
}

static double bench_cl0(const Bench *b) { return b->flaps ? 0.9 : 0.25; }

/* Trim the bench: α for 1 g, then the stabiliser for zero elevator, and the thrust. */
static void bench_trim(Bench *b)
{
    const double rho = 1.225, S = 845, W = b->m * G0;
    double qbar = 0.5 * rho * b->V * b->V, cl = W / (qbar * S);
    b->alpha = (cl - bench_cl0(b)) / 5.5;
    b->theta = b->alpha; b->gam = 0;
    /* Cm = 0.02 - 1.2 (α - 0) + 0.06 * ths(deg)*D2R*... : solve ths for Cm = 0 with zero elevator. */
    b->ths_deg = -(0.02 - 1.2 * b->alpha) / (2.0 * D2R);
    b->thrust = qbar * S * (0.022 + 0.045 * cl * cl);
}

/* One bench step with the FBW surface orders (−1..1, + nose up / right). */
static void bench_step(Bench *b, double pitch, double roll, double yaw, double dt)
{
    const double rho = 1.225, S = 845, c = 12.3, Iyy = 6.5e7, W = b->m * G0;
    double qbar = 0.5 * rho * b->V * b->V, qref = 0.5 * rho * pow(250 * 0.514444, 2), sc = qbar / qref;
    double cl, cd, cm, lift, drag, de = pitch * 25 * D2R;
    b->alpha = b->theta - b->gam;
    cl = bench_cl0(b) + 5.5 * b->alpha + 0.35 * de * b->pitch_power;
    cl = fmin(cl, b->flaps ? 2.4 : 1.45);                              /* stall plateau */
    cd = 0.022 + 0.045 * cl * cl;
    cm = 0.02 - 1.2 * b->alpha - 25.0 * b->q * c / (2 * b->V) + 1.6 * de * b->pitch_power + 2.0 * b->ths_deg * D2R;
    lift = qbar * S * cl; drag = qbar * S * cd;
    b->thrust = fmax(0.0, fmin(1.2e6, b->thrust + (b->athr ? 0.02 * W * (b->v_target - b->V) * dt : 0)));
    b->V += ((b->thrust * cos(b->alpha) - drag) / b->m - G0 * sin(b->gam)) * dt;
    b->gam += ((lift + b->thrust * sin(b->alpha)) * cos(b->phi) - W * cos(b->gam)) / (b->m * b->V) * dt;
    b->q += qbar * S * c * cm / Iyy * dt;
    /* lateral-directional, derivatives scaled with dynamic pressure */
    {
        double Lp = -0.8 * sqrt(sc), Lda = 0.35 * sc * b->roll_power, Lb = -0.8 * sc, Nb = 0.5 * sc, Nr = -0.25 * sqrt(sc);
        double Ndr = 0.12 * sc, Yb = -0.1 * sqrt(sc);
        b->p += (Lp * b->p + Lda * roll + Lb * b->beta) * dt;
        b->r += (Nb * b->beta + Nr * b->r + Ndr * yaw) * dt;
        b->beta += (Yb * b->beta - b->r + G0 / b->V * sin(b->phi) * cos(b->theta)) * dt;
        b->phi += (b->p + (b->q * sin(b->phi) + b->r * cos(b->phi)) * tan(b->theta)) * dt;
        b->psi += (b->q * sin(b->phi) + b->r * cos(b->phi)) / cos(b->theta) * dt;
    }
    b->theta += (b->q * cos(b->phi) - b->r * sin(b->phi)) * dt;
    b->h += b->V * sin(b->gam) * dt;
    b->nz = lift / W;
}

static void bench_inputs(const Bench *b, A380XFctlInputs *i, double stick_pitch, double stick_roll, double dt)
{
    memset(i, 0, sizeof(*i));
    i->dt = dt; i->enabled = i->powered = i->adr_valid = i->ir_valid = 1;
    i->stick_pitch = stick_pitch; i->stick_roll = stick_roll;
    i->ias_kt = i->tas_kt = b->V / 0.514444; i->gs_kt = i->ias_kt; i->mach = b->V / 340.0;
    i->vs_fpm = b->V * sin(b->gam) / 0.00508;
    i->pitch_deg = b->theta * R2D; i->roll_deg = b->phi * R2D; i->alpha_deg = (b->theta - b->gam) * R2D;
    i->q_dps = b->q * R2D; i->p_dps = b->p * R2D; i->r_dps = b->r * R2D; i->nz = b->nz;
    i->ra_valid = 1; i->ra_ft = b->h / 0.3048; i->flap_index = b->flaps ? 3 : 0;
}

typedef struct { double max_nz, min_nz, max_theta, min_theta, max_alpha, max_phi, q_rms, h_dev, phi_end, gam_dev, max_kt; } Stats;

typedef double (*StickFn)(double t, const Bench *b);
static double zero(double t, const Bench *b) { (void)t; (void)b; return 0; }

/* Fly a case: settle 20 s hands off in the flight law, then the stick inputs for `seconds`. */
static Stats fly(Bench *b, A380XFctlState *s, StickFn pitch_fn, StickFn roll_fn, double seconds, double from_stats)
{
    const double dt = 0.02;
    Stats st;
    double t, h0 = 0, n = 0, gam0 = 0;
    A380XFctlInputs in; A380XFctlOutputs out;
    memset(&st, 0, sizeof(st));
    st.max_nz = st.max_theta = st.max_alpha = st.max_phi = -1e9; st.min_nz = st.min_theta = 1e9;
    for (t = 0; t < seconds; t += dt) {
        bench_inputs(b, &in, pitch_fn(t, b), roll_fn(t, b), dt);
        a380x_fctl_step(s, &in, &out);
        if (out.auto_trim) b->ths_deg += out.trim_rate_deg_s * dt;
        bench_step(b, out.pitch, out.roll, out.yaw, dt);
        assert(isfinite(b->V) && isfinite(b->theta) && isfinite(b->phi));
        if (t >= from_stats) {
            if (n == 0) { h0 = b->h; gam0 = b->gam * R2D; }
            st.max_nz = fmax(st.max_nz, b->nz); st.min_nz = fmin(st.min_nz, b->nz);
            st.max_theta = fmax(st.max_theta, b->theta * R2D); st.min_theta = fmin(st.min_theta, b->theta * R2D);
            st.max_alpha = fmax(st.max_alpha, (b->theta - b->gam) * R2D); st.max_phi = fmax(st.max_phi, fabs(b->phi * R2D));
            st.q_rms += b->q * R2D * b->q * R2D; n += 1; st.max_kt = fmax(st.max_kt, b->V / 0.514444);
            st.h_dev = fmax(st.h_dev, fabs(b->h - h0)); st.gam_dev = fmax(st.gam_dev, fabs(b->gam * R2D - gam0));
        }
    }
    st.q_rms = n > 0 ? sqrt(st.q_rms / n) : 0; st.phi_end = b->phi * R2D;
    return st;
}

static void airborne(A380XFctlState *s, Bench *b)
{
    /* Enter the flight law (liftoff blend completed) with the bench trimmed. */
    a380x_fctl_init(s);
    s->mode = A380X_FCTL_FLIGHT; s->blend = 1; s->airborne_s = 100;
    bench_trim(b);
}

static double pull_03(double t, const Bench *b) { (void)b; return t < 5 ? 0.3 : 0; }
static double full_aft(double t, const Bench *b) { (void)t; (void)b; return 1.0; }
static double full_fwd(double t, const Bench *b) { (void)b; return t < 15 ? -1.0 : 0; }
static double roll_half(double t, const Bench *b) { (void)b; return t < 2.0 ? 0.5 : 0; }
static double roll_full(double t, const Bench *b) { (void)b; return t < 12 ? 1.0 : 0; }
static double roll_to_30(double t, const Bench *b) { return (t < 20 && b->phi * R2D < 30) ? 0.4 : 0; }

static void test_closed_loop(double power, const char *label)
{
    Bench b; A380XFctlState s; Stats st;
    /* 1. hands-off clean 250 kt and approach 150 kt: steady, path and altitude held, trim unloads */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    st = fly(&b, &s, zero, zero, 90, 20);
    CHECK(st.q_rms < 0.3 && st.h_dev < 120, "%s clean hands off: q rms %.2f, altitude drift %.0f m", label, st.q_rms, st.h_dev);
    CHECK(fabs(s.pitch_int) < 0.08, "%s auto-trim unloads the elevator: %.3f", label, s.pitch_int);
    bench_init(&b, 150, 1, power, power); airborne(&s, &b);
    st = fly(&b, &s, zero, zero, 60, 20);
    CHECK(st.q_rms < 0.3 && st.h_dev < 120, "%s approach hands off: q rms %.2f, drift %.0f m", label, st.q_rms, st.h_dev);
    bench_init(&b, 320, 0, power, power); airborne(&s, &b);
    st = fly(&b, &s, zero, zero, 60, 20);
    CHECK(st.q_rms < 0.3 && st.h_dev < 120, "%s 320 kt hands off: q rms %.2f, drift %.0f m", label, st.q_rms, st.h_dev);
    bench_init(&b, 140, 1, power, power); b.flaps = 1; airborne(&s, &b);
    st = fly(&b, &s, zero, zero, 60, 20);
    CHECK(st.q_rms < 0.3 && st.h_dev < 120, "%s 140 kt flaps hands off: q rms %.2f, drift %.0f m", label, st.q_rms, st.h_dev);
    /* high-speed protection: full forward stick at level-flight thrust from 330 kt, then released */
    bench_init(&b, 330, 0, power, power); airborne(&s, &b); b.athr = 0; b.h = 9000;
    st = fly(&b, &s, full_fwd, zero, 15, 0);
    CHECK(st.max_kt < 340 + 35, "%s high-speed protection in a dive: %.0f kt", label, st.max_kt);
    st = fly(&b, &s, zero, zero, 60, 40);
    CHECK(st.max_kt < 342, "%s back below VMO after the release: %.0f kt", label, st.max_kt);
    /* 2. small pull: about 1.2 g, then the new path is held */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    fly(&b, &s, zero, zero, 20, 99);
    st = fly(&b, &s, pull_03, zero, 5, 0);
    CHECK(st.max_nz > 1.08 && st.max_nz < 1.45, "%s 0.3 pull: max nz %.2f", label, st.max_nz);
    st = fly(&b, &s, zero, zero, 30, 15);   /* settled 15 s after the release */
    CHECK(st.gam_dev < 0.6 && st.q_rms < 0.3, "%s path held after release: γ dev %.2f q rms %.2f", label, st.gam_dev, st.q_rms);
    /* 3. full aft stick clean: load factor and attitude limits; α protection at low energy */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    st = fly(&b, &s, full_aft, zero, 25, 0);
    CHECK(st.max_nz < 2.65 && st.max_theta < 31.0, "%s full aft: nz %.2f θ %.1f", label, st.max_nz, st.max_theta);
    bench_init(&b, 180, 0, power, power); airborne(&s, &b); b.athr = 0; b.thrust *= 0.3;
    st = fly(&b, &s, full_aft, zero, 60, 0);
    {
        double ap, am; a380x_fctl_alpha_limits(0, 0.3, &ap, &am);
        CHECK(st.max_alpha < am + 1.0, "%s full aft at low energy: α %.1f (α max %.1f)", label, st.max_alpha, am);
    }
    /* 4. full forward: −15° pitch limit, load factor ≥ −1 g */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    st = fly(&b, &s, full_fwd, zero, 15, 0);
    CHECK(st.min_theta > -16.0 && st.min_nz > -1.15, "%s full forward: θ %.1f nz %.2f", label, st.min_theta, st.min_nz);
    /* 5. roll: rate demand, bank held when released, 67° limit, back to 33° */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    fly(&b, &s, zero, roll_half, 3, 99);
    {
        double phi_rel = b.phi * R2D;
        st = fly(&b, &s, zero, zero, 15, 3);
        CHECK(fabs(st.phi_end - phi_rel) < 4.0 && fabs(phi_rel) > 5.0, "%s bank held: released at %.1f, %.1f after 15 s", label, phi_rel, st.phi_end);
    }
    bench_init(&b, 220, 0, power, power); airborne(&s, &b);   /* (a steep descending bank at 250 kt reaches HSP: wings level) */
    st = fly(&b, &s, zero, roll_full, 12, 0);
    CHECK(st.max_phi < 68.5 && st.max_phi > 55, "%s full roll: max bank %.1f", label, st.max_phi);
    st = fly(&b, &s, zero, zero, 25, 20);
    CHECK(fabs(fabs(st.phi_end) - 33.0) < 3.0, "%s released beyond 33: %.1f", label, st.phi_end);
    /* 6. 30° turn, stick neutral: pitch compensation keeps the altitude */
    bench_init(&b, 250, 0, power, power); airborne(&s, &b);
    fly(&b, &s, zero, zero, 10, 99);
    st = fly(&b, &s, zero, roll_to_30, 40, 10);
    CHECK(st.h_dev < 120 && fabs(st.phi_end) > 25, "%s 30° turn: altitude change %.0f m, bank %.1f", label, st.h_dev, st.phi_end);
}

/* ---------------------------------------------------------------- unit checks */
static void base(A380XFctlInputs *i)
{
    memset(i, 0, sizeof(*i));
    i->dt = 0.05; i->enabled = i->powered = i->adr_valid = i->ir_valid = 1;
    i->ias_kt = i->tas_kt = 250; i->gs_kt = 250; i->mach = 0.45; i->nz = 1; i->pitch_deg = 3;
    i->ra_valid = 1; i->ra_ft = 5000;
}

static void test_modes_and_bypass(void)
{
    A380XFctlState s; A380XFctlInputs i; A380XFctlOutputs o; int k;
    a380x_fctl_init(&s); base(&i);
    i.enabled = 0; i.stick_pitch = 0.4; a380x_fctl_step(&s, &i, &o);
    CHECK(o.law == A380X_FCTL_LAW_OFF && !o.active && !o.steering && o.pitch == 0.4, "off: X-Plane's own controls");
    i.enabled = 1; i.on_ground = 1; i.gs_kt = 0; i.ias_kt = 0;
    a380x_fctl_step(&s, &i, &o);
    CHECK(o.mode == A380X_FCTL_GROUND && o.pitch == 0.4 && o.artstab_pitch == 0 && o.steering, "ground mode is direct");
    /* liftoff: the order is continuous, the law blends in over 5 s */
    i.on_ground = 0; i.ias_kt = i.tas_kt = 160; i.q_dps = 2;
    a380x_fctl_step(&s, &i, &o);
    CHECK(o.mode == A380X_FCTL_FLIGHT && fabs(o.pitch - 0.4) < 0.05, "liftoff bumpless: %.3f", o.pitch);
    for (k = 0; k < 120; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(s.blend > 0.999 && o.active, "flight law after 5 s");
    /* AP engaged: no pitch/roll correction, the AP order is tracked */
    i.q_dps = 0; i.ap_pitch_roll = 1; i.ap_pitch = 0.12; i.ap_roll = -0.05; i.stick_pitch = 0.12;
    a380x_fctl_step(&s, &i, &o);
    CHECK(o.artstab_pitch == 0 && o.artstab_roll == 0 && !o.auto_trim && fabs(s.pitch_int - 0.12) < 1e-9, "AP bypass");
    i.ap_pitch_roll = 0; i.stick_pitch = 0;
    a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.pitch - 0.12) < 0.02 && fabs(o.roll + 0.05) < 0.02, "AP disconnect handover: %.3f %.3f", o.pitch, o.roll);
    /* IR lost: direct law, pass-through, no corrections, no auto-trim */
    i.ir_valid = 0; i.stick_pitch = -0.3; a380x_fctl_step(&s, &i, &o);
    CHECK(o.law == A380X_FCTL_LAW_DIRECT && o.pitch == -0.3 && !o.active && !o.auto_trim, "direct law");
    /* flare: below 100 ft descending, attitude lowered 2° over 8 s with the stick released */
    i.ir_valid = 1; i.stick_pitch = 0; for (k = 0; k < 200; ++k) a380x_fctl_step(&s, &i, &o);
    i.ra_ft = 90; i.vs_fpm = -700; i.pitch_deg = 4;
    a380x_fctl_step(&s, &i, &o);
    CHECK(o.mode == A380X_FCTL_FLARE, "flare mode at 100 ft");
    i.ra_ft = 150; i.vs_fpm = 1500; a380x_fctl_step(&s, &i, &o);
    CHECK(o.mode == A380X_FCTL_FLIGHT, "go-around leaves the flare law");
    i.on_ground = 1; for (k = 0; k < 50; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(o.mode == A380X_FCTL_GROUND && s.blend == 0, "touchdown: ground mode within 2 s");
}

static void test_protections(void)
{
    A380XFctlState s; A380XFctlInputs i; A380XFctlOutputs o; int k;
    a380x_fctl_init(&s); base(&i); s.mode = A380X_FCTL_FLIGHT; s.blend = 1; s.airborne_s = 100;
    i.ias_kt = 360; i.mach = 0.6; a380x_fctl_step(&s, &i, &o);
    CHECK((o.protections & A380X_FCTL_PROT_HIGH_SPEED) && o.nz_cmd > 1.0 && !o.auto_trim, "high speed: nose-up demand, trim frozen");
    i.ias_kt = 330; a380x_fctl_step(&s, &i, &o);
    CHECK(!(o.protections & A380X_FCTL_PROT_HIGH_SPEED), "high speed protection ends below VMO");
    i.ias_kt = 160; i.mach = 0.25; i.alpha_deg = 13; for (k = 0; k < 40; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK((o.protections & A380X_FCTL_PROT_ALPHA) && o.alpha_floor && !o.auto_trim, "high AoA protection + A.FLOOR");
    i.alpha_deg = 5; i.stick_pitch = -0.8; a380x_fctl_step(&s, &i, &o);
    CHECK(!(o.protections & A380X_FCTL_PROT_ALPHA), "nose-down stick leaves α protection");
}

static void test_steering(void)
{
    A380XFctlState s; A380XFctlInputs i; A380XFctlOutputs o; int k;
    a380x_fctl_init(&s); base(&i); i.on_ground = 1; i.ias_kt = i.tas_kt = 5;
    i.tiller_hardware = 1; i.tiller = 1; i.gs_kt = 10;
    for (k = 0; k < 100; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.nose_deg - 70) < 1e-6, "tiller full at taxi speed: %.1f", o.nose_deg);
    CHECK(fabs(o.body_deg + 15) < 1e-6, "body-wheel steering opposite, 15°: %.1f", o.body_deg);
    i.gs_kt = 45; for (k = 0; k < 100; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.nose_deg - 35) < 0.5 && fabs(o.body_deg) < 1e-6, "tiller halved at 45 kt, BWS off: %.1f %.1f", o.nose_deg, o.body_deg);
    i.tiller = 0; i.pedals = 1; i.gs_kt = 85; for (k = 0; k < 100; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.nose_deg - 3) < 0.05, "pedals ±6° fading to 0 at 130 kt: %.2f", o.nose_deg);
    i.tiller_hardware = 0; i.gs_kt = 5; for (k = 0; k < 100; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.nose_deg - 70) < 1e-6, "no hardware tiller: pedals steer 70° at taxi speed: %.1f", o.nose_deg);
    i.pedals = -1; a380x_fctl_step(&s, &i, &o);
    CHECK(o.nose_deg > 68.9, "nose-gear slew limited (20°/s): %.1f", o.nose_deg);
    i.on_ground = 0; for (k = 0; k < 200; ++k) a380x_fctl_step(&s, &i, &o);
    CHECK(fabs(o.nose_deg) < 1e-6, "nose wheel centred in flight");
}

int main(void)
{
    test_modes_and_bypass();
    test_protections();
    test_steering();
    test_closed_loop(1.0, "nominal");
    test_closed_loop(0.5, "half control power");
    test_closed_loop(2.0, "double control power");
    if (failures) { printf("%d of %d checks failed\n", failures, checks); return 1; }
    printf("A380 flight controls: %d checks passed (modes, AP bypass, direct law, protections, steering, "
           "closed loop at 0.5/1/2x control power)\n", checks);
    return 0;
}
