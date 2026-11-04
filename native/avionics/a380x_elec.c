/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 electrical network core. See a380x_elec.h and docs/ELEC_SYSTEM_SPEC.md.
 * Statement order follows the FlyByWire A380X Rust model (mod.rs update(),
 * alternating_current.rs, direct_current.rs); line references are in the spec.
 */
#include "a380x_elec.h"
#include <math.h>
#include <string.h>

const char *const a380x_eln_bus_names[A380X_ELN_BUS_COUNT] = {
    "ac1", "ac2", "ac3", "ac4", "ac_ess", "ac_emer", "ac_eha", "ac_gnd_svc",
    "dc1", "dc2", "dc_ess", "dc_ess_inflt", "dc_eha", "dc_apu", "apu_start",
    "hot1", "hot2", "hot_ess", "hot_apu", "dc_gnd_svc"
};
const char *const a380x_eln_contactor_names[A380X_ELN_K_COUNT] = {
    "990XU1", "990XU2", "990XU3", "990XU4", "990XG1", "990XG2", "990XG3", "990XG4",
    "990XS1", "990XS2", "980XU1", "980XU2", "980XU3", "980XU4", "980XU5", "980XU6",
    "900XU", "3XC1", "3XC2", "5XE", "3XB_1", "3XB_2", "911XN", "911XH", "991PU6", "991PU2",
    "990PU1", "990PU2", "6PE", "7PU", "990PB1", "990PB2", "6PC1", "6PB3", "5PB",
    "6PC2", "980PC", "7XB", "970PN2", "14PH", "990PX", "970PN", "10PH", "10KA_5KA"
};
const char *const a380x_eln_du_names[A380X_ELN_DU_COUNT] = {
    "capt_pfd", "capt_nd", "capt_mfd", "fo_pfd", "fo_nd", "fo_mfd", "ewd", "sd", "capt_oit", "fo_oit"
};

/* ---------------------------------------------------------------- union-find "electricity" */
enum {
    N_BUS0 = 0,
    N_K0 = A380X_ELN_BUS_COUNT,
    N_GEN1 = N_K0 + A380X_ELN_K_COUNT, N_APU_A = N_GEN1 + 4, N_APU_B, N_EXT1,
    N_EMER_GEN = N_EXT1 + 4,
    N_TR_IN1, N_TR_OUT1 = N_TR_IN1 + A380X_ELN_TRS,   /* TR1, TR2, TR ESS, TR APU */
    N_SI_IN = N_TR_OUT1 + A380X_ELN_TRS, N_SI_OUT,
    N_BAT1, N_COUNT = N_BAT1 + A380X_ELN_BATTERIES
};
typedef struct {
    int parent[N_COUNT];
    unsigned char conductive[N_COUNT];
    uint32_t src[N_COUNT];   /* aggregated at the root */
    float pot[N_COUNT];      /* aggregated maximum potential at the root */
} Net;

static int net_find(Net *n, int x)
{
    int root = x;
    while (n->parent[root] != root) root = n->parent[root];
    while (n->parent[x] != root) { int next = n->parent[x]; n->parent[x] = root; x = next; }
    return root;
}
/* The component's origin is the set of sources at the highest potential (a charging battery
 * below its TR does not count as a supplier, like FBW's potential merge). */
static void net_merge(Net *n, int root, uint32_t src, float pot)
{
    if (!src) return;
    if (!n->src[root] || pot > n->pot[root] + 0.01f) { n->src[root] = src; n->pot[root] = pot; }
    else if (pot >= n->pot[root] - 0.01f) { n->src[root] |= src; if (pot > n->pot[root]) n->pot[root] = pot; }
}
static void net_flow(Net *n, int a, int b)
{
    int ra, rb;
    if (!n->conductive[a] || !n->conductive[b]) return;
    ra = net_find(n, a); rb = net_find(n, b);
    if (ra == rb) return;
    n->parent[rb] = ra;
    net_merge(n, ra, n->src[rb], n->pot[rb]);
}
static void net_supply(Net *n, int element, int source, float potential)
{
    int r;
    if (!n->conductive[element] || potential <= 0.0f) return;
    r = net_find(n, element);
    net_merge(n, r, 1u << (unsigned)source, potential);
}
static int net_powered(Net *n, int element)
{
    return n->conductive[element] && n->src[net_find(n, element)] != 0;
}
static float net_potential(Net *n, int element)
{
    return net_powered(n, element) ? n->pot[net_find(n, element)] : 0.0f;
}
#define BUS(b) (N_BUS0 + (b))
#define KN(k) (N_K0 + (k))

/* Contactor helpers: the closed state lives in s->contactors and in the conductive flag. */
static void k_set(A380XElnState *s, Net *n, int k, int closed)
{
    uint64_t bit = (uint64_t)1 << (unsigned)k;
    if (closed) s->contactors |= bit; else s->contactors &= ~bit;
    n->conductive[KN(k)] = (unsigned char)(closed != 0);
}
static int k_closed(const A380XElnState *s, int k) { return a380x_eln_contactor(s, k); }

/* ---------------------------------------------------------------- battery model (BAT:98-135) */
double a380x_eln_battery_potential(double q)
{
    if (!(q > 0.0)) return 0.0;
    if (q <= 3.488) return 13.95303731988 * q - 2.0 * q * q;
    if (q < 22.449) return 23.85 + 0.14 * q;
    return 8483298.0 + -2373273.312763873 * q + 276476.10619333945 * pow(q, 2) +
           -17167.409762003314 * pow(q, 3) + 599.2597390001015 * pow(q, 4) +
           -11.149802489333474 * pow(q, 5) + 0.08638809969727154 * pow(q, 6);
}

/* ---------------------------------------------------------------- AC 1-4 source selection (AC:297-385,585-634) */
enum { PS_NONE, PS_GEN, PS_EXT, PS_APU1, PS_APU2, PS_AC1, PS_AC2, PS_AC3, PS_AC4 };
static const unsigned char normal_table[8][4] = {
    {PS_GEN, PS_GEN, PS_GEN, PS_GEN}, {PS_EXT, PS_EXT, PS_EXT, PS_EXT},
    {PS_APU1, PS_APU1, PS_APU2, PS_APU2}, {PS_APU2, PS_APU2, PS_APU1, PS_APU1},
    {PS_AC2, PS_AC1, PS_AC4, PS_AC3}, {PS_AC2, PS_AC1, PS_AC4, PS_AC3},
    {PS_AC4, PS_AC3, PS_AC2, PS_AC1}, {PS_AC4, PS_AC3, PS_AC2, PS_AC1},
};
static const unsigned char special_table[8][4] = {
    {PS_GEN, PS_GEN, PS_GEN, PS_GEN}, {PS_EXT, PS_EXT, PS_EXT, PS_EXT},
    {PS_APU1, PS_AC1, PS_AC4, PS_APU2}, {PS_AC2, PS_AC1, PS_AC4, PS_AC3},
    {PS_AC2, PS_APU1, PS_APU2, PS_AC3}, {PS_APU2, PS_AC3, PS_AC2, PS_APU1},
    {PS_NONE, PS_AC3, PS_AC2, PS_NONE}, {PS_NONE, PS_APU2, PS_APU1, PS_NONE},
};
static void calc_ac_sources(const int gen[4], const int ext[4], const int apu_in[2], int in_flight,
                            unsigned char powered_by[4])
{
    int apu[2] = {apu_in[0], apu_in[1]};
    int ext_any = 0, gens = 0, apus, i, row;
    int can_power_other[4] = {0, 0, 0, 0}, apu_count[2] = {0, 0};
    const unsigned char (*table)[4] = normal_table;
    if (in_flight) {
        int left = gen[0] + gen[1], right = gen[2] + gen[3];
        int a = apu[0] && (left <= right || !apu[1]);
        int b = apu[1] && (left > right || !apu[0]);
        apu[0] = a; apu[1] = b;
    }
    for (i = 0; i < 4; ++i) { ext_any |= ext[i]; gens += gen[i] != 0; }
    apus = (apu[0] != 0) + (apu[1] != 0);
    if (!ext_any && gens == 1 && apus == 1 && !((gen[0] && apu[0]) || (gen[3] && apu[1]))) table = special_table;
    memset(powered_by, PS_NONE, 4);
    for (row = 0; row < 8; ++row) {
        int next_count[2] = {apu_count[0], apu_count[1]}, all = 1;
        for (i = 0; i < 4; ++i) {
            int source = table[row][i], valid = 0;
            if (powered_by[i] != PS_NONE) continue;
            switch (source) {
            case PS_GEN: valid = gen[i]; break;
            case PS_EXT: valid = ext[i]; break;
            case PS_APU1: case PS_APU2: valid = apu[source - PS_APU1] && apu_count[source - PS_APU1] < 2; break;
            case PS_AC1: case PS_AC2: case PS_AC3: case PS_AC4: valid = can_power_other[source - PS_AC1]; break;
            default: valid = 0; break;
            }
            if (!valid) continue;
            powered_by[i] = (unsigned char)source;
            can_power_other[i] = source == PS_GEN || source == PS_EXT;
            if (source >= PS_AC1) can_power_other[source - PS_AC1] = 0;
            if (source == PS_APU1 || source == PS_APU2) ++next_count[source - PS_APU1];
        }
        for (i = 0; i < 4; ++i) all &= powered_by[i] != PS_NONE;
        if (all) break;
        apu_count[0] = next_count[0]; apu_count[1] = next_count[1];
    }
}

/* ---------------------------------------------------------------- BCRU (BCRU:68-275) */
enum { BCRU_BAT_OFF, BCRU_OPEN, BCRU_CLOSED };
static void bcru_before(A380XElnBcru *b, int input_powered, int ground_servicing, double bat_v, double dt)
{
    if (input_powered || (b->ground_servicing && !ground_servicing)) b->loss_of_ac = 0.0;
    else b->loss_of_ac += dt;
    b->ground_servicing = ground_servicing;
    b->input_powered = input_powered;
    b->line_closed = !ground_servicing &&
        (input_powered || (b->line_closed && b->loss_of_ac < A380X_ELN_BCRU_LINE_HOLD_S));
    b->gnd_svc_closed = ground_servicing && input_powered;
    b->soc_low = bat_v < A380X_ELN_BAT_SOC_LOW_V;
}
static void bcru_state(A380XElnBcru *b, int bat_auto, int emergency_config)
{
    switch (b->state) {
    case BCRU_BAT_OFF:
        if (bat_auto || emergency_config) b->state = BCRU_OPEN;
        break;
    case BCRU_OPEN:
        if (emergency_config || b->ground_servicing) b->state = BCRU_OPEN;
        else if (!bat_auto) b->state = BCRU_BAT_OFF;
        else if (b->loss_of_ac <= 0.0) b->state = BCRU_CLOSED;
        break;
    default:
        if (emergency_config || b->ground_servicing || b->loss_of_ac >= A380X_ELN_BCRU_BAT_OPEN_S) b->state = BCRU_OPEN;
        else if (!bat_auto) b->state = BCRU_BAT_OFF;
        break;
    }
}

/* ---------------------------------------------------------------- init */
void a380x_eln_spawn_controls(int spawn_mode, A380XElnControls *c)
{
    int i;
    memset(c, 0, sizeof(*c));
    for (i = 0; i < 4; ++i) {
        c->bat_auto[i] = (unsigned char)(spawn_mode != A380X_ELN_SPAWN_COLD_DARK);
        c->gen_on[i] = 1;
        c->ext_on[i] = (unsigned char)(spawn_mode == A380X_ELN_SPAWN_TURNAROUND);
    }
    c->apu_gen_on[0] = c->apu_gen_on[1] = 1;
    c->bus_tie_auto = c->ac_ess_feed_normal = c->galley_auto = c->pax_sys_on = 1;
    c->comm_on[0] = c->comm_on[1] = c->elmu_on = 1;
}

void a380x_eln_init(A380XElnState *s, int spawn_mode)
{
    int i;
    memset(s, 0, sizeof(*s));
    s->spawn_mode = spawn_mode;
    for (i = 0; i < 4; ++i) {
        s->bat_ah[i] = A380X_ELN_BAT_CAPACITY_AH;
        s->bat_v[i] = (float)a380x_eln_battery_potential(s->bat_ah[i]);
        s->vfg_stable_s[i] = A380X_ELN_VFG_STABLE_S;  /* FBW starts at the stabilisation threshold */
        s->drive_connected[i] = 1;
    }
    for (i = 0; i < 3; ++i) s->bcru[i].state = BCRU_OPEN;
    s->sply2_timer = 1e9;
    s->rng = 0x2545F491u;
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) s->du[i].state = A380X_ELN_DU_OFF;
}

int a380x_eln_any_ac_main(const A380XElnState *s)
{
    return (s->powered & (A380X_ELN_BIT(A380X_ELN_AC1) | A380X_ELN_BIT(A380X_ELN_AC2) |
                          A380X_ELN_BIT(A380X_ELN_AC3) | A380X_ELN_BIT(A380X_ELN_AC4))) != 0;
}

/* ---------------------------------------------------------------- one network evaluation */
static double bus_load_w(int bus)
{
    switch (bus) {
    case A380X_ELN_DC_ESS: return A380X_ELN_LOAD_DC_ESS_W;
    case A380X_ELN_DC1: case A380X_ELN_DC2: return A380X_ELN_LOAD_DC_MAIN_W;
    case A380X_ELN_DC_APU: return A380X_ELN_LOAD_DC_APU_W;
    case A380X_ELN_HOT1: case A380X_ELN_HOT2: case A380X_ELN_HOT_ESS: case A380X_ELN_HOT_APU:
        return A380X_ELN_LOAD_HOT_W;
    default: return 0.0;
    }
}

static void eln_evaluate(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in, double dt)
{
    Net net, *n = &net;
    unsigned char powered_by[4];
    int i, j, gen_avail[4], ext_supply[4], ext_avail[4], apu_supply[2], apu_avail[2];
    int any_ac, ac3, dc_ess, rat_deploy, elc, ess_lc, emer_cfg_ess, emer_evac;
    int prev_6pc2 = s->dc_tie_6pc2, prev_980pc = s->dc_tie_980pc;
    const A380XElnFailures *f = &in->fail;
    float bat_pot[4];

    memset(n, 0, sizeof(*n));
    for (i = 0; i < N_COUNT; ++i) { n->parent[i] = i; n->conductive[i] = 1; }
    for (i = 0; i < A380X_ELN_K_COUNT; ++i) n->conductive[KN(i)] = (unsigned char)k_closed(s, i);
    /* Bus failures (not in FBW): the bus is forced unpowered regardless of sources. */
    for (i = 0; i < 4; ++i) if (f->ac_bus[i]) n->conductive[BUS(A380X_ELN_AC1 + i)] = 0;
    if (f->dc_bus[0]) n->conductive[BUS(A380X_ELN_DC1)] = 0;
    if (f->dc_bus[1]) n->conductive[BUS(A380X_ELN_DC2)] = 0;
    if (f->dc_bus[2]) n->conductive[BUS(A380X_ELN_DC_ESS)] = 0;
    for (i = 0; i < 4; ++i) bat_pot[i] = (float)a380x_eln_battery_potential(s->bat_ah[i]);

    /* -- 1. main sources: VFGs (EGEN:75-137), APU generators, EXT ------------------- */
    for (i = 0; i < 4; ++i) {
        int activated = c->gen_on[i] && !in->eng_fire_released[i];
        double n2 = isfinite(in->eng_n2[i]) ? in->eng_n2[i] : 0.0;
        int normal;
        s->drive_connected[i] = !c->drive_disc[i];
        s->gen_hz[i] = activated && s->drive_connected[i]
            ? fmin(fmax(0.0, n2), A380X_ELN_VFG_N2_CAP_PCT) * A380X_ELN_VFG_HZ_PER_N2_PCT : 0.0;
        normal = s->gen_hz[i] >= A380X_ELN_VFG_HZ_MIN && s->gen_hz[i] <= A380X_ELN_VFG_HZ_MAX;
        if (!activated) s->vfg_stable_s[i] = 0.0;
        else s->vfg_stable_s[i] = normal ? fmin(A380X_ELN_VFG_STABLE_S, s->vfg_stable_s[i] + dt) : 0.0;
        gen_avail[i] = s->vfg_stable_s[i] >= A380X_ELN_VFG_STABLE_S && activated && normal && !f->gen[i];
        s->gen_avail[i] = gen_avail[i];
        s->gen_volts[i] = gen_avail[i] ? 115.0f : 0.0f;
        net_supply(n, N_GEN1 + i, A380X_ELN_SRC_GEN1 + i, s->gen_volts[i]);
    }
    for (j = 0; j < 2; ++j) {
        apu_supply[j] = in->apu_n >= A380X_ELN_APU_GEN_MIN_N && !f->apu_gen[j];
        apu_avail[j] = c->apu_gen_on[j] && apu_supply[j];
        s->apu_gen_avail[j] = apu_avail[j];
        s->apu_gen_volts[j] = apu_supply[j] ? 115.0f : 0.0f;
        s->apu_gen_hz[j] = apu_supply[j] ? 400.0f : 0.0f;
        net_supply(n, N_APU_A + j, A380X_ELN_SRC_APU_A + j, s->apu_gen_volts[j]);
    }
    for (i = 0; i < 4; ++i) {
        ext_supply[i] = in->ext_connected[i] && !f->ext[i];
        ext_avail[i] = c->ext_on[i] && ext_supply[i];
        s->ext_avail[i] = ext_supply[i];
        net_supply(n, N_EXT1 + i, A380X_ELN_SRC_EXT1 + i, ext_supply[i] ? 115.0f : 0.0f);
    }
    s->ext_volts = ext_supply[0] || ext_supply[1] || ext_supply[2] || ext_supply[3] ? 115.0f : 0.0f;
    s->ext_hz = s->ext_volts > 0.0f ? 400.0f : 0.0f;
    /* in-flight discretes (NAV:24-27): ADR 1/3 low-speed warning, set > 54 kt, reset < 50 kt */
    if (!in->adr_lsw_valid) s->in_flight_lsw1 = 0;
    else if (in->ias_kt > 54.0) s->in_flight_lsw1 = 1;
    else if (in->ias_kt < 50.0) s->in_flight_lsw1 = 0;
    if (!in->adr_lsw_valid) s->lsw3 = 0;
    else if (in->ias_kt > 104.0) s->lsw3 = 1;
    else if (in->ias_kt < 100.0) s->lsw3 = 0;
    calc_ac_sources(gen_avail, ext_avail, apu_avail, s->in_flight_lsw1, powered_by);
    for (i = 0; i < 4; ++i) {
        int bus_number = i + 1, pb = powered_by[i];
        int not_self = !(pb == PS_GEN || pb == PS_EXT || pb == PS_NONE);
        k_set(s, n, A380X_ELN_K_990XU1 + i, pb == PS_GEN);
        k_set(s, n, A380X_ELN_K_990XG1 + i, pb == PS_EXT);
        if (bus_number == 1 || bus_number == 4) {
            int other = 0;
            for (j = 0; j < 4; ++j) if (j != i && powered_by[j] == PS_AC1 + i) other = 1;
            k_set(s, n, A380X_ELN_K_980XU1 + i, not_self || other);
        } else {
            int opposite = i % 2 + 2; /* bus 2 -> 3, bus 3 -> 2 */
            int from_opposite = pb == PS_AC1 + opposite - 1;
            int outer_from_bus = powered_by[0] == PS_AC1 + i || powered_by[3] == PS_AC1 + i;
            int opposite_from_bus = powered_by[opposite - 1] == PS_AC1 + i;
            k_set(s, n, A380X_ELN_K_980XU1 + i, (not_self && !from_opposite) || outer_from_bus);
            k_set(s, n, A380X_ELN_K_980XU1 + i + 3,
                  c->bus_tie_auto && ((not_self && from_opposite) || opposite_from_bus));
        }
    }
    for (j = 0; j < 2; ++j) {
        int any = 0;
        for (i = 0; i < 4; ++i) if (powered_by[i] == PS_APU1 + j) any = 1;
        k_set(s, n, A380X_ELN_K_990XS1 + j, any);
        net_flow(n, N_APU_A + j, KN(A380X_ELN_K_990XS1 + j));
    }
    k_set(s, n, A380X_ELN_K_900XU, c->bus_tie_auto &&
          (powered_by[0] == PS_APU2 || powered_by[1] == PS_APU2 ||
           powered_by[2] == PS_APU1 || powered_by[3] == PS_APU1 ||
           powered_by[0] == PS_AC4 || powered_by[3] == PS_AC1));
    for (i = 0; i < 4; ++i) {
        net_flow(n, N_EXT1 + i, KN(A380X_ELN_K_990XG1 + i));
        net_flow(n, N_GEN1 + i, KN(A380X_ELN_K_990XU1 + i));
    }
    for (j = 0; j < 2; ++j) {
        int apu_k = KN(A380X_ELN_K_990XS1 + j), t0 = KN(A380X_ELN_K_980XU1 + 2 * j), t1 = t0 + 1;
        net_flow(n, apu_k, KN(A380X_ELN_K_900XU));
        net_flow(n, t0, t1);
        net_flow(n, apu_k, t0); net_flow(n, KN(A380X_ELN_K_900XU), t0);
        net_flow(n, apu_k, t1); net_flow(n, KN(A380X_ELN_K_900XU), t1);
    }
    net_flow(n, KN(A380X_ELN_K_980XU5), KN(A380X_ELN_K_980XU6));
    for (i = 0; i < 4; ++i) {
        net_flow(n, KN(A380X_ELN_K_990XU1 + i), BUS(A380X_ELN_AC1 + i));
        net_flow(n, KN(A380X_ELN_K_990XG1 + i), BUS(A380X_ELN_AC1 + i));
        net_flow(n, KN(A380X_ELN_K_980XU1 + i), BUS(A380X_ELN_AC1 + i));
    }
    net_flow(n, KN(A380X_ELN_K_980XU5), BUS(A380X_ELN_AC2));
    net_flow(n, KN(A380X_ELN_K_980XU6), BUS(A380X_ELN_AC3));
    any_ac = 0;
    for (i = 0; i < 4; ++i) any_ac |= net_powered(n, BUS(A380X_ELN_AC1 + i));

    /* -- 2. emergency elec, RAT controller, GCU, RAT, emergency generator (EM:122-144) -- */
    if (!any_ac && in->ias_kt > A380X_ELN_EMER_IAS_KT) s->emer_active_s += dt;
    else s->emer_active_s = 0.0;
    rat_deploy = in->sim_ready && ((s->prev_hot1 && s->emer_active_s > 0.0) ||
                                   (s->prev_hot_ess && c->rat_man_on));
    s->rat_should_deploy = rat_deploy;
    if (rat_deploy) s->rat_latched = 1;
    s->gcu_active = s->emer_active_s > 0.0 || (c->rat_man_on && !in->gear_compressed);
    if (s->rat_latched) s->rat_stow = fmin(1.0, s->rat_stow + A380X_ELN_RAT_DEPLOY_PER_S * dt);
    {
        double target = s->rat_stow >= 1.0 ? A380X_ELN_RAT_GOVERNED_RPM * fmin(1.0, fmax(0.0, in->ias_kt) / 80.0) : 0.0;
        double k = dt > 0.0 ? 1.0 - exp(-dt / A380X_ELN_RAT_SPOOL_TAU_S) : 0.0;
        s->rat_rpm += (target - s->rat_rpm) * k;
    }
    s->emer_gen_supplying = s->gcu_active && s->rat_rpm * A380X_ELN_EMER_GEN_RATIO > 2000.0;
    s->emer_gen_v = s->emer_gen_supplying ? 115.0f : 0.0f;
    net_supply(n, N_EMER_GEN, A380X_ELN_SRC_EMER_GEN, s->emer_gen_v);

    /* -- 3. AC ESS feed, AC ESS, TR APU, 5XE, AC EMER, EHA, ground service (AC:96-156) */
    {
        int ac1 = net_powered(n, BUS(A380X_ELN_AC1)), ac4 = net_powered(n, BUS(A380X_ELN_AC4));
        k_set(s, n, A380X_ELN_K_3XC1, ac1 && c->ac_ess_feed_normal);
        k_set(s, n, A380X_ELN_K_3XC2, ac4 && (!ac1 || !c->ac_ess_feed_normal));
        net_flow(n, BUS(A380X_ELN_AC1), KN(A380X_ELN_K_3XC1));
        net_flow(n, BUS(A380X_ELN_AC4), KN(A380X_ELN_K_3XC2));
        net_flow(n, KN(A380X_ELN_K_3XC1), BUS(A380X_ELN_AC_ESS));
        net_flow(n, KN(A380X_ELN_K_3XC2), BUS(A380X_ELN_AC_ESS));
    }
    net_flow(n, BUS(A380X_ELN_AC4), N_TR_IN1 + A380X_ELN_TR_APU);
    if (net_powered(n, N_TR_IN1 + A380X_ELN_TR_APU) && !f->tr[A380X_ELN_TR_APU])
        net_supply(n, N_TR_OUT1 + A380X_ELN_TR_APU, A380X_ELN_SRC_TR_APU, (float)A380X_ELN_TR_APU_VOLTS);
    k_set(s, n, A380X_ELN_K_5XE, !any_ac && s->emer_gen_supplying);
    net_flow(n, N_EMER_GEN, KN(A380X_ELN_K_5XE));
    net_flow(n, KN(A380X_ELN_K_5XE), BUS(A380X_ELN_AC_ESS));
    /* 3XB.1 / 3XB.2 are evaluated on this statement's previous closed state, then updated. */
    net_flow(n, BUS(A380X_ELN_AC_ESS), KN(A380X_ELN_K_3XB1));
    ac3 = net_powered(n, BUS(A380X_ELN_AC3));
    k_set(s, n, A380X_ELN_K_911XN, ac3);
    net_flow(n, BUS(A380X_ELN_AC3), KN(A380X_ELN_K_911XN));
    k_set(s, n, A380X_ELN_K_991PU6, !any_ac && ext_supply[0]);
    k_set(s, n, A380X_ELN_K_991PU2, ac3);
    net_flow(n, N_EXT1, KN(A380X_ELN_K_991PU6));
    net_flow(n, KN(A380X_ELN_K_991PU2), BUS(A380X_ELN_AC3));
    net_flow(n, KN(A380X_ELN_K_991PU2), BUS(A380X_ELN_AC_GND_SVC));
    net_flow(n, KN(A380X_ELN_K_991PU6), BUS(A380X_ELN_AC_GND_SVC));
    {
        int src = any_ac || s->emer_gen_supplying;
        k_set(s, n, A380X_ELN_K_3XB1, src);
        k_set(s, n, A380X_ELN_K_3XB2, !src);
        net_flow(n, BUS(A380X_ELN_AC_ESS), KN(A380X_ELN_K_3XB1));
        net_flow(n, KN(A380X_ELN_K_3XB1), BUS(A380X_ELN_AC_EMER));
    }

    /* -- 4. DC network (DC:149-331) ------------------------------------------------- */
    net_flow(n, BUS(A380X_ELN_AC2), N_TR_IN1 + A380X_ELN_TR1);
    net_flow(n, KN(A380X_ELN_K_991PU2), N_TR_IN1 + A380X_ELN_TR2);
    net_flow(n, KN(A380X_ELN_K_991PU6), N_TR_IN1 + A380X_ELN_TR2);
    net_flow(n, BUS(A380X_ELN_AC_ESS), N_TR_IN1 + A380X_ELN_TR_ESS);
    {
        static const int line_k[3] = {A380X_ELN_K_990PU1, A380X_ELN_K_990PU2, A380X_ELN_K_6PE};
        static const int dc_bus[3] = {A380X_ELN_DC1, A380X_ELN_DC2, A380X_ELN_DC_ESS};
        static const int bat_of[3] = {A380X_ELN_BAT1, A380X_ELN_BAT2, A380X_ELN_BAT_ESS};
        for (i = 0; i < 3; ++i) {
            /* The BCRU sees "AC" through its own output: a failed unit behaves as AC lost. */
            int input = net_powered(n, N_TR_IN1 + i) && !f->tr[i];
            int gs = i == A380X_ELN_TR2 && k_closed(s, A380X_ELN_K_991PU6);
            if (input) net_supply(n, N_TR_OUT1 + i, A380X_ELN_SRC_TR1 + i, (float)A380X_ELN_TR_VOLTS);
            bcru_before(&s->bcru[i], input, gs, bat_pot[bat_of[i]], dt);
            k_set(s, n, line_k[i], s->bcru[i].line_closed);
            net_flow(n, N_TR_OUT1 + i, KN(line_k[i]));
            net_flow(n, KN(line_k[i]), BUS(dc_bus[i]));
        }
    }
    k_set(s, n, A380X_ELN_K_7PU, net_powered(n, N_TR_OUT1 + A380X_ELN_TR_APU));
    net_flow(n, N_TR_OUT1 + A380X_ELN_TR_APU, KN(A380X_ELN_K_7PU));
    net_flow(n, KN(A380X_ELN_K_7PU), BUS(A380X_ELN_DC_APU));
    /* DC ties read the other tie's previous state (mutual interlock, DC:208-222). */
    s->dc_tie_6pc2 = !prev_980pc && !k_closed(s, A380X_ELN_K_6PE) && net_powered(n, BUS(A380X_ELN_DC1));
    s->dc_tie_980pc = c->bus_tie_auto && !prev_6pc2 &&
        (k_closed(s, A380X_ELN_K_990PU1) != k_closed(s, A380X_ELN_K_990PU2));
    k_set(s, n, A380X_ELN_K_6PC2, s->dc_tie_6pc2);
    k_set(s, n, A380X_ELN_K_980PC, s->dc_tie_980pc);
    net_flow(n, KN(A380X_ELN_K_6PC2), BUS(A380X_ELN_DC1));
    net_flow(n, KN(A380X_ELN_K_6PC2), BUS(A380X_ELN_DC_ESS));
    net_flow(n, KN(A380X_ELN_K_980PC), BUS(A380X_ELN_DC1));
    net_flow(n, KN(A380X_ELN_K_980PC), BUS(A380X_ELN_DC2));
    any_ac = 0;
    for (i = 0; i < 4; ++i) any_ac |= net_powered(n, BUS(A380X_ELN_AC1 + i));
    /* BAT 1 + ELC */
    for (i = 0; i < 4; ++i) n->conductive[N_BAT1 + i] = (unsigned char)!f->bat[i];
    net_supply(n, N_BAT1 + A380X_ELN_BAT1, A380X_ELN_SRC_BAT1, bat_pot[A380X_ELN_BAT1]);
    elc = (c->bat_auto[A380X_ELN_BAT1] && c->bat_auto[A380X_ELN_BAT_ESS] && !s->bcru[0].soc_low && !any_ac) || rat_deploy;
    elc = elc && !f->bat[A380X_ELN_BAT1];
    bcru_state(&s->bcru[0], c->bat_auto[A380X_ELN_BAT1], elc);
    k_set(s, n, A380X_ELN_K_990PB1, s->bcru[0].state == BCRU_CLOSED && !f->bat[A380X_ELN_BAT1]);
    k_set(s, n, A380X_ELN_K_6PC1, elc);
    net_flow(n, KN(A380X_ELN_K_990PB1), N_BAT1 + A380X_ELN_BAT1);
    net_flow(n, KN(A380X_ELN_K_6PC1), N_BAT1 + A380X_ELN_BAT1);
    net_flow(n, BUS(A380X_ELN_HOT1), N_BAT1 + A380X_ELN_BAT1);
    net_flow(n, KN(A380X_ELN_K_990PB1), BUS(A380X_ELN_DC1));
    net_flow(n, KN(A380X_ELN_K_6PC1), BUS(A380X_ELN_DC_ESS));
    /* BAT 2 */
    net_supply(n, N_BAT1 + A380X_ELN_BAT2, A380X_ELN_SRC_BAT2, bat_pot[A380X_ELN_BAT2]);
    bcru_state(&s->bcru[1], c->bat_auto[A380X_ELN_BAT2], 0);
    k_set(s, n, A380X_ELN_K_990PB2, s->bcru[1].state == BCRU_CLOSED && !f->bat[A380X_ELN_BAT2]);
    net_flow(n, KN(A380X_ELN_K_990PB2), N_BAT1 + A380X_ELN_BAT2);
    net_flow(n, BUS(A380X_ELN_HOT2), N_BAT1 + A380X_ELN_BAT2);
    net_flow(n, KN(A380X_ELN_K_990PB2), BUS(A380X_ELN_DC2));
    /* ESS BAT */
    ess_lc = (c->bat_auto[A380X_ELN_BAT1] && c->bat_auto[A380X_ELN_BAT_ESS] && !s->bcru[2].soc_low && !any_ac) || rat_deploy;
    emer_cfg_ess = (!net_powered(n, BUS(A380X_ELN_DC1)) && s->emer_active_s > 0.0) || rat_deploy;
    net_supply(n, N_BAT1 + A380X_ELN_BAT_ESS, A380X_ELN_SRC_BAT_ESS, bat_pot[A380X_ELN_BAT_ESS]);
    bcru_state(&s->bcru[2], c->bat_auto[A380X_ELN_BAT_ESS], emer_cfg_ess);
    k_set(s, n, A380X_ELN_K_6PB3, (s->bcru[2].state == BCRU_CLOSED || ess_lc) && !f->bat[A380X_ELN_BAT_ESS]);
    net_flow(n, KN(A380X_ELN_K_6PB3), N_BAT1 + A380X_ELN_BAT_ESS);
    net_flow(n, BUS(A380X_ELN_HOT_ESS), N_BAT1 + A380X_ELN_BAT_ESS);
    net_flow(n, KN(A380X_ELN_K_6PB3), BUS(A380X_ELN_DC_ESS));
    net_flow(n, KN(A380X_ELN_K_6PB3), KN(A380X_ELN_K_6PC1));
    /* APU BAT */
    net_supply(n, N_BAT1 + A380X_ELN_BAT_APU, A380X_ELN_SRC_BAT_APU, bat_pot[A380X_ELN_BAT_APU]);
    k_set(s, n, A380X_ELN_K_5PB, c->bat_auto[A380X_ELN_BAT_APU] && !f->bat[A380X_ELN_BAT_APU] &&
          (!k_closed(s, A380X_ELN_K_7PU) || s->bat_ah[A380X_ELN_BAT_APU] <= A380X_ELN_BAT_NEEDS_CHARGE_AH));
    net_flow(n, KN(A380X_ELN_K_5PB), N_BAT1 + A380X_ELN_BAT_APU);
    net_flow(n, BUS(A380X_ELN_HOT_APU), N_BAT1 + A380X_ELN_BAT_APU);
    net_flow(n, KN(A380X_ELN_K_5PB), BUS(A380X_ELN_DC_APU));
    k_set(s, n, A380X_ELN_K_10KA_5KA, net_powered(n, BUS(A380X_ELN_DC_APU)) && s->apu_start_contactor);
    net_flow(n, BUS(A380X_ELN_DC_APU), KN(A380X_ELN_K_10KA_5KA));
    net_flow(n, KN(A380X_ELN_K_10KA_5KA), BUS(A380X_ELN_APU_START));
    /* static inverter (DC:289-301, SINV:51-58) */
    k_set(s, n, A380X_ELN_K_7XB, !net_powered(n, BUS(A380X_ELN_AC_ESS)) &&
          k_closed(s, A380X_ELN_K_6PC1) && k_closed(s, A380X_ELN_K_6PB3));
    net_flow(n, KN(A380X_ELN_K_6PC1), KN(A380X_ELN_K_7XB));
    net_flow(n, KN(A380X_ELN_K_6PB3), KN(A380X_ELN_K_7XB));
    net_flow(n, BUS(A380X_ELN_DC_ESS), KN(A380X_ELN_K_7XB));
    net_flow(n, KN(A380X_ELN_K_7XB), N_SI_IN);
    s->stat_inv_v = 0.0f; s->stat_inv_hz = 0.0f;
    if (net_potential(n, N_SI_IN) >= A380X_ELN_SI_MIN_INPUT_V && !f->stat_inv) {
        s->stat_inv_v = 115.0f; s->stat_inv_hz = 400.0f;
        net_supply(n, N_SI_OUT, A380X_ELN_SRC_STAT_INV, 115.0f);
    }
    k_set(s, n, A380X_ELN_K_970PN2, net_powered(n, BUS(A380X_ELN_DC2)));
    k_set(s, n, A380X_ELN_K_14PH, s->tefo);
    net_flow(n, BUS(A380X_ELN_DC2), KN(A380X_ELN_K_970PN2));
    net_flow(n, BUS(A380X_ELN_DC_ESS), KN(A380X_ELN_K_14PH));
    net_flow(n, KN(A380X_ELN_K_970PN2), BUS(A380X_ELN_DC_EHA));
    net_flow(n, KN(A380X_ELN_K_14PH), BUS(A380X_ELN_DC_EHA));
    k_set(s, n, A380X_ELN_K_990PX, s->bcru[1].gnd_svc_closed);
    k_set(s, n, A380X_ELN_K_970PN, net_powered(n, BUS(A380X_ELN_DC2)) && !s->bcru[1].gnd_svc_closed);
    net_flow(n, N_TR_OUT1 + A380X_ELN_TR2, KN(A380X_ELN_K_990PX));
    net_flow(n, BUS(A380X_ELN_DC2), KN(A380X_ELN_K_970PN));
    net_flow(n, KN(A380X_ELN_K_990PX), BUS(A380X_ELN_DC_GND_SVC));
    net_flow(n, KN(A380X_ELN_K_970PN), BUS(A380X_ELN_DC_GND_SVC));

    /* -- 5. AC after DC: static inverter -> AC EMER, 911XH (AC:158-176) ------------- */
    net_flow(n, N_SI_OUT, KN(A380X_ELN_K_3XB2));
    net_flow(n, KN(A380X_ELN_K_3XB2), BUS(A380X_ELN_AC_EMER));
    k_set(s, n, A380X_ELN_K_911XH, net_powered(n, BUS(A380X_ELN_DC_ESS)) && (s->tefo || !ac3));
    net_flow(n, BUS(A380X_ELN_AC_ESS), KN(A380X_ELN_K_911XH));
    net_flow(n, KN(A380X_ELN_K_911XN), BUS(A380X_ELN_AC_EHA));
    net_flow(n, KN(A380X_ELN_K_911XH), BUS(A380X_ELN_AC_EHA));

    /* -- 6. galleys (GAL:4-51). Main galley: two or fewer sources feed AC 1-4 (spec 3.11). */
    {
        uint32_t seen = 0; int sources = 0;
        int comm_off = !c->comm_on[0] || !c->comm_on[1], galy_off = !c->galley_auto || !c->pax_sys_on;
        for (i = 0; i < 4; ++i) if (net_powered(n, BUS(A380X_ELN_AC1 + i))) {
            uint32_t o = n->src[net_find(n, BUS(A380X_ELN_AC1 + i))] & ~seen;
            if (o) { seen |= o; ++sources; }
        }
        s->galley_sec_shed = !any_ac || comm_off || galy_off;
        s->galley_main_shed = s->galley_sec_shed || sources <= 2;
    }

    /* -- 7. relays: emer evac, flight condition, TEFO, EMER CONFIG (EM:176-208) ----- */
    dc_ess = net_powered(n, BUS(A380X_ELN_DC_ESS));
    emer_evac = !c->bat_auto[A380X_ELN_BAT1] && !c->bat_auto[A380X_ELN_BAT_ESS] && dc_ess;
    s->flt_condition = !in->on_ground && dc_ess && s->lsw3;
    if (emer_evac) s->tefo = s->emer_config = 0;
    {
        int any_n2 = 0;
        for (i = 0; i < 4; ++i) any_n2 |= in->eng_n2[i] > 50.0;
        if ((!any_n2 || (!dc_ess && !net_powered(n, BUS(A380X_ELN_DC2)))) && s->flt_condition) s->tefo = 1;
    }
    if (net_powered(n, BUS(A380X_ELN_HOT_ESS)) && rat_deploy) s->emer_config = 1;

    /* -- 8. 108PH sub-bus (DC:333-355) ----------------------------------------------- */
    s->sply2_timer = any_ac ? 0.0 : s->sply2_timer + dt;
    {
        int bus_ctrl = k_closed(s, A380X_ELN_K_7XB) && !s->flt_condition;
        int sply2 = s->sply2_timer < A380X_ELN_INFLT_HOLD_S;
        s->ess_inflt_closed = dc_ess && ((s->ess_inflt_closed && !bus_ctrl) || sply2 || s->emer_config ||
                                         net_powered(n, BUS(A380X_ELN_AC_ESS)));
        k_set(s, n, A380X_ELN_K_10PH, s->ess_inflt_closed);
        net_flow(n, BUS(A380X_ELN_DC_ESS), KN(A380X_ELN_K_10PH));
        net_flow(n, KN(A380X_ELN_K_10PH), BUS(A380X_ELN_DC_ESS_INFLT));
    }

    /* -- outputs ------------------------------------------------------------------- */
    s->powered = 0;
    for (i = 0; i < A380X_ELN_BUS_COUNT; ++i) {
        int on = net_powered(n, BUS(i));
        s->origin[i] = on ? n->src[net_find(n, BUS(i))] : 0;
        s->volts[i] = on ? n->pot[net_find(n, BUS(i))] : 0.0f;
        s->hz[i] = on && i <= A380X_ELN_AC_GND_SVC ? 400.0f : 0.0f;
        if (on) s->powered |= A380X_ELN_BIT(i);
    }
    for (i = 0; i < 4; ++i) if (k_closed(s, A380X_ELN_K_990XU1 + i)) s->hz[A380X_ELN_AC1 + i] = (float)s->gen_hz[i];
    s->prev_hot1 = a380x_eln_powered(s, A380X_ELN_HOT1);
    s->prev_hot_ess = a380x_eln_powered(s, A380X_ELN_HOT_ESS);
    /* generator loads (cosmetic, SD ELEC AC): ~30 kW per supplied AC bus */
    for (i = 0; i < 4; ++i) {
        int buses = 0;
        for (j = 0; j < 4; ++j) if ((s->origin[A380X_ELN_AC1 + j] >> (A380X_ELN_SRC_GEN1 + i)) & 1u) ++buses;
        s->gen_load_pct[i] = gen_avail[i] ? (float)(buses * 20) : 0.0f;
    }
    for (j = 0; j < 2; ++j) {
        int buses = 0;
        for (i = 0; i < 4; ++i) if ((s->origin[A380X_ELN_AC1 + i] >> (A380X_ELN_SRC_APU_A + j)) & 1u) ++buses;
        s->apu_gen_load_pct[j] = apu_supply[j] ? (float)(buses * 25) : 0.0f;
    }
    s->emer_gen_load_pct = k_closed(s, A380X_ELN_K_5XE) ? 45.0f : 0.0f;

    /* -- battery charge / discharge (BAT:123-160, spec 3.10) ------------------------- */
    {
        double comp_load[N_COUNT];
        int comp_bats[N_COUNT];
        memset(comp_load, 0, sizeof(comp_load)); memset(comp_bats, 0, sizeof(comp_bats));
        for (i = 0; i < A380X_ELN_BUS_COUNT; ++i) if (a380x_eln_powered(s, i)) comp_load[net_find(n, BUS(i))] += bus_load_w(i);
        if (k_closed(s, A380X_ELN_K_10KA_5KA)) comp_load[net_find(n, BUS(A380X_ELN_APU_START))] += A380X_ELN_LOAD_APU_START_W;
        if (s->stat_inv_v > 0.0f && k_closed(s, A380X_ELN_K_3XB2) && a380x_eln_powered(s, A380X_ELN_AC_EMER))
            comp_load[net_find(n, N_SI_IN)] += A380X_ELN_LOAD_AC_EMER_W;
        for (i = 0; i < 3; ++i) {      /* ADIRU on its hot-bus backup (spec 6.4 proposal) */
            int primary, backup;
            a380x_eln_adiru_supply(s, i, &primary, &backup);
            if (in->adiru_on[i] && !primary && backup) {
                static const int hot[3] = {A380X_ELN_HOT_ESS, A380X_ELN_HOT2, A380X_ELN_HOT1};
                comp_load[net_find(n, BUS(hot[i]))] += A380X_ELN_LOAD_ADIRU_BACKUP_W;
            }
        }
        for (i = 0; i < 4; ++i) if (n->conductive[N_BAT1 + i]) ++comp_bats[net_find(n, N_BAT1 + i)];
        for (i = 0; i < 4; ++i) {
            int r = net_find(n, N_BAT1 + i);
            uint32_t chargers = n->src[r] & ~((1u << A380X_ELN_SRC_BAT1) | (1u << A380X_ELN_SRC_BAT2) |
                                              (1u << A380X_ELN_SRC_BAT_ESS) | (1u << A380X_ELN_SRC_BAT_APU));
            double current = 0.0;
            if (!n->conductive[N_BAT1 + i]) { s->bat_a[i] = 0.0f; s->bat_v[i] = bat_pot[i]; continue; }
            if (chargers && n->pot[r] > bat_pot[i]) {
                current = fmin(A380X_ELN_BAT_CHARGE_MAX_A, fmax(0.0, (n->pot[r] - bat_pot[i]) / A380X_ELN_BAT_CHARGE_OHM));
            } else if (!chargers && bat_pot[i] > 0.0f && comp_bats[r] > 0) {
                current = -comp_load[r] / comp_bats[r] / bat_pot[i];
            }
            s->bat_ah[i] = fmin(A380X_ELN_BAT_CAPACITY_AH, fmax(0.0, s->bat_ah[i] + current * dt / 3600.0));
            s->bat_a[i] = (float)current;
            s->bat_v[i] = (float)fmax(a380x_eln_battery_potential(s->bat_ah[i]), chargers ? n->pot[r] : 0.0);
        }
        for (i = 0; i < A380X_ELN_TRS; ++i) {
            int r = net_find(n, N_TR_OUT1 + i);
            int on = (n->src[r] >> (A380X_ELN_SRC_TR1 + i)) & 1u;
            s->tr_v[i] = on ? (float)A380X_ELN_TR_VOLTS : 0.0f;
            s->tr_a[i] = on ? (float)(comp_load[r] / A380X_ELN_TR_VOLTS) : 0.0f;
        }
    }
}

/* ---------------------------------------------------------------- APU electronic control box */
static void apu_ecb(A380XElnState *s, const A380XElnInputs *in, double dt)
{
    int powered = a380x_eln_powered(s, A380X_ELN_DC_ESS);  /* LIB:113-118 */
    double n_pct = isfinite(in->apu_n) ? in->apu_n : 0.0;
    if (s->apu_ecb_powered && !powered && n_pct > A380X_ELN_APU_STOPPED_N && in->apu_master)
        s->apu_emergency_shutdown = 1;                      /* spec 8.2 step 5 (L) */
    s->apu_ecb_powered = powered;
    s->apu_master = in->apu_master != 0;
    if (powered) {
        double target = s->apu_master ? 1.0 : (n_pct < 7.0 ? 0.0 : s->apu_flap);
        double step = dt / A380X_ELN_APU_FLAP_S;
        s->apu_flap = s->apu_flap < target ? fmin(target, s->apu_flap + step) : fmax(target, s->apu_flap - step);
    }
    if (!s->apu_master) s->apu_start_latched = 0;
    if (n_pct > A380X_ELN_APU_AVAIL_N) s->apu_avail_timer += dt; else s->apu_avail_timer = 0.0;
    s->apu_avail = s->apu_master && powered &&
        (n_pct > A380X_ELN_APU_AVAIL_FAST_N || s->apu_avail_timer >= A380X_ELN_APU_AVAIL_HOLD_S);
    if (s->apu_avail) s->apu_start_latched = 0;
    s->apu_start_contactor = s->apu_start_latched && s->apu_master && powered && s->apu_flap >= 1.0 &&
                             n_pct < A380X_ELN_APU_STARTER_CUTOUT_N;
}

int a380x_eln_apu_start_press(A380XElnState *s)
{
    /* The START pb latches mechanically; the ECB acts on it once powered with MASTER on
     * (the latch clears on the next step if MASTER is off). */
    if (s->apu_avail) return 0;
    s->apu_start_latched = 1;
    return 1;
}

int a380x_eln_apu_starter_command(const A380XElnState *s)
{
    return a380x_eln_contactor(s, A380X_ELN_K_10KA_5KA) ? 2 : -1;
}

/* ---------------------------------------------------------------- lamps (spec 4.2) */
static void eln_lamps(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in)
{
    A380XElnLamps *l = &s->lamps;
    int dc_ess = a380x_eln_powered(s, A380X_ELN_DC_ESS), dc1 = a380x_eln_powered(s, A380X_ELN_DC1);
    int dc2 = a380x_eln_powered(s, A380X_ELN_DC2), i;
    memset(l, 0, sizeof(*l));
    for (i = 0; i < 4; ++i) {
        l->bat_fault[i] = in->fail.bat[i];
        l->gen_fault[i] = (unsigned char)(c->gen_on[i] && !a380x_eln_contactor(s, A380X_ELN_K_990XU1 + i));
        l->drive_fault[i] = in->fail.drive_oil[i];
        l->pw_gen[i] = (unsigned char)(i < 2 ? dc_ess : dc2);
        l->pw_ext_avail[i] = (unsigned char)(in->ext_connected[i] || (i < 2 ? dc1 : dc2));
        l->pw_ext_on[i] = (unsigned char)dc_ess;
        l->pw_drive[i] = (unsigned char)dc_ess;
    }
    l->pw_bat[A380X_ELN_BAT1] = l->pw_bat[A380X_ELN_BAT_ESS] = (unsigned char)dc_ess;
    l->pw_bat[A380X_ELN_BAT2] = l->pw_bat[A380X_ELN_BAT_APU] = (unsigned char)dc2;
    for (i = 0; i < 2; ++i) {
        l->apu_gen_fault[i] = (unsigned char)(s->apu_avail && c->apu_gen_on[i] && in->fail.apu_gen[i]);
        l->pw_apu_gen[i] = (unsigned char)(i == 0 ? dc_ess : dc2);
    }
    l->ac_ess_feed_fault = (unsigned char)!a380x_eln_powered(s, A380X_ELN_AC_EMER);
    l->elmu_fault = in->fail.elmu;
    l->galley_fault = in->fail.galley;
    l->rat_fault = (unsigned char)(s->emer_active_s > 0.0 && !a380x_eln_contactor(s, A380X_ELN_K_5XE) && !in->on_ground);
    l->pw_bus_tie = l->pw_ac_ess_feed = l->pw_pax_sys = l->pw_rat = (unsigned char)dc_ess;
    l->pw_galley = l->pw_elmu = l->pw_comm[1] = (unsigned char)dc2;
    l->pw_comm[0] = (unsigned char)(dc1 && dc_ess);
}

/* ---------------------------------------------------------------- step */
void a380x_eln_step(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in)
{
    /* Sub-steps of <= 50 ms; a frame is capped at 1 s of simulator time (time acceleration). */
    double remaining = in->paused || !isfinite(in->dt) ? 0.0 : fmin(1.0, fmax(0.0, in->dt));
    s->ctl = *c;
    do {
        double dt = fmin(remaining, 0.05);
        apu_ecb(s, in, dt);
        eln_evaluate(s, c, in, dt);
        remaining -= dt;
        s->time_s += dt;
    } while (remaining > 1e-9);
    apu_ecb(s, in, 0.0);
    eln_lamps(s, c, in);
}

void a380x_eln_settle(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in)
{
    A380XElnInputs settle = *in;
    double charge[4];
    int i;
    memcpy(charge, s->bat_ah, sizeof(charge));
    settle.paused = 0; settle.dt = 0.25;
    for (i = 0; i < 4; ++i) s->vfg_stable_s[i] = A380X_ELN_VFG_STABLE_S;
    for (i = 0; i < 48; ++i) a380x_eln_step(s, c, &settle);   /* 12 s: BCRU, 108PH and ties settle */
    memcpy(s->bat_ah, charge, sizeof(charge));
    for (i = 0; i < 4; ++i) s->bat_v[i] = (float)fmax(s->bat_v[i], a380x_eln_battery_potential(charge[i]));
}

/* ---------------------------------------------------------------- consumers */
int a380x_eln_du_bus_powered(const A380XElnState *s, int du)
{
#define P(b) a380x_eln_powered(s, A380X_ELN_##b)
    switch (du) {
    case A380X_ELN_DU_CAPT_PFD: return P(DC_ESS_INFLT);
    case A380X_ELN_DU_CAPT_ND: return P(DC_ESS_INFLT) || P(DC1);
    case A380X_ELN_DU_CAPT_MFD: return P(DC_ESS) || P(DC1);
    case A380X_ELN_DU_FO_PFD: return P(DC2);
    case A380X_ELN_DU_FO_ND: case A380X_ELN_DU_FO_MFD: return P(DC1) || P(DC2);
    case A380X_ELN_DU_EWD: return P(DC_ESS);
    case A380X_ELN_DU_SD: return P(DC2);
    case A380X_ELN_DU_CAPT_OIT: return P(AC2) || P(DC2);
    case A380X_ELN_DU_FO_OIT: return P(AC_EMER) || P(DC_ESS_INFLT) || P(DC1);
    default: return 0;
    }
#undef P
}

void a380x_eln_du_step(A380XElnDu *du, int powered, float brightness, int failed, double dt,
                       double boot_s, double selftest_s)
{
    int live = powered && brightness > 0.0f;
    if (!isfinite(dt) || dt < 0.0) dt = 0.0;
    if (failed) { du->state = A380X_ELN_DU_OFF; du->timer = 0.0; return; }
    switch (du->state) {
    case A380X_ELN_DU_ON:
        if (!live) { du->state = A380X_ELN_DU_STANDBY; du->timer = A380X_ELN_DU_STANDBY_S; }
        break;
    case A380X_ELN_DU_STANDBY:
        if (live) { du->state = A380X_ELN_DU_ON; du->timer = 0.0; break; }
        du->timer -= dt;
        if (du->timer <= 0.0) { du->state = A380X_ELN_DU_OFF; du->timer = 0.0; }
        break;
    case A380X_ELN_DU_BOOT:
        if (!live) { du->state = A380X_ELN_DU_OFF; du->timer = 0.0; break; }
        du->timer -= dt;
        if (du->timer <= 0.0) { du->state = A380X_ELN_DU_SELFTEST; du->timer = selftest_s; }
        break;
    case A380X_ELN_DU_SELFTEST:
        if (!live) { du->state = A380X_ELN_DU_OFF; du->timer = 0.0; break; }
        du->timer -= dt;
        if (du->timer <= 0.0) { du->state = A380X_ELN_DU_ON; du->timer = 0.0; }
        break;
    default:
        if (live) { du->state = A380X_ELN_DU_BOOT; du->timer = boot_s; }
        break;
    }
}

void a380x_eln_du_update(A380XElnState *s, const float *brightness,
                         const unsigned char *failed, double dt, double selftest_s)
{
    int i;
    if (!isfinite(selftest_s) || selftest_s < 0.0) selftest_s = A380X_ELN_DU_SELFTEST_S;
    if (selftest_s > 30.0) selftest_s = 30.0;
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) {
        double boot;
        s->rng = s->rng * 1664525u + 1013904223u;          /* deterministic per-unit boot jitter */
        boot = A380X_ELN_DU_BOOT_MIN_S + A380X_ELN_DU_BOOT_SPAN_S * (double)(s->rng >> 8) / 16777216.0;
        a380x_eln_du_step(&s->du[i], a380x_eln_du_bus_powered(s, i), brightness ? brightness[i] : 1.0f,
                          failed ? failed[i] : 0, dt, boot, selftest_s);
    }
}

void a380x_eln_du_snap(A380XElnState *s, int on)
{
    int i;
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) {
        s->du[i].timer = on ? A380X_ELN_DU_STANDBY_S : 0.0;
        s->du[i].state = !on ? A380X_ELN_DU_OFF : a380x_eln_du_bus_powered(s, i) ? A380X_ELN_DU_ON : A380X_ELN_DU_STANDBY;
    }
}

int a380x_eln_isis_powered(const A380XElnState *s, double ias_kt)
{
    return a380x_eln_powered(s, A380X_ELN_DC_ESS) || (ias_kt > 50.0 && a380x_eln_powered(s, A380X_ELN_HOT1));
}

void a380x_eln_adiru_supply(const A380XElnState *s, int unit, int *primary, int *backup)
{
    int p = 0, b = 0, dc_ess = a380x_eln_powered(s, A380X_ELN_DC_ESS);
    switch (unit) {
    case 0: p = a380x_eln_powered(s, A380X_ELN_AC_EMER); b = a380x_eln_powered(s, A380X_ELN_HOT_ESS); break;
    case 1: p = a380x_eln_powered(s, A380X_ELN_AC2); b = a380x_eln_powered(s, A380X_ELN_HOT2); break;
    case 2: p = a380x_eln_powered(s, A380X_ELN_AC1) || a380x_eln_powered(s, A380X_ELN_AC_ESS);
            b = a380x_eln_powered(s, A380X_ELN_HOT1); break;
    default: break;
    }
    /* The hot-bus backup needs the aircraft batteries connected (DC ESS live): an ADIRU
     * left in NAV does not run a cold & dark aircraft from its hot bus (assumption, L). */
    if (primary) *primary = p;
    if (backup) *backup = b && dc_ess;
}
