/* SPDX-License-Identifier: GPL-3.0-or-later
 * Zero-copy display frames (protocol v9) outside X-Plane, macOS.
 *
 *   test_xpkit_zero_copy --probe
 *       GL capability probe of a380x_xpkit_surface.h in a legacy CGL context: the success path, then
 *       the forced failure (test hook) that keeps the texture upload path.
 *   test_xpkit_zero_copy <A380XDisplayHost> <xp-kit dir> <X-Plane dir> <out prefix> [warm-up s] [measure s] [EFB frame JSON]
 *       Acts like the plugin: creates the shared memory and the 27 global surfaces exactly as
 *       a380x_xpkit_adapter.inc does, runs the REAL helper with animated PFD/ND state at 60 Hz, and
 *       measures the captured frames per view in three phases on the same warm pages:
 *         1. pixel frames (surface_mode 0, before the plugin's probe passed);
 *         2. zero-copy (surface_mode 1): every new frame must land in a surface, the front index must
 *            cycle, and the helper-written PFD/ND/SD/EFB surfaces are bound with CGLTexImageIOSurface2D,
 *            drawn and read back by GL (non-black);
 *         3. fallback (surface_mode 0 again, as after a failed probe): every view, also the static
 *            SD/EFB pages, is re-captured into the pixel arrays.
 *       Prints one JSON object (fps, front cycling, GL read-back counts) and writes
 *       <prefix>-<phase>-<view>.ppm images. Exit status 0 only if every check passed.
 * Never touches X-Plane or the aircraft (read-only use of the helper and its xp-kit assets).
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <IOSurface/IOSurface.h>
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#include "a380x_xpkit_shared.h"
#include "a380x_xpkit_surface.h"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
extern char **environ;

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void pause_s(double s) { struct timespec t = {(time_t)s, (long)((s - (time_t)s) * 1e9)}; nanosleep(&t, NULL); }

static int make_context(void) {
    CGLPixelFormatAttribute attributes[] = {kCGLPFAAccelerated, kCGLPFAAllowOfflineRenderers, kCGLPFAOpenGLProfile,
                                            (CGLPixelFormatAttribute)kCGLOGLPVersion_Legacy, 0};
    CGLPixelFormatObj format = NULL; GLint count = 0; CGLContextObj context = NULL;
    if (CGLChoosePixelFormat(attributes, &format, &count) != kCGLNoError || !format) return 0;
    CGLCreateContext(format, NULL, &context);
    CGLDestroyPixelFormat(format);
    return context && CGLSetCurrentContext(context) == kCGLNoError;
}

static int probe_test(void) {
    char why[160]; int ok, forced; GLuint names[2];
    if (!make_context()) { puts("Zero-copy probe: skipped (no CGL context)"); return 0; }
    glGenTextures(2, names);
    ok = xpkit_surface_probe(0, names, why, sizeof(why));
    if (!ok) { printf("Zero-copy probe FAILED in a legacy CGL context: %s\n", why); return 1; }
    forced = xpkit_surface_probe(1, names, why, sizeof(why));
    glDeleteTextures(2, names);
    if (forced) { puts("Zero-copy probe: forced failure was not reported"); return 1; }
    if (glGetError() != GL_NO_ERROR) { puts("Zero-copy probe left a GL error"); return 1; }
    printf("Zero-copy probe: CPU-written IOSurface sampled back exactly by GL (%s); forced failure keeps the upload path\n",
           (const char *)glGetString(GL_RENDERER));
    return 0;
}

/* ---- animated state, as tools' bench_displays.py publishes it ---- */
static int state_json(char *out, size_t cap, double t) {
    const char *A = "a380x/xp12/adirs/";
    double hdg = fmod(90 + 6 * t, 360), ra = 2400 + 300 * sin(t / 3);
    int n = snprintf(out, cap, "{\"ok\":true,\"mode\":\"native\",\"values\":{\"sim/cockpit2/electrical/bus_volts\":[28],"
        "\"sim/flightmodel/failures/onground_any\":0,\"sim/time/total_flight_time_sec\":%.6f,\"sim/time/sim_speed_actual\":1,"
        "\"sim/time/paused\":0,\"a380x/xp12/du/captain_pfd\":1,\"a380x/xp12/du/first_officer_pfd\":1,"
        "\"a380x/xp12/du/captain_nd\":1,\"a380x/xp12/du/first_officer_nd\":1,\"sim/operation/failures/rel_pil_radalt\":0,"
        "\"sim/operation/failures/rel_cop_radalt\":0,\"sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot\":%.4f,"
        "\"sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot\":%.4f", t, ra, ra);
    for (int u = 1; u <= 3 && n > 0 && (size_t)n < cap; ++u)
        n += snprintf(out + n, cap - (size_t)n,
            ",\"%sir%d_attitude_valid\":1,\"%sir%d_heading_valid\":1,\"%sir%d_nav_valid\":1,\"%sir%d_pitch_deg\":%.5f,"
            "\"%sir%d_roll_deg\":%.5f,\"%sir%d_heading_deg\":%.5f,\"%sir%d_track_deg\":%.5f,\"%sir%d_vertical_speed_fpm\":%.4f,"
            "\"%sir%d_ground_speed_knots\":%.4f,\"%sir%d_flight_path_deg\":%.5f,\"%sir%d_latitude_deg\":%.7f,\"%sir%d_longitude_deg\":-0.4614,"
            "\"%sadr%d_valid\":1,\"%sadr%d_ias_knots\":%.4f,\"%sadr%d_altitude_ft\":%.3f,\"%sadr%d_mach\":0.45,\"%sadr%d_sat_c\":-5,\"%sadr%d_tat_c\":0",
            A, u, A, u, A, u, A, u, 3 + 4 * sin(t * .9), A, u, 20 * sin(t * .5), A, u, hdg, A, u, hdg + 2, A, u, 1500 * sin(t * .4),
            A, u, 280 + 10 * sin(t), A, u, 2 * sin(t * .4), A, u, 51.4775 + .001 * t, A, u,
            A, u, A, u, 250 + 15 * sin(t * .7), A, u, 10000 + 800 * sin(t * .3), A, u, A, u, A, u);
    if (n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - (size_t)n,
            ",\"%scaptain_ir_source\":0,\"%scaptain_adr_source\":0,\"%sfirst_officer_ir_source\":1,\"%sfirst_officer_adr_source\":1},"
            "\"words\":{},\"numbers\":{},\"nav\":{\"positionValid\":true,\"lat\":%.7f,\"lon\":-0.4614,\"magVar\":0,\"tas\":270,"
            "\"groundSpeed\":280,\"zulu\":%.3f,\"windDir\":270,\"windSpeed\":10,\"controls\":[{\"mode\":3,\"range\":3},{\"mode\":3,\"range\":3}],"
            "\"route\":[]}}", A, A, A, A, 51.4775 + .001 * t, 43200 + t);
    return n > 0 && (size_t)n < cap ? n : -1;
}
static void publish_state(A380XXpKitShared *s, double t) {
    static char json[16384];
    int n = state_json(json, sizeof(json), t);
    if (n < 0) return;
    uint32_t seq = atomic_load(&s->state_seq);
    atomic_store(&s->state_seq, seq + 1);
    memcpy(s->state, json, (size_t)n + 1); s->state_size = (uint32_t)n; s->state_time = now();
    atomic_store(&s->state_seq, seq + 2);
}
static void publish_sd(A380XXpKitShared *s, double t) {
    uint32_t seq = atomic_load(&s->sd.sequence);
    atomic_store(&s->sd.sequence, seq + 1);
    s->sd.size = (uint32_t)snprintf(s->sd.json, sizeof(s->sd.json),
        "{\"page\":0,\"powered\":true,\"more\":false,\"manual\":false,\"action\":0,\"vars\":{\"sim/time/zulu_time_sec\":%d,"
        "\"sim/cockpit2/engine/indicators/N2_percent\":[%.4f,%.4f,%.4f,%.4f]},\"warnings\":[],\"fwsComplete\":false}",
        43200 + (int)t, 80 + sin(t), 80 + sin(t + 1), 80 + sin(t + 2), 80 + sin(t + 3));
    atomic_store(&s->sd.sequence, seq + 2);
}

/* ---- images ---- */
static size_t nonblack_rgba(const unsigned char *p, size_t pixels, size_t stride_px, int width, int bgra) {
    size_t count = 0; (void)bgra;
    for (size_t i = 0; i < pixels; ++i) {
        size_t x = i % (size_t)width, y = i / (size_t)width;
        const unsigned char *q = p + (y * stride_px + x) * 4;
        if (q[0] > 30 || q[1] > 30 || q[2] > 30) ++count;
    }
    return count;
}
/* rows top-down; bgra: B,G,R byte order; row_bytes stride; flip: GL bottom-up rows */
static void write_ppm(const char *name, const unsigned char *p, int width, int height, size_t row_bytes, int bgra, int flip) {
    FILE *f = fopen(name, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    unsigned char *row = malloc((size_t)width * 3);
    for (int y = 0; y < height; ++y) {
        const unsigned char *src = p + (size_t)(flip ? height - 1 - y : y) * row_bytes;
        for (int x = 0; x < width; ++x) {
            row[x * 3] = src[x * 4 + (bgra ? 2 : 0)]; row[x * 3 + 1] = src[x * 4 + 1]; row[x * 3 + 2] = src[x * 4 + (bgra ? 0 : 2)];
        }
        fwrite(row, 1, (size_t)width * 3, f);
    }
    free(row); fclose(f);
}

/* GL end-to-end: bind the helper-written front surface of a view as the plugin does, draw it into a
 * view-sized framebuffer and read it back. Returns the non-black pixel count (-1 on a GL failure). */
static long gl_readback(IOSurfaceRef surface, int width, int height, const char *ppm) {
    GLuint texture = 0, target = 0, fbo = 0; long count = -1;
    unsigned char *pixels = malloc((size_t)width * height * 4);
    glGenTextures(1, &texture); glGenTextures(1, &target);
    if (pixels && xpkit_surface_attach(texture, (XpKitSurface)surface, width, height)) {
        glBindTexture(GL_TEXTURE_2D, target);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glGenFramebuffersEXT(1, &fbo); glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, target, 0);
        if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) == GL_FRAMEBUFFER_COMPLETE_EXT) {
            glViewport(0, 0, width, height);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, width, 0, height, -1, 1);
            glMatrixMode(GL_MODELVIEW); glLoadIdentity();
            glDisable(GL_TEXTURE_2D); glDisable(GL_BLEND);
            glClearColor(0, 0, 0, 1); glClear(GL_COLOR_BUFFER_BIT);
            xpkit_surface_quad(texture, width, height, 0, 0, (float)width, (float)height);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
            if (glGetError() == GL_NO_ERROR) {
                count = (long)nonblack_rgba(pixels, (size_t)width * height, (size_t)width, width, 0);
                write_ppm(ppm, pixels, width, height, (size_t)width * 4, 0, 1);
            }
        }
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        glDeleteFramebuffersEXT(1, &fbo);
    }
    glDeleteTextures(1, &texture); glDeleteTextures(1, &target);
    free(pixels);
    return count;
}

int main(int argc, char **argv) {
    if (argc == 2 && !strcmp(argv[1], "--probe")) return probe_test();
    if (argc < 5) { fprintf(stderr, "usage: %s --probe | helper assets xplane prefix [warmup] [measure] [efb.json]\n", argv[0]); return 2; }
    const char *prefix = argv[4];
    double warmup = argc > 5 ? atof(argv[5]) : 35, measure = argc > 6 ? atof(argv[6]) : 12;
    char why[160], name[4096];
    int ok = 1;
    if (!make_context()) { puts("{\"error\":\"no CGL context\"}"); return 1; }
    GLuint probe_names[2]; glGenTextures(2, probe_names);
    int probe = xpkit_surface_probe(0, probe_names, why, sizeof(why));
    int forced = xpkit_surface_probe(1, probe_names, why, sizeof(why));
    glDeleteTextures(2, probe_names);
    char shared_path[] = "/tmp/a380x-zero-copy-XXXXXX";
    int fd = mkstemp(shared_path);
    if (fd < 0 || ftruncate(fd, sizeof(A380XXpKitShared))) return 3;
    A380XXpKitShared *s = mmap(NULL, sizeof(*s), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (s == MAP_FAILED) return 4;
    memset(s, 0, sizeof(*s)); s->magic = XPKIT_MAGIC; s->version = XPKIT_VERSION;
    /* Exactly the plugin's xpkit_start / xpkit_surfaces_create. */
    static IOSurfaceRef surface[XPKIT_VIEWS][XPKIT_SURFACES];
    for (int v = 0; v < XPKIT_VIEWS; ++v) {
        atomic_store(&s->surface_front[v], XPKIT_SURFACE_PIXELS);
        for (int i = 0; i < XPKIT_SURFACES; ++i) {
            surface[v][i] = (IOSurfaceRef)xpkit_surface_create(xpkit_view_width(v), xpkit_view_height(v));
            if (!surface[v][i]) { puts("{\"error\":\"IOSurfaceCreate\"}"); return 5; }
            atomic_store(&s->surface_id[v][i], IOSurfaceGetID(surface[v][i]));
        }
    }
    if (argc > 7) {   /* one native EFB frame for both tablets */
        FILE *f = fopen(argv[7], "rb"); if (!f) return 6;
        size_t n = fread(s->efb[0].json, 1, XPKIT_EFB_CAP - 1, f); fclose(f);
        s->efb[0].json[n] = 0; s->efb[0].size = (uint32_t)n; atomic_store(&s->efb[0].sequence, 2);
        memcpy(s->efb[1].json, s->efb[0].json, n + 1); s->efb[1].size = (uint32_t)n; atomic_store(&s->efb[1].sequence, 2);
    }
    double start = now();
    publish_state(s, 0); publish_sd(s, 0);
    pid_t pid = 0; char *args[] = {argv[1], shared_path, argv[2], argv[3], NULL};
    if (posix_spawn(&pid, argv[1], NULL, NULL, args, environ)) return 7;

    enum { PIXEL, ZERO, FALLBACK, PHASES };
    static const char *phase_name[PHASES] = {"pixels", "zero_copy", "fallback"};
    double begin[PHASES] = {warmup, warmup + measure + 2, warmup + 2 * measure + 4};
    double end[PHASES] = {warmup + measure, warmup + 2 * measure + 2, warmup + 2 * measure + 9};
    double mode_at[PHASES] = {0, warmup + measure, warmup + 2 * measure + 2};
    unsigned seq0[PHASES][XPKIT_VIEWS], seq1[PHASES][XPKIT_VIEWS];
    double t0[PHASES] = {0}, t1[PHASES] = {0};
    unsigned wrong_place[PHASES][XPKIT_VIEWS], front_changes[XPKIT_VIEWS], fronts_seen[XPKIT_VIEWS];
    unsigned last_seq[XPKIT_VIEWS], last_front[XPKIT_VIEWS];
    memset(wrong_place, 0, sizeof(wrong_place)); memset(front_changes, 0, sizeof(front_changes));
    memset(fronts_seen, 0, sizeof(fronts_seen)); memset(seq0, 0, sizeof(seq0)); memset(seq1, 0, sizeof(seq1));
    for (int v = 0; v < XPKIT_VIEWS; ++v) { last_seq[v] = 0; last_front[v] = XPKIT_SURFACE_PIXELS; }
    int phase = -1, exited = 0;
    double next = start, last_sd = 0, total = end[FALLBACK] + .5;
    while (now() - start < total) {
        double t = now() - start;
        for (int p = 0; p < PHASES; ++p) {
            if (t >= mode_at[p] && phase < p && (p == PIXEL || t >= mode_at[p])) {
                phase = p;
                uint32_t mode = p == ZERO ? 1 : 0;
                if (p == ZERO && !probe) mode = 0;   /* as the plugin: zero-copy only after the probe passed */
                for (int v = 0; v < XPKIT_VIEWS; ++v) atomic_store(&s->surface_mode[v], mode);
            }
            if (!t0[p] && t >= begin[p]) { t0[p] = now(); for (int v = 0; v < XPKIT_VIEWS; ++v) seq0[p][v] = atomic_load(&s->frame_seq[v]); }
            if (!t1[p] && t >= end[p]) { t1[p] = now(); for (int v = 0; v < XPKIT_VIEWS; ++v) seq1[p][v] = atomic_load(&s->frame_seq[v]); }
        }
        publish_state(s, t);
        if (t - last_sd >= .25) { publish_sd(s, t); last_sd = t; }
        /* Where does each new frame land? (pixels in phases 1/3, a surface in phase 2) */
        for (int v = 0; v < XPKIT_VIEWS; ++v) {
            unsigned seq = atomic_load(&s->frame_seq[v]);
            if ((seq & 1) || seq == last_seq[v]) continue;
            unsigned front = atomic_load(&s->surface_front[v]);
            if (atomic_load(&s->frame_seq[v]) != seq) continue;
            last_seq[v] = seq;
            int in_window = phase >= 0 && t0[phase] && !t1[phase];
            if (in_window && ((phase == ZERO) != (front < XPKIT_SURFACES))) ++wrong_place[phase][v];
            if (front < XPKIT_SURFACES) { fronts_seen[v] |= 1u << front; if (front != last_front[v]) ++front_changes[v]; }
            last_front[v] = front;
        }
        if (waitpid(pid, NULL, WNOHANG) == pid) { exited = 1; break; }
        /* GL end-to-end just before zero-copy ends: helper-written surfaces through CGLTexImageIOSurface2D */
        static int read_back; static long gl_nonblack[XPKIT_VIEWS];
        if (phase == ZERO && !read_back && t >= end[ZERO] - 1) {
            read_back = 1;
            for (int v = 0; v < XPKIT_VIEWS; ++v) {
                gl_nonblack[v] = -2;
                unsigned seq = atomic_load(&s->frame_seq[v]), front = atomic_load(&s->surface_front[v]);
                if ((seq & 1) || front >= XPKIT_SURFACES || atomic_load(&s->frame_seq[v]) != seq) continue;
                snprintf(name, sizeof(name), "%s-gl-%d.ppm", prefix, v);
                gl_nonblack[v] = gl_readback(surface[v][front], xpkit_view_width(v), xpkit_view_height(v), name);
                /* the same surface as the CPU sees it */
                IOSurfaceRef f = surface[v][front];
                IOSurfaceLock(f, kIOSurfaceLockReadOnly, NULL);
                snprintf(name, sizeof(name), "%s-zero_copy-%d.ppm", prefix, v);
                write_ppm(name, IOSurfaceGetBaseAddress(f), xpkit_view_width(v), xpkit_view_height(v), IOSurfaceGetBytesPerRow(f), 1, 0);
                IOSurfaceUnlock(f, kIOSurfaceLockReadOnly, NULL);
            }
            printf("{\"gl_readback_nonblack\":[");
            for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%ld", v ? "," : "", gl_nonblack[v]);
            printf("]}\n");
            fflush(stdout);
            if (probe && (gl_nonblack[0] < 2000 || gl_nonblack[2] < 2000 || gl_nonblack[6] < 2000)) ok = 0;
            if (probe && argc > 7 && gl_nonblack[7] < 2000) ok = 0;
        }
        next += 1.0 / 60.0;
        double wait = next - now();
        if (wait > 0) pause_s(wait);
    }
    /* Final pixel frames (fallback phase): the pixel arrays of every captured view. */
    size_t pixel_nonblack[XPKIT_VIEWS];
    for (int v = 0; v < XPKIT_VIEWS; ++v) {
        unsigned char *p = xpkit_view_pixels(s, v);
        pixel_nonblack[v] = nonblack_rgba(p, (size_t)xpkit_view_width(v) * xpkit_view_height(v), (size_t)xpkit_view_width(v), xpkit_view_width(v), 0);
        snprintf(name, sizeof(name), "%s-fallback-%d.ppm", prefix, v);
        write_ppm(name, p, xpkit_view_width(v), xpkit_view_height(v), (size_t)xpkit_view_width(v) * 4, 0, 0);
    }
    unsigned errors = atomic_load(&s->errors);
    char error_text[sizeof(s->error) + 1]; memcpy(error_text, s->error, sizeof(s->error)); error_text[sizeof(s->error)] = 0;
    for (char *c = error_text; *c; ++c) if (*c == '"' || *c == '\\' || (unsigned char)*c < 32) *c = ' ';
    kill(pid, SIGTERM); waitpid(pid, NULL, 0);
    static const char *names[XPKIT_VIEWS] = {"PFD capt", "PFD f/o", "ND capt", "ND f/o", "MFD capt", "MFD f/o", "SD", "EFB capt", "EFB f/o"};
    printf("{\"probe\":%d,\"forced_probe\":%d,\"helper_exited\":%d,\"errors\":%u,\"error\":\"%s\",\"pfd\":[%d,%d],\"nd\":[%d,%d],\"phases\":{",
           probe, forced, exited, errors, error_text, xpkit_view_width(0), xpkit_view_height(0), xpkit_view_width(2), xpkit_view_height(2));
    for (int p = 0; p < PHASES; ++p) {
        double dt = t1[p] - t0[p];
        printf("%s\"%s\":{\"seconds\":%.2f,\"fps\":{", p ? "," : "", phase_name[p], dt);
        for (int v = 0; v < XPKIT_VIEWS; ++v)
            printf("%s\"%s\":%.1f", v ? "," : "", names[v], dt > 0 ? (seq1[p][v] - seq0[p][v]) / 2.0 / dt : 0);
        printf("},\"frames_in_wrong_place\":[");
        for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%u", v ? "," : "", wrong_place[p][v]);
        printf("]}");
    }
    printf("},\"front_changes\":[");
    for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%u", v ? "," : "", front_changes[v]);
    printf("],\"fronts_seen_mask\":[");
    for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%u", v ? "," : "", fronts_seen[v]);
    printf("],\"final_front\":[");
    for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%u", v ? "," : "", atomic_load(&s->surface_front[v]));
    printf("],\"pixel_nonblack_after_fallback\":[");
    for (int v = 0; v < XPKIT_VIEWS; ++v) printf("%s%zu", v ? "," : "", pixel_nonblack[v]);
    printf("]}\n");

    if (!probe || forced || exited || errors) ok = 0;
    for (int p = 0; p < PHASES; ++p) {
        if (!(t1[p] > t0[p])) ok = 0;
        for (int v = 0; v < 4; ++v) if (seq1[p][v] <= seq0[p][v] || wrong_place[p][v]) ok = 0;   /* PFD/ND advance, right place */
        if (wrong_place[p][6]) ok = 0;
    }
    for (int v = 0; v < 4; ++v) if (fronts_seen[v] != 7 || front_changes[v] < 3) ok = 0;            /* all three surfaces cycle */
    if (!fronts_seen[6]) ok = 0;                                                                     /* SD landed in a surface */
    for (int v = 0; v < 4; ++v) if (pixel_nonblack[v] < 2000) ok = 0;
    if (pixel_nonblack[6] < 2000) ok = 0;
    if (argc > 7 && (pixel_nonblack[7] < 2000 || !fronts_seen[7])) ok = 0;
    /* Both switches re-captured every captured view, also a static page (EFB): its last frame is in
     * the pixel arrays again after the fallback and was in a surface during zero-copy. */
    for (int v = 0; v < XPKIT_VIEWS; ++v) {
        if (v == 4 || v == 5 || (v >= XPKIT_EFB_VIEW0 && argc <= 7)) continue;   /* no MFD fixture / EFB frame */
        if (atomic_load(&s->surface_front[v]) != XPKIT_SURFACE_PIXELS) ok = 0;
    }
    for (int v = 0; v < XPKIT_VIEWS; ++v) for (int i = 0; i < XPKIT_SURFACES; ++i) CFRelease(surface[v][i]);
    munmap(s, sizeof(*s)); unlink(shared_path);
    printf("%s\n", ok ? "Zero-copy display frames: PASSED" : "Zero-copy display frames: FAILED");
    return ok ? 0 : 1;
}
