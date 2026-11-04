/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * png2dds: convert one 8-bit PNG texture to an X-Plane DDS (DXT1 when fully opaque, else DXT5)
 * with a full mipmap chain, so X-Plane loads it pre-compressed instead of decoding the PNG and
 * keeping it as RGBA (4-8x less memory, no load-time decoding or mipmap generation).
 *
 * X-Plane uses NAME.dds in place of a referenced NAME.png when it exists (Laminar's own aircraft
 * reference .png names and ship only .dds). Rows are stored top-down like the PNG: Laminar's DDS
 * (Sikorsky S-76 panels, Cessna 172 audio panel) decode upright with text reading normally.
 *
 *   png2dds [--alpha-test CUTOFF] in.png out.dds   convert
 *   png2dds --verify in.png out.dds                decode out.dds level 0, print the PSNR against in.png
 *
 * Transparency: the colour of fully transparent texels is filled from the nearest visible texels
 * (pull-push) and mip levels average colour weighted by alpha. The FBW decal exports store black
 * under transparent texels; filtering and mipmapping mixed that black into every visible edge and
 * drew a dark, smeared outline around white panel lettering. With --alpha-test (the OBJ's
 * ATTR_no_blend cutoff), each mip level's alpha is scaled so the share of texels above the cutoff
 * stays that of level 0: lettering keeps its weight at a distance instead of thinning or bloating.
 * --verify weights the colour error by alpha (colour under transparent texels is never seen).
 *
 * Exit status: 0 ok, 2 usage, 3 unsupported or invalid PNG, 4 I/O error, 5 verification mismatch.
 * Only non-interlaced 8-bit grey, grey+alpha, RGB, RGBA and palette PNGs are supported (every
 * texture of the aircraft). Dimensions must be powers of two.
 *
 * Encoder: per 4x4 block, endpoints on the principal axis of the colours (power iteration), inset,
 * then two least-squares refinements of the endpoints for the chosen indices, keeping the
 * lowest-error result. Alpha (DXT5) uses the 8-value mode between the block's min and max.
 * Mipmaps: 2x2 box filter (colour weighted by alpha), down to 1x1.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

/* ------------------------------------------------------------------ PNG decode */

typedef struct { int w, h; uint8_t *rgba; } Image;

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

static uint8_t *read_file(const char *path, size_t *size)
{
    FILE *f = fopen(path, "rb");
    uint8_t *data;
    long n;
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) || (n = ftell(f)) < 0 || fseek(f, 0, SEEK_SET)) { fclose(f); return NULL; }
    data = malloc((size_t)n ? (size_t)n : 1);
    if (data && fread(data, 1, (size_t)n, f) != (size_t)n) { free(data); data = NULL; }
    fclose(f);
    *size = (size_t)n;
    return data;
}

static int paeth(int a, int b, int c)
{
    int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
    return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
}

/* Returns 0 on success; *why names the problem otherwise. */
static int png_decode(const char *path, Image *img, const char **why)
{
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    size_t size, pos = 8, idat_len = 0, row_bytes, raw_len;
    uint8_t *file = read_file(path, &size), *idat = NULL, *raw = NULL, palette[256][4];
    int w = 0, h = 0, depth = 0, type = -1, interlace = 0, channels, have_plte = 0, x, y, rc = 3;
    memset(palette, 255, sizeof(palette));
    *why = "unreadable file";
    if (!file) return 4;
    if (size < 33 || memcmp(file, sig, 8)) { *why = "not a PNG"; goto done; }
    while (pos + 12 <= size) {
        uint32_t len = be32(file + pos);
        const uint8_t *tag = file + pos + 4, *body = file + pos + 8;
        if (pos + 12 + (size_t)len > size) { *why = "truncated chunk"; goto done; }
        if (!memcmp(tag, "IHDR", 4) && len >= 13) {
            w = (int)be32(body); h = (int)be32(body + 4); depth = body[8]; type = body[9]; interlace = body[12];
        } else if (!memcmp(tag, "PLTE", 4)) {
            for (uint32_t i = 0; i < len / 3 && i < 256; ++i) { palette[i][0] = body[3*i]; palette[i][1] = body[3*i+1]; palette[i][2] = body[3*i+2]; }
            have_plte = 1;
        } else if (!memcmp(tag, "tRNS", 4) && type == 3) {
            for (uint32_t i = 0; i < len && i < 256; ++i) palette[i][3] = body[i];
        } else if (!memcmp(tag, "IDAT", 4)) {
            uint8_t *grown = realloc(idat, idat_len + len);
            if (!grown) { *why = "out of memory"; goto done; }
            idat = grown; memcpy(idat + idat_len, body, len); idat_len += len;
        } else if (!memcmp(tag, "IEND", 4)) break;
        pos += 12 + (size_t)len;
    }
    if (w <= 0 || h <= 0 || w > 16384 || h > 16384) { *why = "bad dimensions"; goto done; }
    if (depth != 8) { *why = "only 8-bit PNGs are supported"; goto done; }
    if (interlace) { *why = "interlaced PNGs are not supported"; goto done; }
    channels = type == 0 ? 1 : type == 2 ? 3 : type == 3 ? 1 : type == 4 ? 2 : type == 6 ? 4 : 0;
    if (!channels || (type == 3 && !have_plte)) { *why = "unsupported PNG colour type"; goto done; }
    row_bytes = (size_t)w * (size_t)channels;
    raw_len = (row_bytes + 1) * (size_t)h;
    raw = malloc(raw_len);
    if (!raw) { *why = "out of memory"; goto done; }
    {
        z_stream z;
        memset(&z, 0, sizeof(z));
        if (inflateInit(&z) != Z_OK) { *why = "zlib init failed"; goto done; }
        z.next_in = idat; z.avail_in = (uInt)idat_len; z.next_out = raw; z.avail_out = (uInt)raw_len;
        int zr = inflate(&z, Z_FINISH);
        inflateEnd(&z);
        if ((zr != Z_STREAM_END && zr != Z_OK) || z.total_out != raw_len) { *why = "corrupt image data"; goto done; }
    }
    /* Unfilter in place (bytes per pixel = channels for 8-bit). */
    for (y = 0; y < h; ++y) {
        uint8_t *line = raw + (size_t)y * (row_bytes + 1), filter = line[0], *cur = line + 1;
        const uint8_t *prev = y ? raw + (size_t)(y - 1) * (row_bytes + 1) + 1 : NULL;
        for (size_t i = 0; i < row_bytes; ++i) {
            int a = i >= (size_t)channels ? cur[i - (size_t)channels] : 0, b = prev ? prev[i] : 0;
            int c = prev && i >= (size_t)channels ? prev[i - (size_t)channels] : 0, v = cur[i];
            switch (filter) {
            case 0: break;
            case 1: v += a; break;
            case 2: v += b; break;
            case 3: v += (a + b) / 2; break;
            case 4: v += paeth(a, b, c); break;
            default: *why = "bad PNG filter"; goto done;
            }
            cur[i] = (uint8_t)v;
        }
    }
    img->w = w; img->h = h;
    img->rgba = malloc((size_t)w * (size_t)h * 4);
    if (!img->rgba) { *why = "out of memory"; goto done; }
    for (y = 0; y < h; ++y) {
        const uint8_t *s = raw + (size_t)y * (row_bytes + 1) + 1;
        uint8_t *d = img->rgba + (size_t)y * (size_t)w * 4;
        for (x = 0; x < w; ++x, d += 4) {
            switch (type) {
            case 0: d[0] = d[1] = d[2] = s[x]; d[3] = 255; break;
            case 2: d[0] = s[3*x]; d[1] = s[3*x+1]; d[2] = s[3*x+2]; d[3] = 255; break;
            case 3: memcpy(d, palette[s[x]], 4); break;
            case 4: d[0] = d[1] = d[2] = s[2*x]; d[3] = s[2*x+1]; break;
            default: memcpy(d, s + 4*x, 4); break;
            }
        }
    }
    rc = 0;
done:
    free(file); free(idat); free(raw);
    return rc;
}

/* ------------------------------------------------------------------ BC1 / BC3 */

static uint16_t pack565(const float c[3])
{
    int r = (int)lrintf(fminf(fmaxf(c[0], 0.0f), 255.0f) * 31.0f / 255.0f);
    int g = (int)lrintf(fminf(fmaxf(c[1], 0.0f), 255.0f) * 63.0f / 255.0f);
    int b = (int)lrintf(fminf(fmaxf(c[2], 0.0f), 255.0f) * 31.0f / 255.0f);
    return (uint16_t)(r << 11 | g << 5 | b);
}
static void unpack565(uint16_t v, int c[3])
{
    int r = v >> 11 & 31, g = v >> 5 & 63, b = v & 31;
    c[0] = r << 3 | r >> 2; c[1] = g << 2 | g >> 4; c[2] = b << 3 | b >> 2;
}
/* 4-colour palette exactly as decoders build it. */
static void palette4(uint16_t c0, uint16_t c1, int pal[4][3])
{
    unpack565(c0, pal[0]); unpack565(c1, pal[1]);
    for (int k = 0; k < 3; ++k) {
        pal[2][k] = (2 * pal[0][k] + pal[1][k]) / 3;
        pal[3][k] = (pal[0][k] + 2 * pal[1][k]) / 3;
    }
}
/* Indices for c0 > c1 (4-colour mode); returns the squared error. */
static long fit_indices(const uint8_t px[16][4], uint16_t c0, uint16_t c1, uint32_t *bits, int idx[16])
{
    int pal[4][3];
    long total = 0;
    palette4(c0, c1, pal);
    *bits = 0;
    for (int i = 0; i < 16; ++i) {
        long best = -1; int bi = 0;
        for (int k = 0; k < 4; ++k) {
            long e = 0;
            for (int c = 0; c < 3; ++c) { long d = (long)px[i][c] - pal[k][c]; e += d * d; }
            if (best < 0 || e < best) { best = e; bi = k; }
        }
        idx[i] = bi; total += best; *bits |= (uint32_t)bi << (2 * i);
    }
    return total;
}

static void encode_color(const uint8_t px[16][4], uint8_t out[8])
{
    float mean[3] = {0, 0, 0}, cov[6] = {0, 0, 0, 0, 0, 0}, axis[3], lo = 1e9f, hi = -1e9f;
    int i, k, it, idx[16];
    uint16_t best0 = 0, best1 = 0;
    uint32_t best_bits = 0;
    long best_err = -1;
    for (i = 0; i < 16; ++i) for (k = 0; k < 3; ++k) mean[k] += px[i][k] / 16.0f;
    for (i = 0; i < 16; ++i) {
        float d[3] = {px[i][0] - mean[0], px[i][1] - mean[1], px[i][2] - mean[2]};
        cov[0] += d[0]*d[0]; cov[1] += d[0]*d[1]; cov[2] += d[0]*d[2];
        cov[3] += d[1]*d[1]; cov[4] += d[1]*d[2]; cov[5] += d[2]*d[2];
    }
    /* Principal axis by power iteration, starting along the widest channel. */
    axis[0] = cov[0]; axis[1] = cov[3]; axis[2] = cov[5];
    {
        int m = cov[3] > cov[0] ? (cov[5] > cov[3] ? 2 : 1) : (cov[5] > cov[0] ? 2 : 0);
        axis[0] = axis[1] = axis[2] = 0.0f; axis[m] = 1.0f;
    }
    for (it = 0; it < 8; ++it) {
        float v[3] = {cov[0]*axis[0] + cov[1]*axis[1] + cov[2]*axis[2],
                      cov[1]*axis[0] + cov[3]*axis[1] + cov[4]*axis[2],
                      cov[2]*axis[0] + cov[4]*axis[1] + cov[5]*axis[2]};
        float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n < 1e-6f) break;
        axis[0] = v[0] / n; axis[1] = v[1] / n; axis[2] = v[2] / n;
    }
    for (i = 0; i < 16; ++i) {
        float t = (px[i][0] - mean[0]) * axis[0] + (px[i][1] - mean[1]) * axis[1] + (px[i][2] - mean[2]) * axis[2];
        if (t < lo) lo = t;
        if (t > hi) hi = t;
    }
    {
        float inset = (hi - lo) / 16.0f, e0[3], e1[3];
        for (k = 0; k < 3; ++k) { e0[k] = mean[k] + axis[k] * (hi - inset); e1[k] = mean[k] + axis[k] * (lo + inset); }
        for (it = 0; it < 3; ++it) {
            uint16_t c0 = pack565(e0), c1 = pack565(e1);
            uint32_t bits;
            long err;
            if (c0 < c1) { uint16_t t = c0; c0 = c1; c1 = t; }
            if (c0 == c1) {                      /* flat after quantisation: all index 0 */
                int pal[3]; long e = 0;
                unpack565(c0, pal);
                for (i = 0; i < 16; ++i) for (k = 0; k < 3; ++k) { long d = (long)px[i][k] - pal[k]; e += d * d; }
                bits = 0; err = e;
                for (i = 0; i < 16; ++i) idx[i] = 0;
            } else {
                err = fit_indices(px, c0, c1, &bits, idx);
            }
            if (best_err < 0 || err < best_err) { best_err = err; best0 = c0; best1 = c1; best_bits = bits; }
            if (c0 == c1 || err == 0) break;
            /* Least-squares endpoints for these indices (weights of colour 0 per index). */
            {
                static const float wa[4] = {1.0f, 0.0f, 2.0f / 3.0f, 1.0f / 3.0f};
                float aa = 0, bb = 0, ab = 0, ax[3] = {0, 0, 0}, bx[3] = {0, 0, 0}, det;
                for (i = 0; i < 16; ++i) {
                    float a = wa[idx[i]], b = 1.0f - a;
                    aa += a*a; bb += b*b; ab += a*b;
                    for (k = 0; k < 3; ++k) { ax[k] += a * px[i][k]; bx[k] += b * px[i][k]; }
                }
                det = aa * bb - ab * ab;
                if (fabsf(det) < 1e-6f) break;
                for (k = 0; k < 3; ++k) {
                    e0[k] = (ax[k] * bb - bx[k] * ab) / det;
                    e1[k] = (bx[k] * aa - ax[k] * ab) / det;
                }
            }
        }
    }
    out[0] = (uint8_t)best0; out[1] = (uint8_t)(best0 >> 8);
    out[2] = (uint8_t)best1; out[3] = (uint8_t)(best1 >> 8);
    out[4] = (uint8_t)best_bits; out[5] = (uint8_t)(best_bits >> 8);
    out[6] = (uint8_t)(best_bits >> 16); out[7] = (uint8_t)(best_bits >> 24);
}

static void encode_alpha(const uint8_t px[16][4], uint8_t out[8])
{
    int lo = 255, hi = 0, i, k, pal[8];
    uint64_t bits = 0;
    for (i = 0; i < 16; ++i) { if (px[i][3] < lo) lo = px[i][3]; if (px[i][3] > hi) hi = px[i][3]; }
    out[0] = (uint8_t)hi; out[1] = (uint8_t)lo;
    pal[0] = hi; pal[1] = lo;
    for (k = 1; k < 7; ++k) pal[k + 1] = ((7 - k) * hi + k * lo) / 7;
    for (i = 0; i < 16; ++i) {
        int best = 0, be = 1 << 30;
        if (hi != lo)
            for (k = 0; k < 8; ++k) { int e = abs(px[i][3] - pal[k]); if (e < be) { be = e; best = k; } }
        bits |= (uint64_t)best << (3 * i);
    }
    for (i = 0; i < 6; ++i) out[2 + i] = (uint8_t)(bits >> (8 * i));
}

static void load_block(const uint8_t *rgba, int w, int h, int bx, int by, uint8_t px[16][4])
{
    for (int i = 0; i < 16; ++i) {
        int x = bx * 4 + i % 4, y = by * 4 + i / 4;
        if (x >= w) x = w - 1;                   /* levels smaller than 4x4 repeat the edge */
        if (y >= h) y = h - 1;
        memcpy(px[i], rgba + ((size_t)y * (size_t)w + (size_t)x) * 4, 4);
    }
}

static size_t level_bytes(int w, int h, int block) { return (size_t)((w + 3) / 4) * (size_t)((h + 3) / 4) * (size_t)block; }

static void encode_level(const uint8_t *rgba, int w, int h, int dxt5, uint8_t *out)
{
    uint8_t px[16][4];
    for (int by = 0; by < (h + 3) / 4; ++by)
        for (int bx = 0; bx < (w + 3) / 4; ++bx) {
            load_block(rgba, w, h, bx, by, px);
            if (dxt5) {
                /* Endpoints for the visible texels only: a transparent texel in a mixed block takes
                 * the mean of the block's visible colours (the fill keeps fully clear blocks smooth). */
                int sum[3] = {0, 0, 0}, visible = 0, clear = 0;
                for (int i = 0; i < 16; ++i) {
                    if (px[i][3]) { for (int c = 0; c < 3; ++c) sum[c] += px[i][c]; ++visible; } else ++clear;
                }
                if (visible && clear)
                    for (int i = 0; i < 16; ++i)
                        if (!px[i][3]) for (int c = 0; c < 3; ++c) px[i][c] = (uint8_t)((sum[c] + visible / 2) / visible);
                encode_alpha(px, out); out += 8;
            }
            encode_color(px, out); out += 8;
        }
}

/* 2x2 box filter (POT sizes: each dimension halves, minimum 1); colour weighted by alpha so a
 * transparent texel's colour never darkens or tints its visible neighbours. */
static uint8_t *downsample(const uint8_t *src, int w, int h, int *nw, int *nh)
{
    int ww = w > 1 ? w / 2 : 1, hh = h > 1 ? h / 2 : 1;
    uint8_t *dst = malloc((size_t)ww * (size_t)hh * 4);
    if (!dst) return NULL;
    for (int y = 0; y < hh; ++y)
        for (int x = 0; x < ww; ++x) {
            int x0 = w > 1 ? 2 * x : 0, x1 = w > 1 ? 2 * x + 1 : 0, y0 = h > 1 ? 2 * y : 0, y1 = h > 1 ? 2 * y + 1 : 0;
            const uint8_t *q[4] = {src + ((size_t)y0 * w + x0) * 4, src + ((size_t)y0 * w + x1) * 4,
                                   src + ((size_t)y1 * w + x0) * 4, src + ((size_t)y1 * w + x1) * 4};
            uint8_t *o = dst + ((size_t)y * ww + x) * 4;
            int sa = q[0][3] + q[1][3] + q[2][3] + q[3][3];
            for (int c = 0; c < 3; ++c) {
                int s = 0, sw = 0;
                for (int k = 0; k < 4; ++k) { s += q[k][c] * q[k][3]; sw += q[k][c]; }
                o[c] = (uint8_t)(sa ? (s + sa / 2) / sa : (sw + 2) / 4);
            }
            o[3] = (uint8_t)((sa + 2) / 4);
        }
    *nw = ww; *nh = hh;
    return dst;
}

/* Textures whose semi-transparent texels are mostly bright straight-alpha colour (>= 80%), like the
 * FBW panel lettering: their darker edge texels (premultiplied or partly darkened by an earlier
 * resample) are export artefacts that draw a dark rim around every glyph. The FCU label shadows and
 * the dark MIP03 panel are dark by design and fail this test, so they keep their edge colours. */
static int bright_straight_alpha(const Image *img)
{
    size_t n = (size_t)img->w * (size_t)img->h, semi = 0, bright = 0;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t *p = img->rgba + i * 4;
        if (p[3] == 0 || p[3] >= 250) continue;
        ++semi;
        if (p[0] > 200 || p[1] > 200 || p[2] > 200) ++bright;
    }
    return semi && bright * 5 >= semi * 4;
}

static float luma(const float c[3]) { return 0.299f * c[0] + 0.587f * c[1] + 0.114f * c[2]; }

/* Pull-push fill: every texel gets the colour of the nearest source texels (alpha > 0; with the
 * edge repair, the glyph cores at alpha >= 200). Fully transparent texels take that colour; with
 * the repair, so do edge texels (alpha < 200) more than 15% darker than it. Alpha never changes.
 * Returns 0, or 4 when out of memory. */
static int bleed_transparent(Image *img)
{
    int w = img->w, h = img->h, levels = 1, any_clear = 0, any_source = 0;
    size_t n = (size_t)w * (size_t)h;
    int repair = bright_straight_alpha(img), core = repair ? 200 : 1;
    for (size_t i = 0; i < n; ++i) {
        int a = img->rgba[i * 4 + 3];
        if (a >= core) any_source = 1;
        if (a == 0 || (repair && a < core)) any_clear = 1;
    }
    if (!any_clear || !any_source) return 0;
    while ((w >> (levels - 1)) > 1 || (h >> (levels - 1)) > 1) ++levels;
    /* Level l >= 1: normalised colour (r, g, b) and coverage weight per texel. */
    float **lv = calloc((size_t)levels, sizeof *lv);
    int *lw = calloc((size_t)levels, sizeof *lw), *lh = calloc((size_t)levels, sizeof *lh), rc = 4;
    if (!lv || !lw || !lh) goto done;
    lw[0] = w; lh[0] = h;
    for (int l = 1; l < levels; ++l) {
        int pw = lw[l - 1], ph = lh[l - 1], cw = pw > 1 ? pw / 2 : 1, ch = ph > 1 ? ph / 2 : 1;
        lw[l] = cw; lh[l] = ch;
        lv[l] = malloc((size_t)cw * (size_t)ch * 4 * sizeof(float));
        if (!lv[l]) goto done;
        for (int y = 0; y < ch; ++y)
            for (int x = 0; x < cw; ++x) {
                float s[3] = {0, 0, 0}, sw = 0;
                for (int k = 0; k < 4; ++k) {
                    int sx = pw > 1 ? 2 * x + (k & 1) : 0, sy = ph > 1 ? 2 * y + (k >> 1) : 0;
                    float wt, c[3];
                    if (l == 1) {
                        const uint8_t *p = img->rgba + ((size_t)sy * pw + sx) * 4;
                        wt = p[3] >= core ? 1.0f : 0.0f; c[0] = p[0]; c[1] = p[1]; c[2] = p[2];
                    } else {
                        const float *p = lv[l - 1] + ((size_t)sy * pw + sx) * 4;
                        wt = p[3]; c[0] = p[0]; c[1] = p[1]; c[2] = p[2];
                    }
                    for (int j = 0; j < 3; ++j) s[j] += c[j] * wt;
                    sw += wt;
                }
                float *o = lv[l] + ((size_t)y * cw + x) * 4;
                for (int j = 0; j < 3; ++j) o[j] = sw > 0 ? s[j] / sw : 0;
                o[3] = sw;
            }
    }
    /* Push: a texel with no coverage takes its parent's (already filled) colour. */
    for (int l = levels - 2; l >= 1; --l)
        for (int y = 0; y < lh[l]; ++y)
            for (int x = 0; x < lw[l]; ++x) {
                float *o = lv[l] + ((size_t)y * lw[l] + x) * 4;
                if (o[3] > 0) continue;
                const float *p = lv[l + 1] + ((size_t)(lh[l + 1] > 1 ? y / 2 : 0) * lw[l + 1] + (lw[l + 1] > 1 ? x / 2 : 0)) * 4;
                o[0] = p[0]; o[1] = p[1]; o[2] = p[2];
            }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t *o = img->rgba + ((size_t)y * w + x) * 4;
            if (o[3] >= core) continue;
            const float *p = lv[1] + ((size_t)(lh[1] > 1 ? y / 2 : 0) * lw[1] + (lw[1] > 1 ? x / 2 : 0)) * 4;
            if (o[3]) {                                   /* repair only: a darkened edge texel */
                float c[3] = {o[0], o[1], o[2]};
                if (luma(c) >= 0.85f * luma(p)) continue;
            }
            for (int j = 0; j < 3; ++j) { float v = p[j] + 0.5f; o[j] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v); }
        }
    rc = 0;
done:
    if (lv) for (int l = 0; l < levels; ++l) free(lv[l]);
    free(lv); free(lw); free(lh);
    return rc;
}

/* Share of texels whose alpha exceeds the cutoff (0..255 scale). */
static double coverage(const uint8_t *rgba, size_t n, double cutoff, double scale)
{
    size_t above = 0;
    for (size_t i = 0; i < n; ++i) if (rgba[i * 4 + 3] * scale > cutoff) ++above;
    return n ? (double)above / (double)n : 0.0;
}

/* Alpha scale for a mip level that restores the level-0 coverage at the alpha-test cutoff. */
static double coverage_scale(const uint8_t *rgba, size_t n, double cutoff, double target)
{
    double lo = 0.0, hi = 8.0;
    for (int i = 0; i < 24; ++i) {
        double mid = 0.5 * (lo + hi);
        if (coverage(rgba, n, cutoff, mid) < target) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

/* ------------------------------------------------------------------ DDS */

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static int write_dds(const char *path, const Image *img, int dxt5, double alpha_test, int *levels_out)
{
    int block = dxt5 ? 16 : 8, levels = 1, w = img->w, h = img->h;
    uint8_t header[128];
    const uint8_t *level = img->rgba;
    uint8_t *owned = NULL, *scaled = NULL;
    double cutoff = alpha_test * 255.0, target = 0.0;
    char tmp[4096];
    FILE *f;
    if (dxt5 && alpha_test > 0.0) target = coverage(img->rgba, (size_t)img->w * (size_t)img->h, cutoff, 1.0);
    while ((w >> (levels - 1)) > 1 || (h >> (levels - 1)) > 1) ++levels;
    memset(header, 0, sizeof(header));
    memcpy(header, "DDS ", 4);
    put32(header + 4, 124);
    put32(header + 8, 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000); /* CAPS HEIGHT WIDTH PIXELFORMAT MIPMAPCOUNT LINEARSIZE */
    put32(header + 12, (uint32_t)h);
    put32(header + 16, (uint32_t)w);
    put32(header + 20, (uint32_t)level_bytes(w, h, block));
    put32(header + 28, (uint32_t)levels);
    put32(header + 76, 32);                                       /* DDS_PIXELFORMAT size */
    put32(header + 80, 0x4);                                      /* DDPF_FOURCC */
    memcpy(header + 84, dxt5 ? "DXT5" : "DXT1", 4);
    put32(header + 108, 0x1000 | 0x400000 | 0x8);                 /* TEXTURE MIPMAP COMPLEX */
    if (snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp)) return 4;
    f = fopen(tmp, "wb");
    if (!f) return 4;
    if (fwrite(header, 1, sizeof(header), f) != sizeof(header)) { fclose(f); remove(tmp); return 4; }
    for (int l = 0; l < levels; ++l) {
        size_t bytes = level_bytes(w, h, block);
        uint8_t *blocks = malloc(bytes);
        if (!blocks) { fclose(f); remove(tmp); free(owned); return 4; }
        if (l > 0 && dxt5 && alpha_test > 0.0 && target > 0.0) {
            size_t n = (size_t)w * (size_t)h;
            double s = coverage_scale(level, n, cutoff, target);
            free(scaled);
            scaled = malloc(n * 4);
            if (!scaled) { free(blocks); fclose(f); remove(tmp); free(owned); return 4; }
            memcpy(scaled, level, n * 4);
            for (size_t i = 0; i < n; ++i) { double a = scaled[i * 4 + 3] * s + 0.5; scaled[i * 4 + 3] = (uint8_t)(a > 255 ? 255 : a); }
            encode_level(scaled, w, h, dxt5, blocks);
        } else encode_level(level, w, h, dxt5, blocks);
        if (fwrite(blocks, 1, bytes, f) != bytes) { free(blocks); fclose(f); remove(tmp); free(owned); free(scaled); return 4; }
        free(blocks);
        if (l + 1 < levels) {
            int nw, nh;
            uint8_t *next = downsample(level, w, h, &nw, &nh);
            free(owned);
            if (!next) { fclose(f); remove(tmp); free(scaled); return 4; }
            owned = next; level = next; w = nw; h = nh;
        }
    }
    free(owned); free(scaled);
    if (fclose(f) || rename(tmp, path)) { remove(tmp); return 4; }
    *levels_out = levels;
    return 0;
}

/* Decode level 0 of our own DDS output (for --verify). */
static int dds_decode_top(const char *path, Image *img)
{
    size_t size;
    uint8_t *d = read_file(path, &size);
    int w, h, dxt5, block, rc = 4;
    if (!d) return 4;
    if (size < 128 || memcmp(d, "DDS ", 4)) goto done;
    h = (int)get32(d + 12); w = (int)get32(d + 16);
    dxt5 = !memcmp(d + 84, "DXT5", 4);
    if (!dxt5 && memcmp(d + 84, "DXT1", 4)) goto done;
    block = dxt5 ? 16 : 8;
    if (size < 128 + level_bytes(w, h, block)) goto done;
    img->w = w; img->h = h; img->rgba = malloc((size_t)w * (size_t)h * 4);
    if (!img->rgba) goto done;
    {
        const uint8_t *b = d + 128;
        for (int by = 0; by < (h + 3) / 4; ++by)
            for (int bx = 0; bx < (w + 3) / 4; ++bx, b += block) {
                int alpha[16], pal[4][3];
                const uint8_t *cb = dxt5 ? b + 8 : b;
                uint16_t c0 = (uint16_t)(cb[0] | cb[1] << 8), c1 = (uint16_t)(cb[2] | cb[3] << 8);
                uint32_t bits = get32(cb + 4);
                for (int i = 0; i < 16; ++i) alpha[i] = 255;
                if (dxt5) {
                    int a[8]; uint64_t ab = 0;
                    a[0] = b[0]; a[1] = b[1];
                    if (a[0] > a[1]) for (int k = 1; k < 7; ++k) a[k + 1] = ((7 - k) * a[0] + k * a[1]) / 7;
                    else { for (int k = 1; k < 5; ++k) a[k + 1] = ((5 - k) * a[0] + k * a[1]) / 5; a[6] = 0; a[7] = 255; }
                    for (int k = 0; k < 6; ++k) ab |= (uint64_t)b[2 + k] << (8 * k);
                    for (int i = 0; i < 16; ++i) alpha[i] = a[(ab >> (3 * i)) & 7];
                }
                palette4(c0, c1, pal);
                if (!dxt5 && c0 <= c1) {               /* 3-colour mode (never written by us) */
                    unpack565(c0, pal[0]); unpack565(c1, pal[1]);
                    for (int k = 0; k < 3; ++k) { pal[2][k] = (pal[0][k] + pal[1][k]) / 2; pal[3][k] = 0; }
                }
                for (int i = 0; i < 16; ++i) {
                    int x = bx * 4 + i % 4, y = by * 4 + i / 4, k = (int)(bits >> (2 * i) & 3);
                    if (x >= w || y >= h) continue;
                    uint8_t *p = img->rgba + ((size_t)y * w + x) * 4;
                    p[0] = (uint8_t)pal[k][0]; p[1] = (uint8_t)pal[k][1]; p[2] = (uint8_t)pal[k][2]; p[3] = (uint8_t)alpha[i];
                }
            }
    }
    rc = 0;
done:
    free(d);
    return rc;
}

static double psnr(double mse) { return mse <= 0.0 ? 99.0 : 10.0 * log10(255.0 * 255.0 / mse); }

int main(int argc, char **argv)
{
    int verify = 0, arg = 1;
    double alpha_test = 0.0;
    const char *why = "";
    Image img = {0, 0, NULL};
    int rc, dxt5 = 0, levels = 0;
    while (arg < argc && argv[arg][0] == '-' && argv[arg][1] == '-') {
        if (!strcmp(argv[arg], "--verify")) { verify = 1; ++arg; }
        else if (!strcmp(argv[arg], "--alpha-test") && arg + 1 < argc) {
            char *end = NULL;
            alpha_test = strtod(argv[arg + 1], &end);
            if (!end || *end || !(alpha_test >= 0.0 && alpha_test < 1.0)) { fprintf(stderr, "png2dds: --alpha-test wants 0..1\n"); return 2; }
            arg += 2;
        } else break;
    }
    if (argc - arg != 2) {
        fprintf(stderr, "usage: png2dds [--alpha-test CUTOFF] in.png out.dds | png2dds --verify in.png out.dds\n");
        return 2;
    }
    const char *in = argv[arg], *out = argv[arg + 1];
    rc = png_decode(in, &img, &why);
    if (rc) { fprintf(stderr, "png2dds: %s: %s\n", in, why); return rc; }
    if ((img.w & (img.w - 1)) || (img.h & (img.h - 1))) {
        fprintf(stderr, "png2dds: %s: %dx%d is not a power of two\n", in, img.w, img.h);
        free(img.rgba);
        return 3;
    }
    for (size_t i = 0; i < (size_t)img.w * (size_t)img.h; ++i) if (img.rgba[i * 4 + 3] != 255) { dxt5 = 1; break; }
    if (verify) {
        Image d = {0, 0, NULL};
        double se_c = 0.0, se_a = 0.0, wsum = 0.0;
        size_t n = (size_t)img.w * (size_t)img.h;
        if (dds_decode_top(out, &d) || d.w != img.w || d.h != img.h) {
            fprintf(stderr, "png2dds: %s: unreadable or mismatched DDS\n", out);
            free(img.rgba); free(d.rgba);
            return 5;
        }
        for (size_t i = 0; i < n; ++i) {
            double wt = img.rgba[i*4+3] / 255.0;   /* colour under transparent texels is never seen */
            for (int c = 0; c < 3; ++c) { double e = (double)img.rgba[i*4+c] - d.rgba[i*4+c]; se_c += wt * e * e; }
            wsum += wt;
            { double e = (double)img.rgba[i*4+3] - d.rgba[i*4+3]; se_a += e * e; }
        }
        printf("%dx%d psnr_rgb=%.2f psnr_alpha=%.2f\n", img.w, img.h,
               psnr(wsum > 0 ? se_c / (3.0 * wsum) : 0.0), psnr(se_a / n));
        free(img.rgba); free(d.rgba);
        return 0;
    }
    if (dxt5 && bleed_transparent(&img)) { fprintf(stderr, "png2dds: %s: out of memory\n", in); free(img.rgba); return 4; }
    rc = write_dds(out, &img, dxt5, alpha_test, &levels);
    if (rc) fprintf(stderr, "png2dds: %s: write failed\n", out);
    else printf("%dx%d %s %d levels\n", img.w, img.h, dxt5 ? "DXT5" : "DXT1", levels);
    free(img.rgba);
    return rc;
}
