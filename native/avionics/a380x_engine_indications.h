/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Trent 900 indications that X-Plane's two-spool engine model does not provide, derived the way
 * FlyByWire's A380X does (simulator independent; shared by the ECAM SD data and the producer in
 * a380x_engine_indications.inc).
 */
#ifndef A380X_ENGINE_INDICATIONS_H
#define A380X_ENGINE_INDICATIONS_H

#include <math.h>

/* X-Plane's N2 is the HP core, which is FBW's N3 (trent_engine.rs gearbox / FADEC N3). */
static inline double a380x_eng_n3_from_xp_n2(double xp_n2) { return xp_n2; }

/* FBW oil quantity: total (FBW 17-20 qt, fixed 18.5) x X-Plane oil ratio, minus the thrust-
 * dependent gulp (FBW EngineControl_A380X oil gulp polynomial, thrust in newtons). */
static inline double a380x_eng_oil_quantity_qt(double oil_ratio, double thrust_n)
{
    double gulp = (20.1968848 - 1.2270302e-6 * thrust_n + 1.78442e-10 * thrust_n * thrust_n) / 100.0;
    double ratio = isfinite(oil_ratio) ? fmin(fmax(oil_ratio, 0.0), 1.0) : NAN;
    gulp = fmin(fmax(gulp, 0.0), 0.9);
    return 18.5 * ratio * (1.0 - gulp);
}

#endif
