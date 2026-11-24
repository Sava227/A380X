/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef A380X_FBW_LAWS_H
#define A380X_FBW_LAWS_H
#include "a380x_guidance.h"
#ifdef __cplusplus
extern "C" {
#endif
void *a380x_fbw_laws_create(void);
void a380x_fbw_laws_destroy(void *context);
void a380x_fbw_laws_reset(void *context);
int a380x_fbw_laws_step(void *context, const A380XGuidanceInputs *in,
                       A380XGuidanceState *state, double dt);
/* Last law fault (non-finite inputs that were replaced, or a non-finite output), "" if none. */
const char *a380x_fbw_laws_fault(void *context);
#ifdef __cplusplus
}
#endif
#endif
