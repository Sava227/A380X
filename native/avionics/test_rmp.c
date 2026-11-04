/* SPDX-License-Identifier: GPL-3.0-or-later
 * a380x_rmp.h: FBW VhfComController entry examples, swap, VHF 3 modes, transponder, power states. */
#include <assert.h>
#include <stdio.h>
#include "a380x_rmp.h"

static A380XRmpRadios radios;
static A380XRmp rmp;

static void fresh(void)
{
    a380x_rmp_radios_init(&radios);
    a380x_rmp_init(&rmp, 0);
    a380x_rmp_update(&rmp, &radios, 1, 0, 0);
}
static void type(const char *digits) { for (; *digits; ++digits) a380x_rmp_key(&rmp, &radios, *digits - '0'); }
static void expect(const char *digits, const char *shown, uint32_t frequency)
{
    fresh();
    type(digits);
    if (strcmp(rmp.entry[0].displayed, shown) || rmp.entry[0].frequency != frequency) {
        printf("%s -> %s %07x, expected %s %07x\n", digits, rmp.entry[0].displayed, rmp.entry[0].frequency, shown, frequency);
        assert(0);
    }
}

int main(void)
{
    /* VhfComController.ts examples */
    expect("18", "118000", 0x1180000); expect("187", "118700", 0x1187000); expect("1875", "118750", 0x1187500);
    expect("8", "118000", 0x1180000); expect("87", "118700", 0x1187000); expect("875", "118750", 0x1187500);
    expect("1", "1_____", 0);
    fresh(); type("17"); assert(a380x_rmp_entry_invalid(&rmp.entry[0]));        /* 17_.___ amber */
    fresh(); type("3"); assert(!strcmp(rmp.entry[0].displayed, "13____"));
    fresh(); type("37"); assert(a380x_rmp_entry_invalid(&rmp.entry[0]));
    fresh(); type("121"); assert(rmp.entry[0].frequency == 0x1210000);

    /* LSK on the selected row enters the standby; ADK swaps */
    fresh(); type("875"); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_LSK1);
    assert(radios.standby[0] == 0x1187500 && !rmp.entry[0].entered[0]);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_ADK1);
    assert(radios.active[0] == 0x1187500 && radios.standby[0] == 0x1183000);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_LSK1);
    assert(!strcmp(rmp.message, "VHF1 FREQ NOT COMPLETED"));
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_MSG_CLR); assert(!rmp.message[0]);
    /* other row: first press selects it */
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_ADK2); assert(rmp.row == 1 && radios.active[1] == 0x1215000);
    /* CLR: last digit, then held -> whole entry */
    fresh(); type("1875"); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_CLR);
    assert(!strcmp(rmp.entry[0].entered, "187"));
    a380x_rmp_update(&rmp, &radios, 1, 0, 1.2f); assert(!rmp.entry[0].entered[0]);
    /* VHF 3: DATA standby, UP to EMER (121.500), digits only in frequency mode */
    fresh(); rmp.row = 2; type("8"); assert(!rmp.entry[2].entered[0]);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_UP); assert(radios.standby_mode[2] == A380X_RMP_MODE_EMERGENCY && radios.standby[2] == 0x1215000);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_UP); assert(radios.standby_mode[2] == A380X_RMP_MODE_EMERGENCY);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_DOWN); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_DOWN);
    assert(radios.standby_mode[2] == A380X_RMP_MODE_FREQUENCY);
    /* transponder */
    fresh(); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_SQWK); type("4521");
    assert(!strcmp(rmp.xpdr_entry, "4521") && radios.xpdr_code == 2000);
    a380x_rmp_update(&rmp, &radios, 1, 0, 1.1f); assert(radios.xpdr_code == 4521 && !rmp.xpdr_entry[0]);
    type("78"); assert(rmp.xpdr_invalid); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_LSK1);
    assert(radios.xpdr_code == 4521 && !strcmp(rmp.message, "SQUAWK CODE REVERTED TO PREV ENTRY"));
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_ADK3); assert(radios.ident_left > 0);
    a380x_rmp_radios_tick(&radios, &rmp, 1, 6.0f); assert(radios.ident_left == 0 && !rmp.message[0]);
    /* transmit select toggles; nothing works while OFF */
    fresh(); a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_VHF2_CALL); assert(rmp.tx == 2);
    a380x_rmp_key(&rmp, &radios, A380X_RMP_KEY_VHF2_CALL); assert(rmp.tx == 0);
    rmp.brightness = 0; a380x_rmp_update(&rmp, &radios, 1, 0, 0); assert(rmp.state == A380X_RMP_OFF_STANDBY);
    type("8"); assert(!rmp.entry[0].entered[0]);
    a380x_rmp_update(&rmp, &radios, 0, 0, 0); assert(rmp.state == A380X_RMP_OFF_FAILED);
    /* BCD <-> kHz */
    assert(a380x_rmp_bcd_to_khz(0x1183000) == 118300 && a380x_rmp_khz_to_bcd(121500) == 0x1215000);
    puts("A380 RMP: FBW frequency entry, swap, VHF 3 modes, transponder, transmit select and power states passed");
    return 0;
}
