/* SPDX-License-Identifier: GPL-3.0-or-later
 * XPLM-free checks of the A380 EFB core (a380x_efb_core.c): sections, door / slide rules, typed
 * payload and radio entries, the FQMS tank view, the bounded JSON writer and the action grammar.
 *
 * clang -std=c11 -Wall -Wextra -Werror test_efb_core.c a380x_efb_core.c -lm -o test_efb_core
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "a380x_efb_core.h"

static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static void test_sections(void)
{
    int page, section, tab;
    CHECK(a380x_efb_section_find("home") == A380X_EFB_HOME && a380x_efb_section_find("settings") == A380X_EFB_SETTINGS);
    CHECK(a380x_efb_section_find("nope") == -1 && a380x_efb_section_find(NULL) == -1 && a380x_efb_section_name(10) == NULL);
    CHECK(a380x_efb_tab_count(A380X_EFB_GROUND) == 3 && a380x_efb_tab_count(A380X_EFB_HOME) == 0 && a380x_efb_tab_count(-1) == 0);
    CHECK(!strcmp(a380x_efb_tab_name(A380X_EFB_OPS, 1), "presets") && a380x_efb_tab_name(A380X_EFB_OPS, 2) == NULL);
    CHECK(a380x_efb_tab_find(A380X_EFB_FLIGHT, "comms") == 2 && a380x_efb_tab_find(A380X_EFB_FLIGHT, "doors") == -1);
    /* Every legacy page lands on a section whose closest legacy page is itself. */
    for (page = 0; page < 10; ++page) {
        a380x_efb_legacy_page_target(page, &section, &tab);
        CHECK(section >= 0 && section < A380X_EFB_SECTIONS);
        CHECK(a380x_efb_section_legacy_page(section, tab) == page);
    }
    a380x_efb_legacy_page_target(99, &section, &tab);
    CHECK(section == A380X_EFB_HOME);
}

static void test_doors(void)
{
    float doors[A380X_EFB_DOORS] = {0};
    unsigned char slides[A380X_EFB_SLIDES] = {0};
    int i;
    CHECK(!strcmp(a380x_efb_door_name(0), "M1L") && !strcmp(a380x_efb_door_name(9), "M5R") &&
          !strcmp(a380x_efb_door_name(15), "U3R") && !strcmp(a380x_efb_door_name(16), "CKPT") &&
          !strcmp(a380x_efb_door_name(19), "BULK") && a380x_efb_door_name(20) == NULL);
    CHECK(a380x_efb_door_find("U2L") == 12 && a380x_efb_door_find("FWD") == 17 && a380x_efb_door_find("X") == -1);
    for (i = 0; i < A380X_EFB_DOORS; ++i) CHECK(a380x_efb_door_has_slide(i) == (i < 16));
    /* An armed door cannot be opened; closing (an open, i.e. not closed, door) is always allowed. */
    CHECK(a380x_efb_door_toggle_rule(3, 0.0f, 1) == A380X_EFB_DOOR_ARMED);
    CHECK(a380x_efb_door_toggle_rule(3, 0.0f, 0) == A380X_EFB_DOOR_OK);
    CHECK(a380x_efb_door_toggle_rule(3, 1.0f, 1) == A380X_EFB_DOOR_OK);
    CHECK(a380x_efb_door_toggle_rule(17, 0.0f, 1) == A380X_EFB_DOOR_OK);   /* cargo: no slide */
    CHECK(a380x_efb_door_toggle_rule(20, 0.0f, 0) == A380X_EFB_DOOR_INVALID);
    /* An open (or moving) door cannot be armed; disarming always works; no slide on cargo/cockpit. */
    CHECK(a380x_efb_slide_rule(2, 0.5f, 1) == A380X_EFB_SLIDE_DOOR_OPEN);
    CHECK(a380x_efb_slide_rule(2, 0.005f, 1) == A380X_EFB_SLIDE_OK);
    CHECK(a380x_efb_slide_rule(2, 1.0f, 0) == A380X_EFB_SLIDE_OK);
    CHECK(a380x_efb_slide_rule(16, 0.0f, 1) == A380X_EFB_SLIDE_NO_SLIDE);
    CHECK(a380x_efb_slide_rule(18, 0.0f, 0) == A380X_EFB_SLIDE_NO_SLIDE);
    CHECK(a380x_efb_slide_rule(-1, 0.0f, 0) == A380X_EFB_SLIDE_INVALID);
    CHECK(a380x_efb_slide_rule(1, NAN, 1) == A380X_EFB_SLIDE_DOOR_OPEN);   /* unknown ratio never arms */
    doors[0] = 1.0f; doors[10] = 0.3f; doors[17] = 1.0f;
    CHECK(a380x_efb_arm_all(doors, slides) == 2);
    CHECK(!slides[0] && !slides[10] && slides[1] && slides[15] && a380x_efb_slides_armed(slides) == 14);
    CHECK(a380x_efb_doors_open(doors) == 3);
    doors[5] = 0.2f;   /* opened by another path */
    CHECK(a380x_efb_slides_follow_doors(doors, slides) == 1 && !slides[5] && a380x_efb_slides_armed(slides) == 13);
    CHECK(a380x_efb_slides_follow_doors(doors, slides) == 0);
}

static void test_payload(void)
{
    A380XEfbLoad load = {{14, 76, 56, 338}, {28577, 20310, 2513}};
    long value = -1;
    int kg = -1;
    CHECK(a380x_efb_class_seats[0] + a380x_efb_class_seats[1] + a380x_efb_class_seats[2] + a380x_efb_class_seats[3] == 484);
    CHECK(a380x_efb_hold_max_kg[0] + a380x_efb_hold_max_kg[1] + a380x_efb_hold_max_kg[2] == 51400);
    CHECK(a380x_efb_load_pax(&load) == 484 && fabs(a380x_efb_load_kg(&load) - (484 * 104.0 + 51400)) < 1e-9);
    CHECK(a380x_efb_load_kg(&load) <= A380X_EFB_MAX_PAYLOAD_KG);
    CHECK(a380x_efb_parse_count("338", 338, &value) == A380X_EFB_ENTRY_OK && value == 338);
    CHECK(a380x_efb_parse_count("0", 14, &value) == A380X_EFB_ENTRY_OK && value == 0);
    CHECK(a380x_efb_parse_count("339", 338, &value) == A380X_EFB_ENTRY_TOO_LARGE);
    CHECK(a380x_efb_parse_count("", 10, &value) == A380X_EFB_ENTRY_EMPTY && a380x_efb_parse_count(NULL, 10, &value) == A380X_EFB_ENTRY_EMPTY);
    CHECK(a380x_efb_parse_count("-1", 10, &value) == A380X_EFB_ENTRY_NOT_A_NUMBER);
    CHECK(a380x_efb_parse_count("1e3", 9999, &value) == A380X_EFB_ENTRY_NOT_A_NUMBER);
    CHECK(a380x_efb_parse_count("12.5", 99, &value) == A380X_EFB_ENTRY_NOT_A_NUMBER);
    CHECK(a380x_efb_parse_count("12345678", 99999999, &value) == A380X_EFB_ENTRY_TOO_LARGE);   /* > 7 digits */
    CHECK(a380x_efb_parse_cargo("2513", 2, 0, &kg) == A380X_EFB_ENTRY_OK && kg == 2513);
    CHECK(a380x_efb_parse_cargo("2514", 2, 0, &kg) == A380X_EFB_ENTRY_TOO_LARGE);
    CHECK(a380x_efb_parse_cargo("5540", 2, 1, &kg) == A380X_EFB_ENTRY_OK && kg == 2513);   /* 5540 lb ~ 2512.9 kg */
    CHECK(a380x_efb_parse_cargo("5541", 2, 1, &kg) == A380X_EFB_ENTRY_TOO_LARGE);
    CHECK(a380x_efb_parse_cargo("1000", 3, 0, &kg) == A380X_EFB_ENTRY_NOT_A_NUMBER);
}

static void test_radios(void)
{
    int khz = 0, code = 0;
    CHECK(a380x_efb_parse_com_khz("121.9", &khz) && khz == 121900);
    CHECK(a380x_efb_parse_com_khz("118.005", &khz) && khz == 118005);
    CHECK(a380x_efb_parse_com_khz("136990", &khz) && khz == 136990);
    CHECK(!a380x_efb_parse_com_khz("117.975", &khz) && !a380x_efb_parse_com_khz("137.000", &khz));
    CHECK(!a380x_efb_parse_com_khz("121.020", &khz) && !a380x_efb_parse_com_khz("121.003", &khz));   /* no channel */
    CHECK(!a380x_efb_parse_com_khz("121..9", &khz) && !a380x_efb_parse_com_khz("12.19", &khz) && !a380x_efb_parse_com_khz("", &khz));
    CHECK(a380x_efb_parse_squawk("7000", &code) && code == 7000 && a380x_efb_parse_squawk("0000", &code) && code == 0);
    CHECK(!a380x_efb_parse_squawk("7800", &code) && !a380x_efb_parse_squawk("700", &code) && !a380x_efb_parse_squawk("70000", &code));
}

static void test_fuel(void)
{
    float xp[9] = {30000, 15000, 50000, 20000, 15000, 5000, 5000, 10000, 0};
    double out[A380X_EFB_TANKS], sum = 0, total = 0;
    int i;
    CHECK(a380x_efb_fqms_tanks(xp, 9, out));
    for (i = 0; i < A380X_EFB_TANKS; ++i) sum += out[i];
    for (i = 0; i < 8; ++i) total += xp[i];
    CHECK(fabs(sum - total) < 1e-6);
    CHECK(fabs(out[4] - A380X_EFB_FEED23_KG) < 1e-6 && fabs(out[3] - (50000 - A380X_EFB_FEED23_KG)) < 1e-6);
    CHECK(out[2] == 15000 && out[7] == 15000 && out[10] == 10000 && !strcmp(a380x_efb_tank_names[10], "TRIM"));
    xp[1] = NAN;
    CHECK(a380x_efb_fqms_tanks(xp, 9, out) && out[1] == 0);
    CHECK(!a380x_efb_fqms_tanks(xp, 7, out) && isnan(out[0]) && isnan(out[10]));
}

static void test_json(void)
{
    char buffer[256], tiny[16];
    A380XEfbJson j;
    a380x_efbj_init(&j, buffer, sizeof(buffer));
    a380x_efbj_object(&j, NULL);
    a380x_efbj_number(&j, "a", 1.25, 1);
    a380x_efbj_number(&j, "nan", NAN, 2);
    a380x_efbj_number(&j, "inf", INFINITY, 0);
    a380x_efbj_number(&j, "neg0", -0.01, 1);
    a380x_efbj_string(&j, "s", "q\"b\\\n");
    a380x_efbj_array(&j, "list");
    a380x_efbj_int(&j, NULL, 3); a380x_efbj_bool(&j, NULL, 1); a380x_efbj_null(&j, NULL);
    a380x_efbj_object(&j, NULL); a380x_efbj_end(&j, '}');
    a380x_efbj_end(&j, ']');
    a380x_efbj_end(&j, '}');
    CHECK(a380x_efbj_finish(&j) > 0);
    CHECK(!strcmp(buffer, "{\"a\":1.3,\"nan\":null,\"inf\":null,\"neg0\":0.0,\"s\":\"q\\\"b\\\\\\u000a\",\"list\":[3,true,null,{}]}") ||
          !strcmp(buffer, "{\"a\":1.2,\"nan\":null,\"inf\":null,\"neg0\":0.0,\"s\":\"q\\\"b\\\\\\u000a\",\"list\":[3,true,null,{}]}"));
    CHECK(!strstr(buffer, "nan,") && !strstr(buffer, "inf,"));
    /* Overflow and unbalanced nesting are refused (and the buffer is emptied). */
    a380x_efbj_init(&j, tiny, sizeof(tiny));
    a380x_efbj_object(&j, NULL);
    a380x_efbj_string(&j, "long", "a value that does not fit");
    a380x_efbj_end(&j, '}');
    CHECK(a380x_efbj_finish(&j) == -1 && tiny[0] == 0);
    a380x_efbj_init(&j, buffer, sizeof(buffer));
    a380x_efbj_object(&j, NULL);
    CHECK(a380x_efbj_finish(&j) == -1);
    a380x_efbj_init(&j, buffer, sizeof(buffer));
    a380x_efbj_end(&j, '}');
    CHECK(a380x_efbj_finish(&j) == -1);
}

static void test_actions(void)
{
    A380XEfbAction a;
    char longer[200];
    CHECK(a380x_efb_parse_action("12 nav ground doors", &a) && a.id == 12 && !strcmp(a.verb, "nav") && a.argc == 2 &&
          !strcmp(a.arg[0], "ground") && !strcmp(a.arg[1], "doors"));
    CHECK(a380x_efb_parse_action("7  fail_clear  ", &a) && a.id == 7 && a.argc == 0 && !strcmp(a.verb, "fail_clear"));
    CHECK(a380x_efb_parse_action("3 radio stby1 121.900", &a) && a.argc == 2 && !strcmp(a.arg[1], "121.900"));
    CHECK(!a380x_efb_parse_action("0 nav home", &a));            /* id must be positive */
    CHECK(!a380x_efb_parse_action("nav home", &a));              /* id first */
    CHECK(!a380x_efb_parse_action("5", &a) && !a380x_efb_parse_action("5 ", &a));
    CHECK(!a380x_efb_parse_action("5 Nav home", &a));            /* lower-case verb */
    CHECK(!a380x_efb_parse_action("5 nav a b c d e", &a));       /* at most four arguments */
    CHECK(!a380x_efb_parse_action("5 nav h\tome", &a));          /* printable ASCII only */
    CHECK(!a380x_efb_parse_action("1234567890 nav home", &a));   /* at most nine id digits */
    memset(longer, 'a', sizeof(longer)); longer[0] = '1'; longer[1] = ' '; longer[199] = 0;
    CHECK(!a380x_efb_parse_action(longer, &a));
    CHECK(!a380x_efb_parse_action(NULL, &a));
}

int main(void)
{
    test_sections();
    test_doors();
    test_payload();
    test_radios();
    test_fuel();
    test_json();
    test_actions();
    if (failures) { fprintf(stderr, "%d EFB core check(s) failed\n", failures); return 1; }
    puts("EFB core: ten sections and legacy pages, door/slide rules, typed payload/radio entries, FQMS tanks, bounded JSON and action grammar passed");
    return 0;
}
