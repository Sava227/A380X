/* SPDX-License-Identifier: GPL-3.0-or-later */
/* TERRAIN on ND for the web NDs (xp-kit-nd/terrain-layer.js, docs/ND_TERRAIN.md): per ND the displayed
 * raster as a run-length string (re-encoded only when its revision changes), its geometry and the FBW
 * TERR box values; the TAWS state the FBW ND shows (TERR INOP, NO TERR AND WX DATA) and the EFIS CP
 * overlay (0 none, 1 WX, 2 TERR). A raster that would not fit the state leaves "cells" empty
 * ("overflow":1) rather than fail the whole publication. */
static void xpkit_terrain_json(XpKitJson *j) {
    static char cells[2][TERR_ROWS_MAX*TERR_COLS_MAX*4+1];
    static int encoded_rev[2]={-1,-1},encoded_len[2];
    xpkit_append(j,"{\"sys\":%d,\"terrOff\":%d,\"failed\":%d,\"overlay\":[%d,%d],\"nd\":[",
        terrain.taws_sys,terrain.terr_off,terrain.failed,efis[0].terr?2:efis[0].wx_requested?1:0,efis[1].terr?2:efis[1].wx_requested?1:0);
    for(int s=0;s<2;++s) {
        const TerrDisplay *d=&nd_data.terrain[s];
        if(!d->on){xpkit_append(j,"%s{\"on\":0,\"rev\":%d}",s?",":"",d->rev);continue;}
        if(encoded_rev[s]!=d->rev){encoded_len[s]=terr_rle_encode(d,cells[s],sizeof(cells[s]));encoded_rev[s]=d->rev;}
        int fits=encoded_len[s]>=0 && sizeof(j->data)-j->size>(size_t)encoded_len[s]+8192;
        xpkit_append(j,"%s{\"on\":1,\"rev\":%d,\"mode\":%d,\"range\":%.10g,\"x0\":%d,\"y0\":%d,\"cols\":%d,\"rows\":%d,"
            "\"min\":%d,\"minMode\":%d,\"max\":%d,\"maxMode\":%d,\"peaks\":%d,\"overflow\":%d,\"cells\":\"%s\"}",
            s?",":"",d->rev,d->geo.mode,d->geo.range_nm,d->geo.x0,d->geo.y0,d->geo.cols,d->geo.rows,
            d->shown.min_elev,d->shown.min_mode,d->shown.max_elev,d->shown.max_mode,d->shown.peaks,!fits,fits?cells[s]:"");
    }
    xpkit_append(j,"]}");
}

/* TAWS / GPWS alerts for the FBW PFD (TawsPublisher: L:A32NX_EGPWS_ALERT_#index#_DISCRETE_WORD_1; the
 * A380 warning group reads index 1, both are published): bit 12 PULL UP (red) for the TAWS warning or
 * GPWS modes 1/2, bit 13 TERRAIN (amber) for the TAWS caution. Word 2 stays 0: its bit 15 is the
 * windshear display. Words carry the integer value, as the FG words do (bridge.js rawWord). */
static void xpkit_taws_words(XpKitJson *j) {
    const int pull_up=nd_data.taws_warning||nd_data.gpws_pull_up;
    const unsigned w1=(pull_up?1u<<11:0u)|(nd_data.taws_caution&&!pull_up?1u<<12:0u);
    for(int unit=1;unit<=2;++unit)
        xpkit_append(j,"\"L:A32NX_EGPWS_ALERT_%d_DISCRETE_WORD_1\":[%u,true],\"L:A32NX_EGPWS_ALERT_%d_DISCRETE_WORD_2\":[0,true],",
                     unit,w1,unit);
}
