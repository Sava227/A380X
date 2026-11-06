/* SPDX-License-Identifier: GPL-3.0-or-later
 * Live map boundary: native aircraft flight plan, raw navigation database and
 * actual sensor/EFIS state. Never imports X-Plane's stock FMS route/map.
 */
static void xpkit_string(XpKitJson *j,const char *text) {
    const unsigned char *c=(const unsigned char *)text;
    while(*c && *c!='"' && *c!='\\' && *c>=32) ++c;
    /* Nothing to escape (the usual ident): one append. The state is now published at up to 75 Hz. */
    if(!*c) { xpkit_append(j,"\"%s\"",text); return; }
    xpkit_append(j,"\"");
    for(const unsigned char *p=(const unsigned char *)text;*p;++p) {
        if(*p=='"'||*p=='\\')xpkit_append(j,"\\%c",*p);
        else if(*p<32)xpkit_append(j,"\\u%04x",*p);
        else xpkit_append(j,"%c",*p);
    }
    xpkit_append(j,"\"");
}
static void xpkit_scalar(XpKitJson *j,double value) {
    if(isfinite(value))xpkit_append(j,"%.10g",value);else xpkit_append(j,"null");
}
static void xpkit_nd_json(XpKitJson *j) {
    xpkit_append(j,"{\"positionValid\":%s,",nd_data.position_valid?"true":"false");
    xpkit_number(j,"lat",nd_data.lat);xpkit_number(j,"lon",nd_data.lon);
    xpkit_number(j,"magVar",fmsa_getf(FMSA_IN_MAGVAR));xpkit_number(j,"tas",fmsa_getf(FMSA_IN_TAS)*1.94384449);
    xpkit_number(j,"windDir",fmsa_getf(FMSA_IN_WIND_DIR));xpkit_number(j,"windSpeed",fmsa_getf(FMSA_IN_WIND_SPD)*1.94384449);
    xpkit_number(j,"groundSpeed",ground_speed_knots);xpkit_number(j,"zulu",fmsa_getf(FMSA_IN_ZULU));
    xpkit_number(j,"selectedHeading",guidance.selected_heading_deg);xpkit_number(j,"selectedAltitude",guidance.selected_altitude_ft);
    xpkit_append(j,"\"controls\":[");
    for(int s=0;s<2;++s) {
        EfisSide *e=&efis[s];
        int option=e->cstr?1:e->vord?2:e->wpt?4:e->ndb?8:e->arpt?16:0;
        xpkit_append(j,"%s{\"mode\":%d,\"range\":%d,\"zoom\":%d,\"option\":%d,\"vor1\":%d,\"vor2\":%d,\"ls\":%d,\"powered\":%d}",
            s?",":"",e->mode,e->range_index,xpkit_nd[s].zoom,option,e->vor1,e->vor2,e->ls,e->powered);
    }
    xpkit_append(j,"],\"pointers\":[");
    for(int s=0;s<2;++s) {
        XpKitNd *d=&xpkit_nd[s];
        xpkit_append(j,"%s{\"x\":%d,\"y\":%d,\"inside\":%d,\"events\":[",s?",":"",d->x,d->y,d->inside);
        unsigned first=d->serial>XPKIT_ND_EVENTS?d->serial-XPKIT_ND_EVENTS+1:1;
        for(unsigned seq=first;seq<=d->serial;++seq) {
            XpKitNdEvent *e=&d->events[(seq-1)%XPKIT_ND_EVENTS];
            xpkit_append(j,"%s[%u,%d,%d,%d,%d]",seq==first?"":",",e->seq,e->type,e->x,e->y,e->amount);
        }
        xpkit_append(j,"]}");
    }
    xpkit_append(j,"],\"radios\":[");
    for(int s=0;s<2;++s) {
        EfisNdVor *r=&nd_data.vor[s];
        xpkit_append(j,"%s{\"ident\":",s?",":"");xpkit_string(j,r->id);
        xpkit_append(j,",\"valid\":%s,\"dmeValid\":%s,",r->valid?"true":"false",r->dme_valid?"true":"false");
        xpkit_number(j,"frequency",r->frequency_mhz);xpkit_number(j,"bearing",r->bearing_mag);
        xpkit_number(j,"distance",r->distance_nm);xpkit_number(j,"course",r->course_mag);
        xpkit_append(j,"\"deviation\":");xpkit_scalar(j,r->deviation_dots);xpkit_append(j,"}");
    }
    xpkit_append(j,"],\"airports\":[");
    int count=0;
    for(int n=0;n<nd_data.nav_count[0];++n) {
        EfisNdNavPoint *p=&nd_data.nav[0][n];if(!p->id[0])continue;
        xpkit_append(j,"%s{\"idarpt\":",count++?",":"");xpkit_string(j,p->id);
        xpkit_append(j,",\"name\":");xpkit_string(j,p->id);
        xpkit_append(j,",\"iata\":\"\",\"coordinates\":{\"lat\":%.10g,\"lon\":%.10g},\"elev\":null}",p->lat,p->lon);
    }
    xpkit_append(j,"],\"airport\":");
    const EfisNdNavPoint *nearest=nd_data.nav_count[0]?&nd_data.nav[0][0]:NULL;
    xpkit_string(j,nearest&&nearest->distance_nm<20?nearest->id:"");
    xpkit_append(j,",\"symbols\":[");count=0;
    const int flags[]={16,2,4,8};
    for(int kind=0;kind<4;++kind)for(int n=0;n<nd_data.nav_count[kind];++n) {
        EfisNdNavPoint *p=&nd_data.nav[kind][n];
        if(!p->id[0]||!isfinite(p->lat)||!isfinite(p->lon))continue;
        xpkit_append(j,"%s{\"databaseId\":\"nav-%d-%d\",\"ident\":",count++?",":"",kind,n);xpkit_string(j,p->id);
        xpkit_append(j,",\"location\":{\"lat\":%.10g,\"long\":%.10g},\"type\":%d}",p->lat,p->lon,flags[kind]|256);
    }
    xpkit_append(j,"],\"route\":[");count=0;
    const A380XFplnPlan *plan=fmsa_fpln?a380x_fpln_plan(fmsa_fpln,A380X_PLAN_ACTIVE):NULL;
    if(plan)for(int n=0;n<plan->pri.count;++n) {
        const A380XFplnLeg *leg=&plan->pri.el[n];
        if((leg->flags&A380X_LEGF_DISCONTINUITY)||!isfinite(leg->term_lat)||!isfinite(leg->term_lon))continue;
        xpkit_append(j,"%s{\"ident\":",count++?",":"");xpkit_string(j,leg->ident);
        xpkit_append(j,",\"lat\":%.10g,\"lon\":%.10g,\"active\":%s",leg->term_lat,leg->term_lon,n==plan->active_leg_index?"true":"false");
        /* Only the native FMS's calculated straight segments are represented;
         * AF/RF/holds are not turned into invented straight guidance. */
        if((leg->type==A380X_LEG_TF||leg->type==A380X_LEG_DF||leg->type==A380X_LEG_CF)&&isfinite(leg->start_lat)&&isfinite(leg->start_lon))
            xpkit_append(j,",\"start\":[%.10g,%.10g]",leg->start_lat,leg->start_lon);
        xpkit_append(j,"}");
    }
    xpkit_append(j,"],\"to\":");
    const A380XFplnLeg *to=plan&&plan->active_leg_index>=0&&plan->active_leg_index<plan->pri.count?&plan->pri.el[plan->active_leg_index]:NULL;
    if(to&&!(to->flags&A380X_LEGF_DISCONTINUITY)&&isfinite(to->term_lat)&&isfinite(to->term_lon)) {
        xpkit_append(j,"{\"ident\":");xpkit_string(j,to->ident);xpkit_append(j,",\"lat\":%.10g,\"lon\":%.10g}",to->term_lat,to->term_lon);
    }else xpkit_append(j,"null");
    /* FMGC pseudo waypoints for the bundle's PseudoWaypointLayer (xp-kit-nd/fmgc-pwp-layer.js), see
     * a380x_fmgc_nd_pwp_json: on the path (lat/lon) or along the track (dist only); dist + alt feed the VD. */
    {
        char pseudo[2048];
        if(!a380x_fmgc_nd_pwp_json(fmsa_nd_pwp,fmsa_nd_pwp_count,pseudo,sizeof(pseudo)))snprintf(pseudo,sizeof(pseudo),"[]");
        xpkit_append(j,",\"pseudo\":%s}",pseudo);
    }
}
