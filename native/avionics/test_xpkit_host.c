/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <spawn.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include "a380x_xpkit_shared.h"
extern char **environ;
static double seconds(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec*1e-9;
}
int main(int argc,char **argv) {
    if(argc!=6) return 2;
    char filename[]="/tmp/a380x-xpkit-probe-XXXXXX";
    int fd=mkstemp(filename); if(fd<0) return 3;
    if(ftruncate(fd,sizeof(A380XXpKitShared))) return 4;
    A380XXpKitShared *s=mmap(NULL,sizeof(*s),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0); close(fd);
    if(s==MAP_FAILED) return 5;
    memset(s,0,sizeof(*s)); s->magic=XPKIT_MAGIC; s->version=XPKIT_VERSION;
    FILE *input=fopen(argv[3],"rb"); if(!input) return 6;
    size_t count=fread(s->state,1,XPKIT_JSON_CAP-1,input); fclose(input);
    s->state_size=(uint32_t)count; s->state_time=seconds(); atomic_store(&s->state_seq,2);
    /* Explicit isolated helper fixture, never copied into installed assets. */
    const char *sd=getenv("A380X_SD_JSON");if(!sd)sd="{\"powered\":true,\"page\":6,\"more\":false,\"vars\":{}}";
    if(strlen(sd)>=XPKIT_SD_CAP)return 10;
    strcpy(s->sd.json,sd);s->sd.size=(uint32_t)strlen(sd);atomic_store(&s->sd.sequence,2);
    for(int side=0;side<2;++side) {
        const char *fixture=getenv(side?"A380X_MFD_FIXTURE_RIGHT":"A380X_MFD_FIXTURE_LEFT");
        if(fixture) {
            FILE *f=fopen(fixture,"rb");if(!f)return 8;
            size_t read=fread(&s->mfd[side],sizeof(s->mfd[side]),1,f);fclose(f);
            if(read!=1)return 9;
        }
    }
    /* Optional EFB tablet fixtures (A380X_EFB_JSON_0/1): native frames (test_efb_adapter --frame),
     * several separated by a "---" line; one is published every 3 s, so touch events in a later
     * frame drive the real page and its actions arrive in the tablet's action ring. */
    static char efb_text[2][8*XPKIT_EFB_CAP];char *efb_frames[2][8];int efb_count[2]={0,0},efb_next[2]={0,0};
    for(int t=0;t<2;++t) {
        const char *path=getenv(t?"A380X_EFB_JSON_1":"A380X_EFB_JSON_0");
        if(!path)continue;
        FILE *f=fopen(path,"rb");if(!f)return 11;
        size_t n=fread(efb_text[t],1,sizeof(efb_text[t])-1,f);fclose(f);efb_text[t][n]=0;
        for(char *p=efb_text[t];p&&efb_count[t]<8;) {
            efb_frames[t][efb_count[t]++]=p;
            char *cut=strstr(p,"\n---\n");
            if(cut){*cut=0;p=cut+5;}else p=NULL;
        }
    }
    for(int t=0;t<2;++t) if(efb_count[t]) {
        A380XXpKitEfb *e=&s->efb[t];size_t n=strlen(efb_frames[t][0]);if(n>=XPKIT_EFB_CAP)return 12;
        memcpy(e->json,efb_frames[t][0],n+1);e->size=(uint32_t)n;atomic_store(&e->sequence,2);efb_next[t]=1;
    }
    pid_t pid=0; char *args[]={argv[1],filename,argv[2],argv[5],NULL};
    if(posix_spawn(&pid,argv[1],NULL,NULL,args,environ)) return 7;
    double start=seconds();
    int cycle=getenv("A380X_SD_CYCLE")!=NULL,last_page=-1,captured[16]={0};
    while(seconds()-start<20) {
        atomic_fetch_add(&s->state_seq,1); s->state_time=seconds(); atomic_fetch_add(&s->state_seq,1);
        double elapsed=seconds()-start;
        /* Later fixture frames are paced from the tablet page's readiness: a page's first frame
         * deliberately skips touch history, so touches must arrive after it. */
        static double efb_ready_at[2]={-1,-1};
        for(int t=0;t<2;++t) if(efb_ready_at[t]<0&&atomic_load(&s->ready[XPKIT_EFB_VIEW0+t])&&atomic_load(&s->frame_seq[XPKIT_EFB_VIEW0+t])>=2)efb_ready_at[t]=elapsed;
        for(int t=0;t<2;++t) if(efb_next[t]<efb_count[t]&&efb_ready_at[t]>=0&&elapsed>efb_ready_at[t]+2.0*efb_next[t]) {
            A380XXpKitEfb *e=&s->efb[t];const char *frame=efb_frames[t][efb_next[t]++];size_t n=strlen(frame);
            if(n>=XPKIT_EFB_CAP)return 12;
            atomic_fetch_add(&e->sequence,1);memcpy(e->json,frame,n+1);e->size=(uint32_t)n;atomic_fetch_add(&e->sequence,1);
        }
        if(cycle) {
            int page=(int)elapsed%16;
            if(page!=last_page) {
                atomic_fetch_add(&s->sd.sequence,1);
                s->sd.size=(uint32_t)snprintf(s->sd.json,sizeof(s->sd.json),"{\"powered\":true,\"page\":%d,\"more\":true,\"vars\":{}}",page);
                atomic_fetch_add(&s->sd.sequence,1);last_page=page;
            }
            if(elapsed-(int)elapsed>.7 && !captured[page] && atomic_load(&s->frame_seq[6])>0) {
                char name[4096];snprintf(name,sizeof(name),"%s-sd-page-%02d.ppm",argv[4],page);
                unsigned seq=atomic_load(&s->frame_seq[6]);
                if(!(seq&1)){FILE *image=fopen(name,"wb");fprintf(image,"P6\n%d %d\n255\n",xpkit_view_width(6),xpkit_view_height(6));
                    unsigned char *pixels=xpkit_view_pixels(s,6);for(size_t k=0;k<XPKIT_MFD_BYTES;k+=4)fwrite(pixels+k,1,3,image);fclose(image);captured[page]=1;}
            }
        }
        struct timespec pause={0,50000000}; nanosleep(&pause,NULL);
    }
    int ok=1;
    for(int side=0;side<XPKIT_VIEWS;side++) {
        unsigned seq=atomic_load(&s->frame_seq[side]);
        unsigned ready=atomic_load(&s->ready[side]); size_t colored=0;
        char output[4096]; snprintf(output,sizeof(output),"%s-%d.ppm",argv[4],side);
        FILE *image=fopen(output,"wb"); fprintf(image,"P6\n%d %d\n255\n",xpkit_view_width(side),xpkit_view_height(side));
        unsigned char *pixels=xpkit_view_pixels(s,side);
        for(size_t i=0;i<xpkit_view_bytes(side);i+=4) {
            fwrite(pixels+i,1,3,image);
            if(pixels[i]>30||pixels[i+1]>30||pixels[i+2]>30) colored++;
        }
        fclose(image);
        printf("side=%d ready=%u frames=%u nonblack=%zu\n",side,ready,seq/2,colored);
        if(side>=XPKIT_EFB_VIEW0) {
            A380XXpKitEfb *e=&s->efb[side-XPKIT_EFB_VIEW0];
            uint32_t head=atomic_load(&e->action_head);
            printf("efb tablet=%d page-actions=%u dropped=%u\n",side-XPKIT_EFB_VIEW0,head,atomic_load(&e->actions_dropped));
            for(uint32_t a=head>XPKIT_EFB_ACTIONS?head-XPKIT_EFB_ACTIONS:0;a<head;++a)printf("  action: %s\n",e->action[a%XPKIT_EFB_ACTIONS]);
            if(efb_count[side-XPKIT_EFB_VIEW0]&&(!ready||seq<2||colored<2000))ok=0;
            continue;
        }
        if(!ready||seq<(side<2?10u:2u)||colored<2000) ok=0;
        if(side>=2 && side<4) {
            A380XXpKitNdProfile *p=&s->nd_profile[side-2];
            uint64_t captures=atomic_load(&p->snapshot_count),applies=atomic_load(&p->apply_count),skipped=atomic_load(&p->skipped_count);
            double age=seconds()-s->frame_time[side];
            printf("nd-profile side=%d captures=%llu applies=%llu skipped=%llu capture_mean_ms=%.3f apply_mean_ms=%.3f heartbeat_age=%.3f\n",
                side,(unsigned long long)captures,(unsigned long long)applies,(unsigned long long)skipped,
                captures?atomic_load(&p->snapshot_ns)/1e6/captures:0,applies?atomic_load(&p->apply_ns)/1e6/applies:0,age);
            if(age>.5)ok=0;
            if(getenv("A380X_ND_REQUIRE_CACHE") && (!skipped || captures>=applies))ok=0;
        }
        if(side>=4) {
            /* Static MFD pages are rendered once, with a cheap liveness
             * heartbeat. Continuous snapshots would recreate the cursor lag. */
            double age=seconds()-s->frame_time[side];
            printf("mfd-cache side=%d snapshots=%u heartbeat_age=%.3f\n",side,seq/2,age);
            if((side<6||!cycle)&&seq>4)ok=0;
            if(age>.5)ok=0;
        }
    }
    unsigned errors=atomic_load(&s->errors);
    printf("errors=%u message=%s\n",errors,s->error);
    if(errors) ok=0;
    if(cycle)for(int i=0;i<16;++i){printf("sd-page=%d captured=%d\n",i,captured[i]);if(!captured[i])ok=0;}
    kill(pid,SIGTERM); waitpid(pid,NULL,0); munmap(s,sizeof(*s)); unlink(filename);
    return ok?0:1;
}
