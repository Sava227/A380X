/* SPDX-License-Identifier: GPL-3.0-or-later
 * WebKit runs in this aircraft-local helper process, never on X-Plane's render thread.
 */
#import <Cocoa/Cocoa.h>
#import <WebKit/WebKit.h>
#import <Accelerate/Accelerate.h>
#import <IOSurface/IOSurface.h>
#import <objc/message.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include "a380x_xpkit_shared.h"

static A380XXpKitShared *shared;
static NSString *assetsDirectory, *simulatorDirectory;
static dispatch_queue_t airportQueue;
static NSMutableDictionary *airportResults;
static id displayActivity;
static double monotonic_seconds(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec * 1e-9;
}
static void maintain_display_activity(void) {
    BOOL fresh=atomic_load_explicit(&shared->state_seq,memory_order_acquire)>0 &&
        monotonic_seconds()-shared->state_time<1.5;
    if(fresh && !displayActivity) {
        /* Offscreen WebKit windows still serve an active cockpit. Keep App Nap
         * from treating this renderer as idle; ordinary system sleep remains
         * allowed. The token exists only while the aircraft supplies input. */
        displayActivity=[[NSProcessInfo processInfo] beginActivityWithOptions:NSActivityUserInitiatedAllowingIdleSystemSleep
            reason:@"Render active A380X cockpit displays"];
    } else if(!fresh && displayActivity) {
        [[NSProcessInfo processInfo] endActivity:displayActivity];displayActivity=nil;
    }
}
static void report_error(NSString *text) {
    snprintf(shared->error, sizeof(shared->error), "%s", text.UTF8String ?: "WebKit error");
    atomic_fetch_add_explicit(&shared->errors, 1, memory_order_release);
}
static uint64_t fnv1a(const void *bytes, size_t length) {
    const unsigned char *p = bytes; uint64_t hash = 1469598103934665603ULL;
    for (size_t i = 0; i < length; ++i) hash = (hash ^ p[i]) * 1099511628211ULL;
    return hash;
}
/* Exact-scale capture. WebKit paints a snapshot at the window's backing scale (2 on a Retina Mac), so a
 * 768x1024 PFD used to be painted as 1536x2048 (an ND at 1.5 as 2304x3072) and then resampled on the
 * CPU: four times the paint work plus a slow high-quality downscale per frame. With the page's device
 * scale factor set to the view's own capture scale, WebKit paints exactly the texture size and the
 * pixels only need a BGRA -> RGBA channel swap (vImage, straight into shared memory).
 * Measured with the real helper and live-moving PFD/ND content: ND capture 80 -> 14 ms. */
static BOOL exact_scale_capture(WKWebView *view, CGFloat scale) {
    SEL setter = NSSelectorFromString(@"_setOverrideDeviceScaleFactor:");
    if (![view respondsToSelector:setter]) return NO;
    ((void (*)(id, SEL, CGFloat))objc_msgSend)(view, setter, scale);
    return YES;
}
/* A same-size 32-bit snapshot -> dst (row stride dst_row) as RGBA (v8 pixel arrays) or BGRA (v9
 * 'BGRA' IOSurface). A snapshot already in the wanted byte order is copied row by row, otherwise its
 * channels are permuted (vImage). NO when the image needs scaling or has another layout. */
static BOOL copy_exact(CGImageRef cg, unsigned char *dst, size_t dst_row, int width, int height, BOOL bgra) {
    if (!cg || CGImageGetWidth(cg) != (size_t)width || CGImageGetHeight(cg) != (size_t)height ||
        CGImageGetBitsPerPixel(cg) != 32 || CGImageGetBitsPerComponent(cg) != 8) return NO;
    CGImageAlphaInfo alpha = CGImageGetAlphaInfo(cg);
    uint32_t order = CGImageGetBitmapInfo(cg) & kCGBitmapByteOrderMask;
    BOOL source_bgra;
    if (order == kCGBitmapByteOrder32Little && (alpha == kCGImageAlphaPremultipliedFirst ||
        alpha == kCGImageAlphaNoneSkipFirst || alpha == kCGImageAlphaFirst)) source_bgra = YES;   /* B,G,R,A bytes */
    else if ((order == kCGBitmapByteOrder32Big || order == kCGBitmapByteOrderDefault) &&
             (alpha == kCGImageAlphaPremultipliedLast || alpha == kCGImageAlphaNoneSkipLast ||
              alpha == kCGImageAlphaLast)) source_bgra = NO;                                     /* R,G,B,A bytes */
    else return NO;
    CGColorSpaceRef space = CGImageGetColorSpace(cg);
    if (space && CGColorSpaceGetModel(space) != kCGColorSpaceModelRGB) return NO;
    CFDataRef data = CGDataProviderCopyData(CGImageGetDataProvider(cg));
    if (!data) return NO;
    size_t source_row = CGImageGetBytesPerRow(cg);
    BOOL ok = (size_t)CFDataGetLength(data) >= source_row * (size_t)height;
    if (ok && source_bgra == bgra) {
        const unsigned char *from = CFDataGetBytePtr(data);
        for (int y = 0; y < height; ++y) memcpy(dst + (size_t)y * dst_row, from + (size_t)y * source_row, (size_t)width * 4);
    } else if (ok) {
        static const uint8_t swap[4] = {2, 1, 0, 3};             /* B,G,R,A <-> R,G,B,A */
        vImage_Buffer source = {(void *)CFDataGetBytePtr(data), (vImagePixelCount)height, (vImagePixelCount)width, source_row};
        vImage_Buffer target = {dst, (vImagePixelCount)height, (vImagePixelCount)width, dst_row};
        ok = vImagePermuteChannels_ARGB8888(&source, &target, swap, kvImageNoFlags) == kvImageNoError;
    }
    CFRelease(data);
    return ok;
}
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
/* v9: the plugin's global surface (IOSurfaceLookup by ID; deprecated API, present on every macOS 11+). */
static IOSurfaceRef lookup_surface(uint32_t identifier) { return identifier ? IOSurfaceLookup(identifier) : NULL; }
#pragma clang diagnostic pop

/* The native page controller owns validation, focus and all aircraft writes.
 * Only bounded drawing primitives cross this boundary. No JS FMC is started.
 */
static NSDictionary *mfd_frame(const A380XXpKitMfd *frame) {
    const MfdDrawList *dl=&frame->list;
    if(dl->overflowed || dl->cmd_count>MFD_DRAW_MAX_CMDS || dl->point_count>MFD_DRAW_MAX_POINTS ||
       dl->text_len>MFD_DRAW_MAX_TEXT || !dl->clip_count || dl->clip_count>MFD_DRAW_MAX_CLIPS) return nil;
    NSMutableArray *commands=[NSMutableArray arrayWithCapacity:dl->cmd_count];
    for(uint32_t i=0;i<dl->cmd_count;++i) {
        const MfdCmd *c=&dl->cmds[i];
        if(c->clip>=dl->clip_count || c->color>=MFD_COL_COUNT || c->kind>MFD_CMD_TEXT) return nil;
        const MfdRect *clip=&dl->clips[c->clip];
        for(int n=0;n<4;++n) if(!isfinite(c->a[n])) return nil;
        if(!isfinite(c->width)||!isfinite(c->size)||!isfinite(clip->x)||!isfinite(clip->y)||
           !isfinite(clip->w)||!isfinite(clip->h)) return nil;
        id payload=[NSNull null];
        if(c->kind==MFD_CMD_TEXT) {
            if(c->first>dl->text_len || c->count>dl->text_len-c->first) return nil;
            payload=[[NSString alloc] initWithBytes:dl->text+c->first length:c->count encoding:NSUTF8StringEncoding];
            if(!payload) return nil;
        } else if(c->kind>=MFD_CMD_POLYLINE) {
            if(c->first>dl->point_count || c->count>dl->point_count-c->first) return nil;
            NSMutableArray *points=[NSMutableArray arrayWithCapacity:c->count*2];
            for(uint32_t p=c->first;p<c->first+c->count;++p) {
                if(!isfinite(dl->points[p].x)||!isfinite(dl->points[p].y)) return nil;
                [points addObject:@(dl->points[p].x)];[points addObject:@(dl->points[p].y)];
            }
            payload=points;
        }
        [commands addObject:@[@(c->kind),@(mfd_color_rgb((MfdColor)c->color)),@(c->font),@(c->anchor),
            @(c->width),@(c->size),@(c->a[0]),@(c->a[1]),@(c->a[2]),@(c->a[3]),payload,
            @[@(clip->x),@(clip->y),@(clip->w),@(clip->h)],@(c->layer),@(c->closed)]];
    }
    NSString *uri=[[NSString alloc] initWithBytes:frame->uri length:strnlen(frame->uri,sizeof(frame->uri)) encoding:NSUTF8StringEncoding];
    return @{@"sequence":@(atomic_load(&frame->sequence)),@"powered":@(frame->powered!=0),
             @"uri":uri?:@"",@"commands":commands};
}

@interface A380XLibraryView : NSObject <WKScriptMessageHandler, WKNavigationDelegate>
@property(nonatomic) WKWebView *view;
@property(nonatomic) NSWindow *window;
@property(nonatomic) NSInteger side;
@property(nonatomic) BOOL loaded;
@property(nonatomic) BOOL busy;
@property(nonatomic) NSTimer *timer;
@property(nonatomic) BOOL diagnosed;
@property(nonatomic) uint32_t lastMfdSequence;
@property(nonatomic) BOOL lastInputValid;
@property(nonatomic) BOOL exactScale;           /* WebKit paints at the view's capture scale */
@property(nonatomic) uint64_t lastStateHash;    /* PFD/ND: state already applied and captured */
@property(nonatomic) CGContextRef scaleContext; /* fallback resampler, drawing into shared memory */
@property(nonatomic) uint32_t lastSurfaceMode;  /* v9 surface_mode seen by the last tick */
@property(nonatomic) BOOL forceCapture;         /* capture the next tick even if the page is unchanged */
@end

@implementation A380XLibraryView {
    IOSurfaceRef _surface[XPKIT_SURFACES];      /* the plugin's surfaces of this view (v9) */
    uint32_t _surfaceId[XPKIT_SURFACES];
    BOOL _surfaceReported;
}
- (void)dealloc {
    if (self.scaleContext) CGContextRelease(self.scaleContext);
    for (int i = 0; i < XPKIT_SURFACES; ++i) if (_surface[i]) CFRelease(_surface[i]);
}
/* The plugin's surfaces for this view, looked up again whenever an ID changes; NO unless all exist
 * with this view's size and 4 bytes per element. */
- (BOOL)surfacesReady {
    int side = (int)self.side, width = xpkit_view_width(side), height = xpkit_view_height(side);
    BOOL ready = YES;
    for (int i = 0; i < XPKIT_SURFACES; ++i) {
        uint32_t identifier = atomic_load_explicit(&shared->surface_id[side][i], memory_order_acquire);
        if (identifier != _surfaceId[i] || (identifier && !_surface[i])) {
            if (_surface[i]) CFRelease(_surface[i]);
            _surface[i] = lookup_surface(identifier);
            _surfaceId[i] = identifier;
            _surfaceReported = NO;
        }
        IOSurfaceRef s = _surface[i];
        if (!s || IOSurfaceGetWidth(s) != (size_t)width || IOSurfaceGetHeight(s) != (size_t)height ||
            IOSurfaceGetBytesPerElement(s) != 4) ready = NO;
    }
    if (!ready && !_surfaceReported) {
        _surfaceReported = YES;
        report_error([NSString stringWithFormat:@"Display surfaces of view %d unavailable; pixel frames", side]);
    }
    return ready;
}
/* v9 zero-copy: the frame goes into the surface after the presented one; the seqlock publishes it. */
- (BOOL)storeSurface:(CGImageRef)cg {
    int side = (int)self.side, width = xpkit_view_width(side), height = xpkit_view_height(side);
    uint32_t front = atomic_load_explicit(&shared->surface_front[side], memory_order_acquire);
    uint32_t back = front < XPKIT_SURFACES ? (front + 1) % XPKIT_SURFACES : 0;
    IOSurfaceRef s = _surface[back];
    uint32_t seed = 0;
    if (IOSurfaceLock(s, 0, &seed) != kIOReturnSuccess) return NO;
    unsigned char *base = IOSurfaceGetBaseAddress(s);
    size_t row = IOSurfaceGetBytesPerRow(s);
    BOOL stored = copy_exact(cg, base, row, width, height, YES);
    if (!stored) {   /* not the capture size (no exact-scale painting): resample into the surface */
        CGColorSpaceRef color = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        CGContextRef context = CGBitmapContextCreate(base, width, height, 8, row, color,
                                                     kCGImageAlphaPremultipliedFirst | kCGBitmapByteOrder32Little);
        CGColorSpaceRelease(color);
        if (context) {
            CGContextSetBlendMode(context, kCGBlendModeCopy);
            CGContextSetInterpolationQuality(context, kCGInterpolationHigh);
            CGContextDrawImage(context, CGRectMake(0, 0, width, height), cg);
            CGContextRelease(context);
            stored = YES;
        }
    }
    IOSurfaceUnlock(s, 0, &seed);
    if (!stored) return NO;
    uint32_t sequence = atomic_load_explicit(&shared->frame_seq[side], memory_order_relaxed);
    if (sequence & 1) ++sequence;   /* left odd by a helper that died while publishing */
    atomic_store_explicit(&shared->frame_seq[side], sequence + 1, memory_order_release);
    atomic_store_explicit(&shared->surface_front[side], back, memory_order_release);
    shared->frame_time[side] = monotonic_seconds();
    atomic_store_explicit(&shared->frame_seq[side], sequence + 2, memory_order_release);
    return YES;
}
/* Store a snapshot as this view's RGBA frame (seqlock: odd while writing). */
- (BOOL)storeImage:(CGImageRef)cg {
    int side = (int)self.side, width = xpkit_view_width(side), height = xpkit_view_height(side);
    unsigned char *pixels = xpkit_view_pixels(shared, side);
    if (!cg) return NO;
    if (atomic_load_explicit(&shared->surface_mode[side], memory_order_acquire) == 1 && [self surfacesReady] &&
        [self storeSurface:cg]) return YES;
    BOOL exact = CGImageGetWidth(cg) == (size_t)width && CGImageGetHeight(cg) == (size_t)height;
    if (!exact && !self.scaleContext) {
        CGColorSpaceRef color = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        self.scaleContext = CGBitmapContextCreate(pixels, width, height, 8, (size_t)width * 4, color,
                                                  kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
        CGColorSpaceRelease(color);
        if (!self.scaleContext) { report_error(@"Unable to allocate display bitmap"); return NO; }
        CGContextSetBlendMode(self.scaleContext, kCGBlendModeCopy);
        CGContextSetInterpolationQuality(self.scaleContext, kCGInterpolationHigh);
    }
    uint32_t sequence = atomic_load_explicit(&shared->frame_seq[side], memory_order_relaxed);
    if (sequence & 1) ++sequence;   /* left odd by a helper that died while publishing */
    atomic_store_explicit(&shared->frame_seq[side], sequence + 1, memory_order_release);
    BOOL stored = exact && copy_exact(cg, pixels, (size_t)width * 4, width, height, NO);
    if (!stored && self.scaleContext) {
        CGContextDrawImage(self.scaleContext, CGRectMake(0, 0, width, height), cg);
        stored = YES;
    }
    if (stored) {
        atomic_store_explicit(&shared->surface_front[side], XPKIT_SURFACE_PIXELS, memory_order_release);
        shared->frame_time[side] = monotonic_seconds();
    }
    atomic_store_explicit(&shared->frame_seq[side], sequence + 2, memory_order_release);
    return stored;
}
/* Snapshot the page and store it; done() runs on the main thread. */
- (void)captureThen:(void (^)(BOOL stored))done {
    int side = (int)self.side;
    WKSnapshotConfiguration *config = [WKSnapshotConfiguration new];
    /* v7+: the page keeps its CSS layout (768x1024, EFB 1440x1000); the capture is scale times that. */
    config.rect = CGRectMake(0, 0, xpkit_view_css_width(side), xpkit_view_css_height(side));
    config.snapshotWidth = @(self.exactScale ? xpkit_view_css_width(side) : xpkit_view_width(side));
    config.afterScreenUpdates = YES;
    [self.view takeSnapshotWithConfiguration:config completionHandler:^(NSImage *image, NSError *captureError) {
        if (captureError || !image) {
            report_error(captureError.localizedDescription ?: @"No WebKit snapshot");
            done(NO); return;
        }
        done([self storeImage:[image CGImageForProposedRect:NULL context:nil hints:nil]]);
    }];
}
- (void)userContentController:(WKUserContentController *)controller didReceiveScriptMessage:(WKScriptMessage *)message {
    (void)controller;
    if (![message.body isKindOfClass:[NSDictionary class]]) return;
    NSDictionary *data = message.body;
    if ([data[@"type"] isEqual:@"a380x-loaded"]) {
        self.loaded = [data[@"ok"] boolValue];
        atomic_store_explicit(&shared->ready[self.side], self.loaded, memory_order_release);
    } else if ([data[@"type"] isEqual:@"a380x-error"]) report_error([data[@"text"] description]);
    else if ([data[@"type"] isEqual:@"a380x-efb-action"] && self.side>=XPKIT_EFB_VIEW0) {
        /* Page -> plugin: a bounded printable "<id> <verb> [args]" string; the native EFB validates
         * it again. Single producer (this main-thread handler), single consumer (the plugin). */
        A380XXpKitEfb *efb=&shared->efb[self.side-XPKIT_EFB_VIEW0];
        NSNumber *identifier=data[@"id"];NSString *action=data[@"action"];
        if(![identifier isKindOfClass:[NSNumber class]]||![action isKindOfClass:[NSString class]])return;
        NSString *line=[NSString stringWithFormat:@"%lu %@",(unsigned long)identifier.unsignedLongValue,action];
        const char *bytes=line.UTF8String;
        if(!bytes||strlen(bytes)>=XPKIT_EFB_ACTION_LEN)return;
        for(const char *p=bytes;*p;++p)if((unsigned char)*p<32||(unsigned char)*p>126)return;
        uint32_t head=atomic_load_explicit(&efb->action_head,memory_order_relaxed);
        uint32_t tail=atomic_load_explicit(&efb->action_tail,memory_order_acquire);
        if(head-tail>=XPKIT_EFB_ACTIONS){atomic_fetch_add(&efb->actions_dropped,1);return;}
        memset(efb->action[head%XPKIT_EFB_ACTIONS],0,XPKIT_EFB_ACTION_LEN);
        memcpy(efb->action[head%XPKIT_EFB_ACTIONS],bytes,strlen(bytes));
        atomic_store_explicit(&efb->action_head,head+1,memory_order_release);
    }
    else if ([data[@"type"] isEqual:@"a380x-nd-command"] && self.side>=2 && self.side<4) {
        int delta=[data[@"action"] isEqual:@"zoom_in"]?-1:[data[@"action"] isEqual:@"zoom_out"]?1:0;
        atomic_fetch_add(&shared->zoom_delta[self.side-2],delta);
    } else if ([data[@"type"] isEqual:@"a380x-nd-status"] && self.side>=2 && self.side<4) {
        NSArray *values=data[@"values"];
        if([values isKindOfClass:[NSArray class]] && values.count==8)
            for(int n=0;n<8;++n)atomic_store(&shared->nd_status[self.side-2][n],[values[n] unsignedIntValue]);
    } else if ([data[@"type"] isEqual:@"a380x-airport"] && self.side>=2 && self.side<4) {
        NSString *icao=data[@"icao"];NSNumber *request=data[@"id"];
        NSCharacterSet *invalid=[[NSCharacterSet characterSetWithCharactersInString:@"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"] invertedSet];
        if(![icao isKindOfClass:[NSString class]]||icao.length<2||icao.length>16||[icao rangeOfCharacterFromSet:invalid].location!=NSNotFound||![request isKindOfClass:[NSNumber class]])return;
        dispatch_async(airportQueue, ^{
            NSString *result=airportResults[icao];NSString *problem=nil;
            if(!result) {
                NSTask *task=[NSTask new];task.executableURL=[NSURL fileURLWithPath:@"/usr/bin/python3"];
                task.arguments=@[[assetsDirectory stringByAppendingPathComponent:@"airport_worker.py"],simulatorDirectory,icao];
                NSPipe *out=[NSPipe pipe],*err=[NSPipe pipe];task.standardOutput=out;task.standardError=err;
                NSError *launch=nil;
                if([task launchAndReturnError:&launch]) {
                    NSData *bytes=[out.fileHandleForReading readDataToEndOfFile];
                    NSData *errorBytes=[err.fileHandleForReading readDataToEndOfFile];[task waitUntilExit];
                    if(task.terminationStatus==0&&bytes.length<16*1024*1024&&[NSJSONSerialization JSONObjectWithData:bytes options:0 error:nil]) {
                        result=[[NSString alloc] initWithData:bytes encoding:NSUTF8StringEncoding];
                        if(airportResults.count>=4)[airportResults removeObjectForKey:airportResults.allKeys.firstObject];
                        airportResults[icao]=result;
                    } else problem=[[NSString alloc] initWithData:errorBytes encoding:NSUTF8StringEncoding]?:@"Airport scenery unavailable";
                } else problem=launch.localizedDescription;
            }
            NSData *quoted=[NSJSONSerialization dataWithJSONObject:@[problem?:[NSNull null]] options:0 error:nil];
            NSString *errorJson=[[NSString alloc] initWithData:quoted encoding:NSUTF8StringEncoding];
            NSString *script=[NSString stringWithFormat:@"window.__a380Nd.airportResult(%@,(%@)[0],%@);true",request,errorJson,result?:@"null"];
            dispatch_async(dispatch_get_main_queue(), ^{[self.view evaluateJavaScript:script completionHandler:nil];});
        });
    }
}
- (void)webView:(WKWebView *)webView didFailNavigation:(WKNavigation *)navigation withError:(NSError *)error {
    (void)webView; (void)navigation; report_error(error.localizedDescription);
}
- (void)webViewWebContentProcessDidTerminate:(WKWebView *)webView {
    (void)webView; self.loaded = NO;
    atomic_store_explicit(&shared->ready[self.side], 0, memory_order_release);
    report_error(@"WebKit display process terminated");
}
- (void)tick {
    if (getppid() == 1) { [NSApp terminate:nil]; return; }
    maintain_display_activity();
    if (!self.loaded || self.busy) return;
    /* v9: the plugin switched this view between surfaces and pixel arrays (zero-copy probe passed or
     * failed): capture again even if the page did not change, so its last frame exists in the new place. */
    uint32_t surfaceMode = atomic_load_explicit(&shared->surface_mode[self.side], memory_order_acquire);
    if (surfaceMode != self.lastSurfaceMode) {
        self.lastSurfaceMode = surfaceMode; self.lastStateHash = 0; self.lastMfdSequence = 0; self.forceCapture = YES;
    }
    uint32_t before = atomic_load_explicit(&shared->state_seq, memory_order_acquire);
    if ((before & 1) || !before || !shared->state_size || shared->state_size >= XPKIT_JSON_CAP) return;
    NSData *data = self.side<4?[NSData dataWithBytes:shared->state length:shared->state_size]:nil;
    double sent = shared->state_time;
    atomic_thread_fence(memory_order_acquire);
    if (atomic_load_explicit(&shared->state_seq, memory_order_acquire) != before) return;
    BOOL inputValid=monotonic_seconds()-sent<1.5;
    /* PFD/ND: the plugin republishes every frame; identical content (paused sim, a timer firing between
     * two publications) keeps the completed texture with a heartbeat instead of a repaint. */
    uint64_t stateHash=self.side<4&&inputValid?fnv1a(data.bytes,data.length):0;
    if(self.side<4 && !self.forceCapture && stateHash==self.lastStateHash && inputValid==self.lastInputValid &&
       atomic_load_explicit(&shared->frame_seq[self.side],memory_order_acquire)) {
        shared->frame_time[self.side]=monotonic_seconds();
        return;
    }
    BOOL efbView=self.side>=XPKIT_EFB_VIEW0;
    if(efbView) {
        [self tickEfb:inputValid];
        return;
    }
    if(self.side>=4) {
        uint32_t seq=self.side<6?atomic_load_explicit(&shared->mfd[self.side-4].sequence,memory_order_acquire):atomic_load_explicit(&shared->sd.sequence,memory_order_acquire);
        if(!seq || (seq&1))return;
        if(!self.forceCapture && seq==self.lastMfdSequence && inputValid==self.lastInputValid &&
           atomic_load_explicit(&shared->frame_seq[self.side],memory_order_acquire)) {
            /* The texture is unchanged, not dead. Heartbeat without snapshots
             * or frame-sequence bumps; a stopped helper cannot renew this. */
            shared->frame_time[self.side]=monotonic_seconds();
            return;
        }
    }
    NSString *json = self.side>=4?@"{}":inputValid ? [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding] : @"{\"ok\":false,\"mode\":\"native\"}";
    if (!json) return;
    self.busy = YES;
    BOOL nd=self.side>=2 && self.side<4;
    double applyStarted=monotonic_seconds();
    NSString *script = self.side>=4?[NSString stringWithFormat:@"window.__a380xTrial.apply({ok:%@});true",inputValid?@"true":@"false"]:
        [NSString stringWithFormat:@"window.__a380xTrial.apply(%@); window.__a380NativeFrame(); true", json];
    if(nd) script=[script stringByAppendingString:@";window.__a380Nd?.needsSnapshot?.()!==false"];
    uint32_t mfdSequence=0;
    if(self.side>=4 && self.side<6) {
        A380XXpKitMfd *source=&shared->mfd[self.side-4];
        uint32_t seq=atomic_load_explicit(&source->sequence,memory_order_acquire);
        if(seq && !(seq&1) && seq!=self.lastMfdSequence) {
            NSData *copy=[NSData dataWithBytes:source length:sizeof(*source)];
            atomic_thread_fence(memory_order_acquire);
            if(atomic_load_explicit(&source->sequence,memory_order_acquire)==seq) {
                NSDictionary *frame=mfd_frame(copy.bytes);
                NSData *encoded=frame?[NSJSONSerialization dataWithJSONObject:frame options:0 error:nil]:nil;
                if(encoded) {
                    script=[script stringByAppendingFormat:@";window.__a380Mfd.apply(%@);true",[[NSString alloc] initWithData:encoded encoding:NSUTF8StringEncoding]];
                    mfdSequence=seq;
                } else report_error(@"Invalid native MFD display list");
            }
        }
    }
    if(self.side==6) {
        A380XXpKitSd *source=&shared->sd;
        uint32_t seq=atomic_load_explicit(&source->sequence,memory_order_acquire);
        if(seq&&!(seq&1)&&source->size>0&&source->size<XPKIT_SD_CAP) {
            NSData *copy=[NSData dataWithBytes:source->json length:source->size];
            atomic_thread_fence(memory_order_acquire);
            if(atomic_load_explicit(&source->sequence,memory_order_acquire)==seq) {
                NSString *encoded=inputValid?[[NSString alloc] initWithData:copy encoding:NSUTF8StringEncoding]:@"{\"powered\":false,\"page\":0,\"vars\":{}}";
                if(encoded&&[NSJSONSerialization JSONObjectWithData:copy options:0 error:nil]) {
                    script=[NSString stringWithFormat:@"window.__a380Sd.apply(%@);true",encoded];mfdSequence=seq;
                } else report_error(@"Invalid native ECAM payload");
            }
        }
    }
    [self.view evaluateJavaScript:script completionHandler:^(id result, NSError *error) {
        if (error) { report_error(error.localizedDescription); self.busy = NO; return; }
        if(nd) {
            A380XXpKitNdProfile *profile=&shared->nd_profile[self.side-2];
            atomic_fetch_add_explicit(&profile->apply_count,1,memory_order_relaxed);
            atomic_fetch_add_explicit(&profile->apply_ns,(uint64_t)((monotonic_seconds()-applyStarted)*1e9),memory_order_relaxed);
            if([result isKindOfClass:[NSNumber class]] && ![result boolValue] && !self.forceCapture &&
               inputValid==self.lastInputValid && atomic_load_explicit(&shared->frame_seq[self.side],memory_order_acquire)) {
                /* Cursor-only/unchanged content keeps its completed texture and
                 * liveness heartbeat. Actual pointer paint is native geometry. */
                shared->frame_time[self.side]=monotonic_seconds();
                atomic_fetch_add_explicit(&profile->skipped_count,1,memory_order_relaxed);
                self.lastStateHash=stateHash;
                self.busy=NO; return;
            }
        }
        if (!self.diagnosed && getenv("A380X_DISPLAY_DEBUG")) {
            self.diagnosed = YES;
            [self.view evaluateJavaScript:@"JSON.stringify({frames:document.querySelector('a380x-pfd,a380x-nd')?.frameCount,visible:document.visibilityState,body:document.body.innerText})" completionHandler:^(id value, NSError *problem) {
                fprintf(stderr, "display %ld: %s %s\n", (long)self.side, [value description].UTF8String, problem.localizedDescription.UTF8String ?: "");
            }];
        }
        double captureStarted=monotonic_seconds();
        [self captureThen:^(BOOL stored) {
            if (stored) {
                self.forceCapture=NO;
                if(mfdSequence)self.lastMfdSequence=mfdSequence;
                self.lastInputValid=inputValid;
                if(self.side<4)self.lastStateHash=stateHash;
                if(nd) {
                    A380XXpKitNdProfile *profile=&shared->nd_profile[self.side-2];
                    atomic_fetch_add_explicit(&profile->snapshot_count,1,memory_order_relaxed);
                    atomic_fetch_add_explicit(&profile->snapshot_ns,(uint64_t)((monotonic_seconds()-captureStarted)*1e9),memory_order_relaxed);
                }
            }
            self.busy = NO;
        }];
    }];
}
/* EFB tablet: apply a new native frame (or a stale-input notice), then capture only when the page
 * reports a visual change (__a380Efb.apply/idle return true). An unchanged page keeps its texture
 * alive with a heartbeat, like the event-driven MFD/SD views. */
- (void)tickEfb:(BOOL)inputValid {
    A380XXpKitEfb *source=&shared->efb[self.side-XPKIT_EFB_VIEW0];
    uint32_t seq=atomic_load_explicit(&source->sequence,memory_order_acquire);
    NSString *script=nil;uint32_t applied=0;
    if(seq&&!(seq&1)&&(seq!=self.lastMfdSequence||inputValid!=self.lastInputValid)&&source->size>0&&source->size<XPKIT_EFB_CAP) {
        NSData *copy=[NSData dataWithBytes:source->json length:source->size];
        atomic_thread_fence(memory_order_acquire);
        if(atomic_load_explicit(&source->sequence,memory_order_acquire)!=seq)return;
        NSString *encoded=[[NSString alloc] initWithData:copy encoding:NSUTF8StringEncoding];
        if(!encoded||![NSJSONSerialization JSONObjectWithData:copy options:0 error:nil]) {report_error(@"Invalid native EFB frame");return;}
        script=[NSString stringWithFormat:@"window.__a380Efb.apply(%@,%@)",encoded,inputValid?@"true":@"false"];
        applied=seq;
    } else script=@"window.__a380Efb.idle()";
    self.busy=YES;
    [self.view evaluateJavaScript:script completionHandler:^(id result, NSError *error) {
        if(error){report_error(error.localizedDescription);self.busy=NO;return;}
        if(applied){self.lastMfdSequence=applied;self.lastInputValid=inputValid;}
        BOOL changed=[result isKindOfClass:[NSNumber class]]&&[result boolValue];
        if(!changed&&!self.forceCapture&&atomic_load_explicit(&shared->frame_seq[self.side],memory_order_acquire)) {
            shared->frame_time[self.side]=monotonic_seconds();self.busy=NO;return;
        }
        [self captureThen:^(BOOL stored) { if(stored)self.forceCapture=NO; self.busy = NO; }];
    }];
}
@end

int main(int argc, const char **argv) {
    @autoreleasepool {
        if (argc != 4) { fprintf(stderr, "usage: A380XDisplayHost shared-file assets-directory X-Plane-directory\n"); return 2; }
        int fd = open(argv[1], O_RDWR | O_NOFOLLOW);
        struct stat st;
        if (fd < 0 || fstat(fd, &st) || st.st_size != sizeof(A380XXpKitShared)) return 3;
        shared = mmap(NULL, sizeof(*shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (shared == MAP_FAILED || shared->magic != XPKIT_MAGIC || shared->version != XPKIT_VERSION) return 4;
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];
        NSMutableArray *views = [NSMutableArray new];
        NSString *root = [NSString stringWithUTF8String:argv[2]];
        assetsDirectory=root;simulatorDirectory=[NSString stringWithUTF8String:argv[3]];
        airportQueue=dispatch_queue_create("org.a380x.scenery-reader",DISPATCH_QUEUE_SERIAL);
        airportResults=[NSMutableDictionary new];
        for (NSInteger side = 0; side < XPKIT_VIEWS; side++) {
            A380XLibraryView *owner = [A380XLibraryView new];
            owner.side = side;
            WKWebViewConfiguration *configuration = [WKWebViewConfiguration new];
            configuration.websiteDataStore = [WKWebsiteDataStore nonPersistentDataStore];
            [configuration.userContentController addScriptMessageHandler:owner name:@"a380"];
            int width=xpkit_view_css_width((int)side),height=xpkit_view_css_height((int)side);   /* CSS size */
            owner.view = [[WKWebView alloc] initWithFrame:NSMakeRect(0, 0, width, height) configuration:configuration];
            owner.view.navigationDelegate = owner;
            owner.exactScale = exact_scale_capture(owner.view, xpkit_view_scale((int)side));
            owner.window = [[NSWindow alloc] initWithContentRect:NSMakeRect(-20000 - side * 2000, -20000, width, height)
                styleMask:NSWindowStyleMaskBorderless backing:NSBackingStoreBuffered defer:NO];
            owner.window.releasedWhenClosed = NO;
            owner.window.ignoresMouseEvents = YES;
            owner.window.contentView = owner.view;
            [owner.window orderBack:nil];
            NSURLComponents *url = [NSURLComponents componentsWithURL:[NSURL fileURLWithPath:[root stringByAppendingPathComponent:side<2?@"pfd.html":side<4?@"nd.html":side<6?@"mfd.html":side<XPKIT_EFB_VIEW0?@"sd.html":@"efb.html"]] resolvingAgainstBaseURL:NO];
            url.query = side == 0 ? @"index=1&duID=0" : side==1?@"index=2&duID=3":side==2?@"index=1&duID=1":side==3?@"index=2&duID=4":side==4?@"index=1&duID=2":side<XPKIT_EFB_VIEW0?@"index=2&duID=5":side==XPKIT_EFB_VIEW0?@"tablet=0":@"tablet=1";
            [owner.view loadFileURL:url.URL allowingReadAccessToURL:[NSURL fileURLWithPath:root]];
            /* PFDs and NDs move continuously (attitude, tapes, heading, map): polled at 60 Hz and
             * captured whenever the plugin's state changed and the previous capture has finished, so
             * their rate follows the simulator's publication rate. Event-driven views keep 20 Hz. */
            owner.timer = [NSTimer scheduledTimerWithTimeInterval:side<4?1.0/60.0:0.05 target:owner selector:@selector(tick) userInfo:nil repeats:YES];
            owner.timer.tolerance = side<4?0.002:0.01;
            [views addObject:owner];
        }
        [NSApp run];
        (void)views;
    }
    return 0;
}
