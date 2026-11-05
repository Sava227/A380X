/* SPDX-License-Identifier: GPL-3.0-or-later
 * Offline preview of the real packaged EFB page (xp-kit efb.html) at the tablet device resolution,
 * in the same macOS WebKit the aircraft's display helper uses. No network: file URL only.
 *
 *   clang -fobjc-arc -framework Cocoa -framework WebKit tools/render_efb_page.m -o render_efb_page
 *   render_efb_page <efb.html> <frame.json> <out.png> [javascript run after the frame]
 *
 * The frame should come from the native adapter (native/avionics/test_efb_adapter --frame ...).
 */
#import <Cocoa/Cocoa.h>
#import <WebKit/WebKit.h>

@interface Renderer : NSObject <WKScriptMessageHandler, WKNavigationDelegate>
@property(nonatomic) WKWebView *view;
@property(nonatomic) NSWindow *window;
@property(nonatomic) NSString *frame, *extra, *output;
@end

@implementation Renderer
- (void)userContentController:(WKUserContentController *)controller didReceiveScriptMessage:(WKScriptMessage *)message {
    (void)controller;
    NSDictionary *data = [message.body isKindOfClass:[NSDictionary class]] ? message.body : nil;
    if ([data[@"type"] isEqual:@"a380x-error"]) { fprintf(stderr, "page error: %s\n", [[data[@"text"] description] UTF8String]); exit(3); }
    if (![data[@"type"] isEqual:@"a380x-loaded"]) return;
    /* The frame stays available to the optional script as window.__frame (e.g. to replay touches). */
    NSString *script = [NSString stringWithFormat:@"window.__frame=%@;window.__a380Efb.apply(window.__frame,true);%@",
                        self.frame, self.extra.length ? self.extra : @"true"];
    [self.view evaluateJavaScript:script completionHandler:^(id result, NSError *error) {
        if (error) { fprintf(stderr, "script: %s\n", error.localizedDescription.UTF8String); exit(4); }
        if ([result isKindOfClass:[NSString class]]) printf("script result: %s\n", [result UTF8String]);
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 300 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{ [self capture]; });
    }];
}
- (void)capture {
    WKSnapshotConfiguration *config = [WKSnapshotConfiguration new];
    config.rect = CGRectMake(0, 0, 1440, 1000);
    config.snapshotWidth = @1440;
    config.afterScreenUpdates = YES;
    [self.view takeSnapshotWithConfiguration:config completionHandler:^(NSImage *image, NSError *error) {
        if (error || !image) { fprintf(stderr, "snapshot failed\n"); exit(5); }
        /* Same resolve as the display helper: the (Retina) snapshot into a 1440x1000 RGBA texture. */
        CGImageRef cg = [image CGImageForProposedRect:NULL context:nil hints:nil];
        CGColorSpaceRef color = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        CGContextRef ctx = CGBitmapContextCreate(NULL, 1440, 1000, 8, 1440 * 4, color, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
        CGColorSpaceRelease(color);
        CGContextSetInterpolationQuality(ctx, CGImageGetWidth(cg) == 1440 ? kCGInterpolationNone : kCGInterpolationHigh);
        CGContextDrawImage(ctx, CGRectMake(0, 0, 1440, 1000), cg);
        CGImageRef texture = CGBitmapContextCreateImage(ctx);
        NSBitmapImageRep *rep = [[NSBitmapImageRep alloc] initWithCGImage:texture];
        NSData *png = [rep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
        if (![png writeToFile:self.output atomically:YES]) { fprintf(stderr, "write failed\n"); exit(6); }
        printf("%s %zux%zu (snapshot %zux%zu)\n", self.output.UTF8String, CGImageGetWidth(texture), CGImageGetHeight(texture),
               CGImageGetWidth(cg), CGImageGetHeight(cg));
        CGImageRelease(texture); CGContextRelease(ctx);
        exit(0);
    }];
}
- (void)webView:(WKWebView *)webView didFailNavigation:(WKNavigation *)navigation withError:(NSError *)error {
    (void)webView; (void)navigation; fprintf(stderr, "navigation: %s\n", error.localizedDescription.UTF8String); exit(7);
}
@end

int main(int argc, const char **argv) {
    @autoreleasepool {
        if (argc < 4) { fprintf(stderr, "usage: render_efb_page efb.html frame.json out.png [js]\n"); return 2; }
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];
        Renderer *r = [Renderer new];
        NSError *error = nil;
        r.frame = [NSString stringWithContentsOfFile:[NSString stringWithUTF8String:argv[2]] encoding:NSUTF8StringEncoding error:&error];
        if (!r.frame) { fprintf(stderr, "frame unreadable\n"); return 2; }
        r.output = [NSString stringWithUTF8String:argv[3]];
        r.extra = argc > 4 ? [NSString stringWithUTF8String:argv[4]] : nil;
        WKWebViewConfiguration *configuration = [WKWebViewConfiguration new];
        configuration.websiteDataStore = [WKWebsiteDataStore nonPersistentDataStore];
        [configuration.userContentController addScriptMessageHandler:r name:@"a380"];
        r.view = [[WKWebView alloc] initWithFrame:NSMakeRect(0, 0, 1440, 1000) configuration:configuration];
        r.view.navigationDelegate = r;
        r.window = [[NSWindow alloc] initWithContentRect:NSMakeRect(-30000, -30000, 1440, 1000) styleMask:NSWindowStyleMaskBorderless
                                                 backing:NSBackingStoreBuffered defer:NO];
        r.window.contentView = r.view;
        [r.window orderBack:nil];
        NSString *page = [NSString stringWithUTF8String:argv[1]];
        if (!page.isAbsolutePath) page = [[NSFileManager defaultManager].currentDirectoryPath stringByAppendingPathComponent:page];
        page = page.stringByStandardizingPath;
        NSURLComponents *url = [NSURLComponents componentsWithURL:[NSURL fileURLWithPath:page] resolvingAgainstBaseURL:NO];
        url.query = @"tablet=0";
        [r.view loadFileURL:url.URL allowingReadAccessToURL:[NSURL fileURLWithPath:page.stringByDeletingLastPathComponent]];
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 20 * NSEC_PER_SEC), dispatch_get_main_queue(), ^{ fprintf(stderr, "timeout\n"); exit(8); });
        [NSApp run];
    }
    return 0;
}
