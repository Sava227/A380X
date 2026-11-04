/* SPDX-License-Identifier: GPL-3.0-or-later
 * Aircraft-local Windows WebView2 renderer. Native C still owns all avionics,
 * commands, keyboard capture and flight-plan validation. No localhost server.
 */
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>
#include <shlobj.h>
#include <wincodec.h>
#include <WebView2.h>
#include "webview2_mingw_guids.h"
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include "a380x_xpkit_wire.hpp"

static A380XXpKitShared *shared;
static ICoreWebView2Environment *environment;
static IWICImagingFactory *imaging;
static HANDLE parent_process;
static HWND message_window;
static std::wstring assets,simulator,plugin;
static std::atomic<bool> closing{false};
static std::vector<std::thread> workers;
static std::mutex airport_mutex;
static std::map<std::string,XpJson> airports;
static const std::wstring origin=L"https://a380x.invalid/";
static double seconds() {LARGE_INTEGER a,b;QueryPerformanceCounter(&a);QueryPerformanceFrequency(&b);return double(a.QuadPart)/double(b.QuadPart);}
static std::wstring wide(const std::string &s) {
    if(s.empty())return {};
    int n=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,s.data(),(int)s.size(),nullptr,0);
    if(!n)throw std::runtime_error("Invalid UTF-8 in native renderer input");
    std::wstring out(n,0);MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,s.data(),(int)s.size(),out.data(),n);return out;
}
static std::string utf8(const wchar_t *s) {
    if(!s)return {};
    int n=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,s,-1,nullptr,0,nullptr,nullptr);
    if(!n)throw std::runtime_error("Invalid Windows text");
    std::string out(n,0);WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,s,-1,out.data(),n,nullptr,nullptr);out.pop_back();return out;
}
static void error(const std::string &text) {
    if(shared){snprintf(shared->error,sizeof(shared->error),"%s",text.c_str());shared->errors.fetch_add(1);}
    static std::mutex log_mutex;static std::string last_error;
    std::lock_guard<std::mutex> lock(log_mutex);
    if(text==last_error)return;
    last_error=text;
    std::wstring log=plugin+L"\\display-host.log";
    FILE *file=_wfopen(log.c_str(),L"ab");if(file){fprintf(file,"A380X Windows display: %s\n",text.c_str());fclose(file);}
}
static bool okay(HRESULT hr,const char *where) {
    if(SUCCEEDED(hr))return true;
    char text[256];snprintf(text,sizeof(text),"%s failed (HRESULT 0x%08lx)",where,(unsigned long)hr);error(text);return false;
}
template<class T>struct Release {void operator()(T *p)const{if(p)p->Release();}};
template<class T>using Com=std::unique_ptr<T,Release<T>>;
template<class I,class...Args>struct Handler final:I {
    std::atomic<ULONG> refs{1};std::function<HRESULT(Args...)> fn;
    explicit Handler(std::function<HRESULT(Args...)> f):fn(std::move(f)){}
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id,void **out) override {
        if(!out)return E_POINTER;*out=nullptr;
        if(IsEqualIID(id,IID_IUnknown)||IsEqualIID(id,__uuidof(I))){*out=static_cast<I*>(this);AddRef();return S_OK;}return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override{return ++refs;}
    ULONG STDMETHODCALLTYPE Release() override{ULONG n=--refs;if(!n)delete this;return n;}
    HRESULT STDMETHODCALLTYPE Invoke(Args... args) override{
        try{return fn(args...);}catch(const std::exception &e){error(e.what());return E_FAIL;}
    }
};
template<class I,class...Args,class F>Com<I> callback(F fn){return Com<I>(new Handler<I,Args...>(fn));}
struct Options final:ICoreWebView2EnvironmentOptions {
    std::atomic<ULONG> refs{1};
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id,void **out)override{
        if(!out)return E_POINTER;*out=nullptr;
        if(IsEqualIID(id,IID_IUnknown)||IsEqualIID(id,__uuidof(ICoreWebView2EnvironmentOptions))){*out=this;AddRef();return S_OK;}return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef()override{return ++refs;}
    ULONG STDMETHODCALLTYPE Release()override{ULONG n=--refs;if(!n)delete this;return n;}
    HRESULT text(LPCWSTR source,LPWSTR *out){if(!out)return E_POINTER;size_t n=(wcslen(source)+1)*sizeof(wchar_t);*out=(LPWSTR)CoTaskMemAlloc(n);if(!*out)return E_OUTOFMEMORY;memcpy(*out,source,n);return S_OK;}
    HRESULT STDMETHODCALLTYPE get_AdditionalBrowserArguments(LPWSTR *out)override{return text(L"--disable-background-timer-throttling --disable-renderer-backgrounding --disable-backgrounding-occluded-windows --disable-features=CalculateNativeWinOcclusion",out);}
    HRESULT STDMETHODCALLTYPE put_AdditionalBrowserArguments(LPCWSTR)override{return S_OK;}
    HRESULT STDMETHODCALLTYPE get_Language(LPWSTR *out)override{return text(L"en-US",out);}
    HRESULT STDMETHODCALLTYPE put_Language(LPCWSTR)override{return S_OK;}
    HRESULT STDMETHODCALLTYPE get_TargetCompatibleBrowserVersion(LPWSTR *out)override{return text(L"100.0.1185.39",out);}
    HRESULT STDMETHODCALLTYPE put_TargetCompatibleBrowserVersion(LPCWSTR)override{return S_OK;}
    HRESULT STDMETHODCALLTYPE get_AllowSingleSignOnUsingOSPrimaryAccount(BOOL *out)override{if(!out)return E_POINTER;*out=FALSE;return S_OK;}
    HRESULT STDMETHODCALLTYPE put_AllowSingleSignOnUsingOSPrimaryAccount(BOOL)override{return S_OK;}
};
struct View;
struct AirportResult{View *view;int request;XpJson data;std::string problem;};
static void airport(View *view,int request,const std::string &icao);
struct View {
    int side;HWND window=nullptr;ICoreWebView2Controller *controller=nullptr;ICoreWebView2 *web=nullptr;
    bool loaded=false,busy=false,last_valid=false;uint32_t last_sequence=0;
    std::vector<unsigned char> bitmap;
    explicit View(int i):side(i),bitmap(xpkit_view_bytes(i)){}
    ~View(){if(controller){controller->Close();controller->Release();}if(web)web->Release();if(window)DestroyWindow(window);}
    HRESULT script(const std::string &source,ICoreWebView2ExecuteScriptCompletedHandler *done=nullptr){return web->ExecuteScript(wide(source).c_str(),done);}
    void message(ICoreWebView2WebMessageReceivedEventArgs *args) {
        LPWSTR source=nullptr,text=nullptr;args->get_Source(&source);
        bool trusted=source&&std::wstring(source).rfind(origin,0)==0;CoTaskMemFree(source);
        if(!trusted||FAILED(args->TryGetWebMessageAsString(&text)))return;
        std::string value=utf8(text);CoTaskMemFree(text);if(value.size()>65536)return;
        XpJson data=XpJson::parse(value);std::string type=data.value("type","");
        if(type=="a380x-loaded"){loaded=data.value("ok",false);shared->ready[side].store(loaded);}
        else if(type=="a380x-error")error(data.value("text","Display script error"));
        else if(side>=2&&side<4){
            if(type=="a380x-nd-command"){auto action=data.value("action","");shared->zoom_delta[side-2].fetch_add(action=="zoom_in"?-1:action=="zoom_out"?1:0);}
            else if(type=="a380x-nd-status"&&data.contains("values")&&data["values"].is_array()&&data["values"].size()==8)
                for(int n=0;n<8;++n)shared->nd_status[side-2][n].store(data["values"][n].get<uint32_t>());
            else if(type=="a380x-airport"&&data.contains("id")&&data["id"].is_number_integer()){
                std::string code=data.value("icao","");
                if(code.size()>=2&&code.size()<=16&&code.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos)airport(this,data["id"].get<int>(),code);
            }
        }
    }
    void capture(uint32_t page_sequence,bool valid,double began) {
        IStream *raw=nullptr;if(!okay(CreateStreamOnHGlobal(nullptr,TRUE,&raw),"Create capture stream")){busy=false;return;}
        // Both the callback and this call own a stream reference until capture completes.
        auto stream=std::shared_ptr<IStream>(raw,[](IStream *p){p->Release();});
        auto done=callback<ICoreWebView2CapturePreviewCompletedHandler,HRESULT>([this,stream,page_sequence,valid,began](HRESULT hr){
            if(!okay(hr,"CapturePreview")){busy=false;return S_OK;}
            LARGE_INTEGER zero{};stream->Seek(zero,STREAM_SEEK_SET,nullptr);
            IWICBitmapDecoder *d=nullptr;IWICBitmapFrameDecode *f=nullptr;IWICBitmapScaler *s=nullptr;IWICFormatConverter *c=nullptr;
            hr=imaging->CreateDecoderFromStream(stream.get(),nullptr,WICDecodeMetadataCacheOnLoad,&d);Com<IWICBitmapDecoder> decoder(d);
            if(SUCCEEDED(hr))hr=d->GetFrame(0,&f);Com<IWICBitmapFrameDecode> frame(f);
            if(SUCCEEDED(hr))hr=imaging->CreateBitmapScaler(&s);Com<IWICBitmapScaler> scaler(s);
            int width=xpkit_view_width(side),height=xpkit_view_height(side);size_t bytes=xpkit_view_bytes(side);
            if(SUCCEEDED(hr))hr=s->Initialize(f,width,height,WICBitmapInterpolationModeFant);
            if(SUCCEEDED(hr))hr=imaging->CreateFormatConverter(&c);Com<IWICFormatConverter> converter(c);
            if(SUCCEEDED(hr))hr=c->Initialize(s,GUID_WICPixelFormat32bppRGBA,WICBitmapDitherTypeNone,nullptr,0,WICBitmapPaletteTypeCustom);
            if(SUCCEEDED(hr))hr=c->CopyPixels(nullptr,width*4,(UINT)bytes,bitmap.data());
            if(okay(hr,"Decode display pixels")){
                auto seq=shared->frame_seq[side].load();shared->frame_seq[side].store(seq+1,std::memory_order_release);
                memcpy(xpkit_view_pixels(shared,side),bitmap.data(),bytes);shared->frame_time[side]=seconds();
                shared->frame_seq[side].store(seq+2,std::memory_order_release);
                last_sequence=page_sequence;last_valid=valid;
                if(side>=2&&side<4){auto &p=shared->nd_profile[side-2];p.snapshot_count.fetch_add(1);p.snapshot_ns.fetch_add(uint64_t((seconds()-began)*1e9));}
            }
            busy=false;return S_OK;
        });
        if(!okay(web->CapturePreview(COREWEBVIEW2_CAPTURE_PREVIEW_IMAGE_FORMAT_PNG,stream.get(),done.get()),"Start display capture"))busy=false;
    }
    void tick() {
        if(!loaded||busy)return;
        auto before=shared->state_seq.load(std::memory_order_acquire);
        if(!before||(before&1)||!shared->state_size||shared->state_size>=XPKIT_JSON_CAP)return;
        double sent=shared->state_time;std::string state(shared->state,shared->state_size);
        std::atomic_thread_fence(std::memory_order_acquire);if(shared->state_seq.load()!=before)return;
        bool valid=seconds()-sent<1.5;uint32_t page_sequence=0;
        if(side>=4){page_sequence=side<6?shared->mfd[side-4].sequence.load():shared->sd.sequence.load();
            if(!page_sequence||(page_sequence&1))return;
            if(page_sequence==last_sequence&&valid==last_valid&&shared->frame_seq[side].load()){
                shared->frame_time[side]=seconds();return;
            }
        }
        std::string source;
        if(side<4){
            auto payload=valid?XpJson::parse(state):XpJson{{"ok",false},{"mode","native"}};
            source="window.__a380xTrial.apply("+payload.dump()+");window.__a380NativeFrame();";
            source+=side>=2?"window.__a380Nd?.needsSnapshot?.()!==false":"true";
        }else if(side<6){
            auto copy=std::make_unique<A380XXpKitMfd>();memcpy(copy.get(),&shared->mfd[side-4],sizeof(*copy));
            std::atomic_thread_fence(std::memory_order_acquire);if(shared->mfd[side-4].sequence.load()!=page_sequence)return;
            source="window.__a380xTrial.apply({ok:"+std::string(valid?"true":"false")+"});window.__a380Mfd.apply("+xpkit_mfd_json(*copy).dump()+");true";
        }else{
            uint32_t count=shared->sd.size;if(!count||count>=XPKIT_SD_CAP)return;
            std::string json(shared->sd.json,count);std::atomic_thread_fence(std::memory_order_acquire);
            if(shared->sd.sequence.load()!=page_sequence)return;
            auto payload=valid?XpJson::parse(json):XpJson{{"powered",false},{"page",0},{"vars",XpJson::object()}};
            source="window.__a380Sd.apply("+payload.dump()+");true";
        }
        busy=true;double began=seconds();
        auto done=callback<ICoreWebView2ExecuteScriptCompletedHandler,HRESULT,LPCWSTR>([this,page_sequence,valid,began](HRESULT hr,LPCWSTR result){
            if(!okay(hr,"Apply native display state")){busy=false;return S_OK;}
            if(side>=2&&side<4){auto &p=shared->nd_profile[side-2];p.apply_count.fetch_add(1);p.apply_ns.fetch_add(uint64_t((seconds()-began)*1e9));
                if(result&&wcscmp(result,L"false")==0&&valid==last_valid&&shared->frame_seq[side].load()){
                    shared->frame_time[side]=seconds();p.skipped_count.fetch_add(1);busy=false;return S_OK;
                }
            }
            capture(page_sequence,valid,seconds());return S_OK;
        });
        if(!okay(script(source,done.get()),"Execute display update"))busy=false;
    }
    void configure(ICoreWebView2Controller *value) {
        controller=value;controller->AddRef();controller->get_CoreWebView2(&web);
        ICoreWebView2Controller3 *c3=nullptr;
        if(SUCCEEDED(controller->QueryInterface(__uuidof(ICoreWebView2Controller3),(void**)&c3))){c3->put_ShouldDetectMonitorScaleChanges(FALSE);c3->put_RasterizationScale(xpkit_view_scale(side));c3->Release();}
        controller->put_Bounds(RECT{0,0,xpkit_view_width(side),xpkit_view_height(side)});controller->put_ZoomFactor(1);controller->put_IsVisible(TRUE);
        ICoreWebView2_3 *w3=nullptr;
        if(!okay(web->QueryInterface(__uuidof(ICoreWebView2_3),(void**)&w3),"WebView2 local content API"))return;
        HRESULT hr=w3->SetVirtualHostNameToFolderMapping(L"a380x.invalid",assets.c_str(),COREWEBVIEW2_HOST_RESOURCE_ACCESS_KIND_DENY_CORS);w3->Release();
        if(!okay(hr,"Map aircraft display assets"))return;
        ICoreWebView2Settings *settings=nullptr;if(SUCCEEDED(web->get_Settings(&settings))){settings->put_IsStatusBarEnabled(FALSE);settings->put_AreDefaultContextMenusEnabled(FALSE);settings->put_AreDevToolsEnabled(FALSE);settings->put_IsZoomControlEnabled(FALSE);settings->put_AreDefaultScriptDialogsEnabled(FALSE);settings->Release();}
        EventRegistrationToken token{};
        auto msg=callback<ICoreWebView2WebMessageReceivedEventHandler,ICoreWebView2*,ICoreWebView2WebMessageReceivedEventArgs*>([this](ICoreWebView2*,auto *args){message(args);return S_OK;});web->add_WebMessageReceived(msg.get(),&token);
        auto nav=callback<ICoreWebView2NavigationStartingEventHandler,ICoreWebView2*,ICoreWebView2NavigationStartingEventArgs*>([](ICoreWebView2*,auto *args){LPWSTR uri=nullptr;args->get_Uri(&uri);args->put_Cancel(!uri||std::wstring(uri).rfind(origin,0)!=0);CoTaskMemFree(uri);return S_OK;});web->add_NavigationStarting(nav.get(),&token);
        auto navigated=callback<ICoreWebView2NavigationCompletedEventHandler,ICoreWebView2*,ICoreWebView2NavigationCompletedEventArgs*>([this](ICoreWebView2*,auto *args){
            BOOL success=FALSE;args->get_IsSuccess(&success);
            if(!success){COREWEBVIEW2_WEB_ERROR_STATUS status;args->get_WebErrorStatus(&status);loaded=false;shared->ready[side].store(0);error("Display "+std::to_string(side)+" navigation failed, WebView2 status "+std::to_string((int)status));}
            return S_OK;
        });web->add_NavigationCompleted(navigated.get(),&token);
        auto popup=callback<ICoreWebView2NewWindowRequestedEventHandler,ICoreWebView2*,ICoreWebView2NewWindowRequestedEventArgs*>([](ICoreWebView2*,auto *args){args->put_Handled(TRUE);return S_OK;});web->add_NewWindowRequested(popup.get(),&token);
        auto permission=callback<ICoreWebView2PermissionRequestedEventHandler,ICoreWebView2*,ICoreWebView2PermissionRequestedEventArgs*>([](ICoreWebView2*,auto *args){args->put_State(COREWEBVIEW2_PERMISSION_STATE_DENY);return S_OK;});web->add_PermissionRequested(permission.get(),&token);
        auto failure=callback<ICoreWebView2ProcessFailedEventHandler,ICoreWebView2*,ICoreWebView2ProcessFailedEventArgs*>([this](ICoreWebView2*,auto*){loaded=false;shared->ready[side].store(0);error("WebView2 display process failed");return S_OK;});web->add_ProcessFailed(failure.get(),&token);
        web->AddWebResourceRequestedFilter(L"*",COREWEBVIEW2_WEB_RESOURCE_CONTEXT_ALL);
        auto resource=callback<ICoreWebView2WebResourceRequestedEventHandler,ICoreWebView2*,ICoreWebView2WebResourceRequestedEventArgs*>([](ICoreWebView2*,auto *args){
            ICoreWebView2WebResourceRequest *r=nullptr;LPWSTR uri=nullptr;args->get_Request(&r);if(r){r->get_Uri(&uri);r->Release();}
            bool allowed=uri&&(std::wstring(uri).rfind(origin,0)==0||std::wstring(uri).rfind(L"data:",0)==0);CoTaskMemFree(uri);
            if(!allowed){ICoreWebView2WebResourceResponse *response=nullptr;environment->CreateWebResourceResponse(nullptr,403,L"Offline aircraft renderer",L"",&response);if(response){args->put_Response(response);response->Release();}}
            return S_OK;
        });web->add_WebResourceRequested(resource.get(),&token);
        const wchar_t *shim=L"window.webkit={messageHandlers:{a380:{postMessage:data=>window.chrome.webview.postMessage(JSON.stringify(data))}}};";
        auto injected=callback<ICoreWebView2AddScriptToExecuteOnDocumentCreatedCompletedHandler,HRESULT,LPCWSTR>([this](HRESULT result,LPCWSTR){
            if(!okay(result,"Install native IPC bridge"))return S_OK;
            const wchar_t *file=side<2?L"pfd.html":side<4?L"nd.html":side<6?L"mfd.html":L"sd.html";
            const wchar_t *query=side==0?L"?index=1&duID=0":side==1?L"?index=2&duID=3":side==2?L"?index=1&duID=1":side==3?L"?index=2&duID=4":side==4?L"?index=1&duID=2":L"?index=2&duID=5";
            okay(web->Navigate((origin+file+query).c_str()),"Open aircraft display");return S_OK;
        });okay(web->AddScriptToExecuteOnDocumentCreated(shim,injected.get()),"Add native bridge");
    }
};
static std::vector<std::unique_ptr<View>> views;
static std::wstring quote(const std::wstring &value) {
    std::wstring out=L"\"";unsigned slashes=0;
    for(wchar_t c:value){if(c==L'\\'){++slashes;continue;}out.append(slashes*(c==L'"'?2:1),L'\\');slashes=0;if(c==L'"')out+=L'\\';out+=c;}
    out.append(slashes*2,L'\\');return out+L'"';
}
static void airport(View *view,int request,const std::string &icao) {
    workers.emplace_back([view,request,icao]{
        auto result=std::make_unique<AirportResult>();result->view=view;result->request=request;
        {std::lock_guard<std::mutex> lock(airport_mutex);auto found=airports.find(icao);if(found!=airports.end())result->data=found->second;}
        if(result->data.is_null()){
            HANDLE read=nullptr,write=nullptr;SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};
            if(CreatePipe(&read,&write,&security,0)){
                SetHandleInformation(read,HANDLE_FLAG_INHERIT,0);
                auto exe=plugin+L"\\python\\python.exe";
                auto command=quote(exe)+L" -B "+quote(assets+L"\\airport_worker.py")+L" "+quote(simulator)+L" "+quote(wide(icao));
                STARTUPINFOW start{};start.cb=sizeof(start);start.dwFlags=STARTF_USESTDHANDLES;start.hStdOutput=write;start.hStdError=write;
                PROCESS_INFORMATION child{};
                if(CreateProcessW(exe.c_str(),command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&start,&child)){
                    CloseHandle(write);write=nullptr;CloseHandle(child.hThread);std::string output;double began=seconds();bool failed=false;
                    while(true){DWORD pending=0;
                        if(PeekNamedPipe(read,nullptr,0,nullptr,&pending,nullptr)&&pending){char buffer[8192];DWORD count=0;if(ReadFile(read,buffer,std::min<DWORD>(pending,sizeof(buffer)),&count,nullptr))output.append(buffer,count);}
                        else if(WaitForSingleObject(child.hProcess,10)!=WAIT_TIMEOUT)break;
                        if(closing||seconds()-began>60||output.size()>16*1024*1024){TerminateProcess(child.hProcess,1);failed=true;break;}
                    }
                    DWORD status=1;GetExitCodeProcess(child.hProcess,&status);CloseHandle(child.hProcess);
                    try{if(failed||status)throw std::runtime_error("Airport reader failed or exceeded its limit");result->data=XpJson::parse(output);
                        std::lock_guard<std::mutex> lock(airport_mutex);if(airports.size()>=4)airports.erase(airports.begin());airports[icao]=result->data;
                    }catch(const std::exception &e){result->problem=e.what();}
                }else result->problem="Bundled airport reader could not start";
                if(write)CloseHandle(write);CloseHandle(read);
            }else result->problem="Airport reader pipe unavailable";
        }
        if(!closing&&PostMessageW(message_window,WM_APP+1,0,(LPARAM)result.get()))result.release();
    });
}
static LRESULT CALLBACK window_proc(HWND window,UINT msg,WPARAM w,LPARAM l) {
    if(msg==WM_TIMER){
        if(WaitForSingleObject(parent_process,0)!=WAIT_TIMEOUT||shared->state_seq.load()==UINT32_MAX){PostQuitMessage(0);return 0;}
        for(auto &view:views)try{view->tick();}catch(const std::exception &e){error(e.what());view->busy=false;}
        return 0;
    }
    if(msg==WM_APP+1){std::unique_ptr<AirportResult> r((AirportResult*)l);
        if(r->view->web)r->view->script("window.__a380Nd.airportResult("+std::to_string(r->request)+","+(r->problem.empty()?std::string("null"):XpJson(r->problem).dump())+","+r->data.dump()+");true");return 0;
    }
    return DefWindowProcW(window,msg,w,l);
}
int WINAPI wWinMain(HINSTANCE instance,HINSTANCE,PWSTR,int) {
    int argc=0;LPWSTR *argv=CommandLineToArgvW(GetCommandLineW(),&argc);if(!argv||argc!=5)return 2;
    assets=argv[2];simulator=argv[3];plugin=assets.substr(0,assets.find_last_of(L"/\\"));
    parent_process=OpenProcess(SYNCHRONIZE,FALSE,wcstoul(argv[4],nullptr,10));
    HANDLE mapping=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,argv[1]);LocalFree(argv);
    if(!mapping||!parent_process)return 3;
    shared=(A380XXpKitShared*)MapViewOfFile(mapping,FILE_MAP_ALL_ACCESS,0,0,sizeof(*shared));
    if(!shared||shared->magic!=XPKIT_MAGIC||shared->version!=XPKIT_VERSION)return 4;
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    if(!okay(CoInitializeEx(nullptr,COINIT_APARTMENTTHREADED),"COM initialization"))return 5;
    if(!okay(CoCreateInstance(CLSID_WICImagingFactory,nullptr,CLSCTX_INPROC_SERVER,IID_PPV_ARGS(&imaging)),"Windows image decoder"))return 6;
    WNDCLASSW klass{};klass.lpfnWndProc=window_proc;klass.hInstance=instance;klass.lpszClassName=L"A380XOfflineDisplays";RegisterClassW(&klass);
    message_window=CreateWindowExW(0,klass.lpszClassName,L"A380X display IPC",0,0,0,0,0,HWND_MESSAGE,nullptr,instance,nullptr);
    PWSTR local=nullptr;if(FAILED(SHGetKnownFolderPath(FOLDERID_LocalAppData,0,nullptr,&local)))return 7;
    std::wstring cache=std::wstring(local)+L"\\A380XDisplayHost";CoTaskMemFree(local);CreateDirectoryW(cache.c_str(),nullptr);
    cache+=L"\\session-"+std::to_wstring(GetCurrentProcessId());
    HMODULE loader=LoadLibraryExW((plugin+L"\\WebView2Loader.dll").c_str(),nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_SYSTEM32);
    using Create=HRESULT(STDMETHODCALLTYPE*)(PCWSTR,PCWSTR,ICoreWebView2EnvironmentOptions*,ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler*);
    auto create=loader?(Create)GetProcAddress(loader,"CreateCoreWebView2EnvironmentWithOptions"):nullptr;
    if(!create){error("WebView2Loader.dll missing; reinstall the complete Windows plugin package");return 8;}
    auto ready=callback<ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler,HRESULT,ICoreWebView2Environment*>([instance](HRESULT hr,ICoreWebView2Environment *env){
        if(!okay(hr,"WebView2 initialization (install Microsoft Evergreen Runtime)")){PostQuitMessage(9);return S_OK;}
        environment=env;environment->AddRef();
        for(int i=0;i<XPKIT_VIEWS;++i){auto view=std::make_unique<View>(i);View *raw=view.get();
            view->window=CreateWindowExW(WS_EX_TOOLWINDOW|WS_EX_NOACTIVATE,L"A380XOfflineDisplays",L"A380X avionics",WS_POPUP,-20000-i*1000,-20000,xpkit_view_width(i),xpkit_view_height(i),nullptr,nullptr,instance,nullptr);
            ShowWindow(view->window,SW_SHOWNOACTIVATE);views.push_back(std::move(view));
            auto done=callback<ICoreWebView2CreateCoreWebView2ControllerCompletedHandler,HRESULT,ICoreWebView2Controller*>([raw](HRESULT status,ICoreWebView2Controller *controller){if(okay(status,"Create WebView2 display"))raw->configure(controller);return S_OK;});
            okay(environment->CreateCoreWebView2Controller(raw->window,done.get()),"Create display controller");
        }return S_OK;
    });
    Com<ICoreWebView2EnvironmentOptions> options(new Options());
    if(!okay(create(nullptr,cache.c_str(),options.get(),ready.get()),"Locate WebView2 Evergreen Runtime"))return 9;
    SetTimer(message_window,1,50,nullptr);MSG msg{};while(GetMessageW(&msg,nullptr,0,0)>0){TranslateMessage(&msg);DispatchMessageW(&msg);}
    closing=true;for(auto &worker:workers)if(worker.joinable())worker.join();
    for(int i=0;i<XPKIT_VIEWS;++i)shared->ready[i].store(0);
    views.clear();if(environment)environment->Release();imaging->Release();CoUninitialize();
    UnmapViewOfFile(shared);CloseHandle(mapping);CloseHandle(parent_process);FreeLibrary(loader);return (int)msg.wParam;
}
