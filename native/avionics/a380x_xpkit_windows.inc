/* SPDX-License-Identifier: GPL-3.0-or-later
 * Windows process/mapping boundary. No shell, TCP server or aircraft writes.
 * The mapping lives for the whole plugin enable; a supervised restart (a380x_xpkit_adapter.inc)
 * only starts a new helper process on it. Windows never uses the v9 zero-copy surfaces.
 */
static wchar_t xpkit_windows_exe[2304],xpkit_windows_root[2304],xpkit_windows_xp[2048];
static int xpkit_windows_wide(const char *utf8,wchar_t *out,int count) {
    return MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,utf8,-1,out,count)>0;
}
/* Watchdog: end an unresponsive helper now (its handles are closed by the caller). */
static void xpkit_windows_kill(void) {
    if(xpkit_pid && WaitForSingleObject(xpkit_pid,0)==WAIT_TIMEOUT){TerminateProcess(xpkit_pid,1);WaitForSingleObject(xpkit_pid,1000);}
}
static void xpkit_windows_stop(void) {
    if(xpkit_pid) {
        /* Signal normal exit by invalidating this owned mapping. The helper
         * also watches the simulator process, so it cannot remain orphaned. */
        if(xpkit_shared)atomic_store(&xpkit_shared->state_seq,UINT32_MAX);
        if(WaitForSingleObject(xpkit_pid,250)==WAIT_TIMEOUT)TerminateProcess(xpkit_pid,0);
        CloseHandle(xpkit_pid);
    }
    xpkit_pid=NULL;
    if(xpkit_job)CloseHandle(xpkit_job);
    xpkit_job=NULL;
    if(xpkit_shared)UnmapViewOfFile(xpkit_shared);
    xpkit_shared=NULL;
    if(xpkit_mapping)CloseHandle(xpkit_mapping);
    xpkit_mapping=NULL;xpkit_shared_name[0]=0;
}
/* Start one helper process on the existing mapping. */
static int xpkit_windows_spawn(void) {
    wchar_t command[8192];
    STARTUPINFOW startup;PROCESS_INFORMATION child;
    if(!xpkit_mapping||!xpkit_shared_name[0])return 0;
    if(swprintf(command,8192,L"\"%ls\" \"%ls\" \"%ls\" \"%ls\" %lu",xpkit_windows_exe,xpkit_shared_name,xpkit_windows_root,
        xpkit_windows_xp,(unsigned long)GetCurrentProcessId())<0)return 0;
    memset(&startup,0,sizeof(startup));startup.cb=sizeof(startup);memset(&child,0,sizeof(child));
    if(!CreateProcessW(xpkit_windows_exe,command,NULL,NULL,FALSE,CREATE_NO_WINDOW|CREATE_SUSPENDED,NULL,NULL,&startup,&child))return 0;
    /* Own only this helper tree. Closing the job also cleans up WebView2 and
     * airport-reader children if the renderer crashes or is forcibly stopped. */
    xpkit_job=CreateJobObjectW(NULL,NULL);
    if(xpkit_job){
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits;memset(&limits,0,sizeof(limits));
        limits.BasicLimitInformation.LimitFlags=JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if(!SetInformationJobObject(xpkit_job,JobObjectExtendedLimitInformation,&limits,sizeof(limits))||
           !AssignProcessToJobObject(xpkit_job,child.hProcess)){CloseHandle(xpkit_job);xpkit_job=NULL;}
    }
    xpkit_pid=child.hProcess;
    if(ResumeThread(child.hThread)==(DWORD)-1){TerminateProcess(child.hProcess,1);CloseHandle(child.hThread);return 0;}
    CloseHandle(child.hThread);return 1;
}
static int xpkit_windows_start(const char *helper,const char *assets,const char *system) {
    static unsigned serial;
    if(!xpkit_windows_wide(helper,xpkit_windows_exe,2304)||!xpkit_windows_wide(assets,xpkit_windows_root,2304)||
       !xpkit_windows_wide(system,xpkit_windows_xp,2048)||GetFileAttributesW(xpkit_windows_exe)==INVALID_FILE_ATTRIBUTES)return 0;
    /* Trailing slash before a closing Windows quote escapes the quote. Paths
     * supplied by XPLM may end in '/'; normalise to a slash-free directory. */
    size_t n=wcslen(xpkit_windows_xp);while(n && (xpkit_windows_xp[n-1]=='/'||xpkit_windows_xp[n-1]=='\\'))xpkit_windows_xp[--n]=0;
    swprintf(xpkit_shared_name,128,L"Local\\A380XDisplay-%lu-%llu-%u",(unsigned long)GetCurrentProcessId(),
        (unsigned long long)GetTickCount64(),++serial);
    xpkit_mapping=CreateFileMappingW(INVALID_HANDLE_VALUE,NULL,PAGE_READWRITE,0,
        (DWORD)sizeof(A380XXpKitShared),xpkit_shared_name);
    if(!xpkit_mapping||GetLastError()==ERROR_ALREADY_EXISTS)return 0;
    xpkit_shared=MapViewOfFile(xpkit_mapping,FILE_MAP_ALL_ACCESS,0,0,sizeof(*xpkit_shared));
    if(!xpkit_shared)return 0;
    memset(xpkit_shared,0,sizeof(*xpkit_shared));xpkit_shared->magic=XPKIT_MAGIC;xpkit_shared->version=XPKIT_VERSION;
    for(int v=0;v<XPKIT_VIEWS;++v)atomic_store(&xpkit_shared->surface_front[v],XPKIT_SURFACE_PIXELS);
    return xpkit_windows_spawn();
}
