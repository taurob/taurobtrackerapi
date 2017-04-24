/*
 * Copyright (c) 2017 taurob GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
// The code below was obtained from:
// http://stackoverflow.com/questions/77005/how-to-generate-a-stacktrace-when-my-gcc-c-app-crashes/1925461#1925461
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef __USE_GNU
#define __USE_GNU
#endif

#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include <unistd.h>

/* This structure mirrors the one found in /usr/include/asm/ucontext.h */
typedef struct _sig_ucontext {
    unsigned long     uc_flags;
    struct ucontext   *uc_link;
    stack_t           uc_stack;
    struct sigcontext uc_mcontext;
    sigset_t          uc_sigmask;
} sig_ucontext_t;


void Print_stacktrace()
{
    void* array[50];
    char **            messages;
    int                size, i;

    size = backtrace(array, 50);
    messages = backtrace_symbols(array, size);
        printf("\n");
    /* skip first stack frame (points here) */
    for (int i = 0; i < size && messages != NULL; ++i) {
        printf("[bt]: (%d) %s\n", i, messages[i]);
    }
}


void crit_err_hdlr(int sig_num, siginfo_t * info, void * ucontext) {
    void *             array[50];
    void *             caller_address;
    char **            messages;
    int                size, i;
    sig_ucontext_t *   uc;

    uc = (sig_ucontext_t *)ucontext;

    /* Get the address at the time the signal was raised */
    #if defined(__i386__) // gcc specific
    caller_address = (void *) uc->uc_mcontext.eip; // EIP: x86 specific
    #elif defined(__x86_64__) // gcc specific
    caller_address = (void *) uc->uc_mcontext.rip; // RIP: x86_64 specific
    #else
    #error Unsupported architecture. // TODO: Add support for other arch.
    #endif

    fprintf(stderr, "\n");
    FILE * backtraceFile;

    // In this example we write the stacktrace to a file. However, we can also just fprintf to stderr (or do both).
    std::string backtraceFilePath = "/home/taurob/stacktrace.txt";
    backtraceFile = fopen(backtraceFilePath.c_str(),"w");

    if (sig_num == SIGSEGV)
        fprintf(backtraceFile, "signal %d (%s), address is %p from %p\n",sig_num, strsignal(sig_num), info->si_addr,(void *)caller_address);
    else
        fprintf(backtraceFile, "signal %d (%s)\n",sig_num, strsignal(sig_num));

    size = backtrace(array, 50);
    /* overwrite sigaction with caller's address */
    array[1] = caller_address;
    messages = backtrace_symbols(array, size);
    /* skip first stack frame (points here) */
    for (i = 1; i < size && messages != NULL; ++i) {
        fprintf(backtraceFile, "[bt]: (%d) %s\n", i, messages[i]);
    }

    fclose(backtraceFile);
    free(messages);

    exit(EXIT_FAILURE);
}

void installSignal(int __sig) {
    struct sigaction sigact;
    sigact.sa_sigaction = crit_err_hdlr;
    sigact.sa_flags = SA_RESTART | SA_SIGINFO;
    if (sigaction(__sig, &sigact, (struct sigaction *)NULL) != 0) {
        fprintf(stderr, "error setting signal handler for %d (%s)\n",__sig, strsignal(__sig));
        exit(EXIT_FAILURE);
    }
}


// helper functions
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) elems.push_back(item);
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
