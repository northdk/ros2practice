/**
 * @file backtrace.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SYSTEM_BASIC_H
#define _SYSTEM_BASIC_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

static void dump(void)
{
#define BACKTRACE_SIZE   100
    int j, nptrs;
    void *buffer[BACKTRACE_SIZE];
    char **strings;

    nptrs = backtrace(buffer, BACKTRACE_SIZE);
    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL) {
        printf("backtrace_symbols error!");
        exit(EXIT_FAILURE);
    }

    for (j = 0; j < nptrs; j++)
        fprintf(stderr, "  [%02d] %s\n", j, strings[j]);

    free(strings);
}

static void signal_handler(int signo)
{
#if 0
    char buff[32];
    sprintf(buff,"cat /proc/%d/maps", getpid());
    system((const char*) buff);
#endif
    fprintf(stderr, "rcv signal: %d\n", signo);
    fprintf(stderr, "=========>>>catch signal %d <<<=========\n", signo);
    fprintf(stderr, "backtrace start...\n");
    dump();
    fprintf(stderr, "backtrace end...\n");

    signal(signo, SIG_DFL);    
    raise(signo);
}

#endif
