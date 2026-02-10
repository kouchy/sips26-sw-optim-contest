/*!
 * \file
 * \brief Define some basic macros for debugging and Mathematics operations.
 */

#pragma once

#include <stdint.h>
#include <sys/time.h>

// MOTION_ENABLE_DEBUG : macro definissant le fonctionnement general des macros de debug

#ifdef MOTION_ENABLE_DEBUG

// macro de debug
#ifndef VERBOSE
#define VERBOSE(X) X
#endif
#define PUTS(str) fprintf(stderr, "(DBG) %s\n", str)
#define CR putchar('\n');
#define SHOWNAME(X) #X

#define IDISP(x) fprintf(stderr, "(DBG) %s = %3d\n", #x, x)
#define FDISP(x) fprintf(stderr, "(DBG) %s = %3f\n", #x, x)
#define DISP(x) fprintf(stderr, "(DBG) %s = %s\n", #x, x)

#else

#ifndef VERBOSE
#define VERBOSE(X)
#endif
#define PUTS(str)
#define CR putchar('\n');
#define SHOWNAME(X) #X // pas d'appel en mode release

#define DISP(x)
#define FDISP(x)
#define IDISP(x)

#endif // MOTION_ENABLE_DEBUG

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) < (b)) ? (b) : (a))
#endif
#define CLAMP(x, a, b) MIN(MAX(x, a), b)

#define TIME_POINT(name) \
    struct timeval t_##name; \
    gettimeofday(&t_##name, NULL); \
    double t_##name##_us = (double)(t_##name.tv_sec * 1e6 + t_##name.tv_usec);

#define TIME_ELAPSED2_US(start_name, stop_name) (t_##stop_name##_us - t_##start_name##_us)
#define TIME_ELAPSED2_MS(start_name, stop_name) (t_##stop_name##_us - t_##start_name##_us) * 1e-3
#define TIME_ELAPSED2_S(start_name, stop_name) (t_##stop_name##_us - t_##start_name##_us) * 1e-6
#define TIME_ELAPSED2_SEC(start_name, stop_name) TIME_ELAPSED2_S(start_name, stop_name)

#define TIME_ELAPSED_US(name) (t_##name##_us)
#define TIME_ELAPSED_MS(name) (t_##name##_us) * 1e-3
#define TIME_ELAPSED_S(name) (t_##stop##_us) * 1e-6
#define TIME_ELAPSED_SEC(name) TIME_ELAPSED_S(name)

#define TIME_SETA(name) \
    double t_##name##_us = 0.;
#define TIME_ACC(acc_name, start_name, stop_name) \
    t_##acc_name##_us += t_##stop_name##_us - t_##start_name##_us;
#define TIME_ADD(acc_name, name) \
    t_##acc_name##_us += t_##name##_us;
