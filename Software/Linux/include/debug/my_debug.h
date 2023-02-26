#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>

extern int common_printf(FILE * fp, const int level, const char * file,
                  const int line, const char * func, const char * fmt, ...);

/* 打印等级 0-4 */
#define PRINT_LEVEL 4 //4

/* 提供4个打印接口 DEBUG, INFO, WARN, ERROR */
#define LEVEL_DEBUG 4
#define LEVEL_INFO 3
#define LEVEL_WARN 2
#define LEVEL_ERROR 1

#if (PRINT_LEVEL >= LEVEL_DEBUG)
#define DEBUG(fmt, ...) \
    common_printf(stdout, LEVEL_DEBUG, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__);
#else
#define DEBUG(fmt, ...)
#endif

#if (PRINT_LEVEL >= LEVEL_INFO)
#define INFO(fmt, ...) \
    common_printf(stdout, LEVEL_INFO, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__);
#else
#define INFO(fmt, ...)
#endif

#if (PRINT_LEVEL >= LEVEL_WARN)
#define WARN(fmt, ...) \
    common_printf(stdout, LEVEL_WARN, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__);
#else
#define WARN(fmt, ...)
#endif

#if (PRINT_LEVEL >= LEVEL_ERROR)
#define ERROR(fmt, ...) \
    common_printf(stdout, LEVEL_ERROR, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__);
#else
#define ERROR(fmt, ...)
#endif

#endif