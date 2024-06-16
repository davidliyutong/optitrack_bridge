
// Created by liyutong on 2022/2/8.
//
#ifndef _DEBUG_H
#define _DEBUG_H

#include <cerrno>
#include <cstdio>
#include <cstring>

#define LEVEL_DISABLE 0
#define LEVEL_ERROR 1
#define LEVEL_WARNING 2
#define LEVEL_INFO 3
#define LEVEL_DEBUG 4

/** Set log level **/
#define LOG_LEVEL LEVEL_DEBUG

#if LOG_LEVEL >= LEVEL_ERROR
#define LOGE(TAG, M, ...) fprintf(stderr, "\033[31;1m[ERROR][%s]\033[0m (%s:%d: errno: %d) " M "\n", TAG, __FILE__, __LINE__, errno, ##__VA_ARGS__)
#else
#define LOGE(M, ...)
#endif

#if LOG_LEVEL >= LEVEL_WARNING
#define LOGW(TAG, M, ...) fprintf(stderr, "\033[33;1m[WARNING][%s]\033[0m (%s:%d) " M "\n", TAG, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define LOGW(M, ...)
#endif

#if LOG_LEVEL >= LEVEL_INFO
#define LOGI(TAG, M, ...) fprintf(stderr, "\033[32;1m[INFO][%s]\033[0m (%s:%d) " M "\n", TAG, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define LOGI(M, ...)
#endif

#if LOG_LEVEL >= LEVEL_DEBUG
#define LOGD(TAG, M, ...) fprintf(stderr, "\033[2m[DEBUG][%s] (%s:%d) " M "\n\033[0m", TAG, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define LOGD(M, ...)
#endif

#define CHECK(exp, M, ...) \
        if(!(exp)) { LOGE(M, ##__VA_ARGS__); errno=0; goto error; }

#endif