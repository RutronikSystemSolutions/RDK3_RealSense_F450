// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020-2021 Intel Corporation. All Rights Reserved.

#include "Logger.h"
#include <memory>
#include <cstdarg> // for va_start
#include <cassert>
#include <stdio.h>

#define LOG_BUFFER_SIZE 512

namespace RealSenseID
{
Logger::Logger()
{
    // TODO
}

Logger::~Logger()
{
    // TODO
}

// if log level is right, vsprintf the args to buffer and log it
#define LOG_IT_(LEVEL)                                                                                                                     \
    va_list args;                                                                                                                          \
    va_start(args, format);                                                                                                                \
    char buffer[LOG_BUFFER_SIZE];                                                                                                          \
    auto Ok = vsnprintf(buffer, sizeof(buffer), format, args) >= 0;                                                                        \
    if (!Ok)                                                                                                                               \
        snprintf(buffer, sizeof(buffer), "(bad printf format \"%s\")", format);                                                            \
    printf("%s: %s\r\n", tag, buffer);                                                                                           		   \
    va_end(args)


void Logger::Trace(const char* tag, const char* format, ...)
{
    LOG_IT_("trace");
}

void Logger::Debug(const char* tag, const char* format, ...)
{
	LOG_IT_("debug");
}

void Logger::Info(const char* tag, const char* format, ...)
{
	LOG_IT_("info");
}

void Logger::Warning(const char* tag, const char* format, ...)
{
	LOG_IT_("warn");
}

void Logger::Error(const char* tag, const char* format, ...)
{
	LOG_IT_("error");
}

void Logger::Critical(const char* tag, const char* format, ...)
{
	LOG_IT_("critical");
}

void Logger::DebugBytes(const char* tag, const char* msg, const char* buf, size_t size)
{
    printf("DebugBytes\r\n");
}
} // namespace RealSenseID
