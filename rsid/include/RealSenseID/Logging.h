// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020-2021 Intel Corporation. All Rights Reserved.

#pragma once

#include "RealSenseIDExports.h"
#include <functional>

/**
 *  RealSenseID offers the SetLogCallback() function to get logging events.
 *  In addition, for debugging purposes, one can use the following cmake options:
 *    cmake -DRSID_DEBUG_CONSOLE=ON - to activate colored debug output to stdout.
 *    cmake -DRSID_DEBUG_FILE=ON - to activate debug output to "rsid_debug.log" file.
 */
namespace RealSenseID
{
/**
 * Log sevrity from lowest(Debug) to highest (Critical)
 */
enum class LogLevel
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical,
    Off
};

} // namespace RealSenseID
