// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020-2021 Intel Corporation. All Rights Reserved.

#pragma once

#include <stdint.h>

namespace RealSenseID
{
namespace PacketManager
{
class Timer
{
public:
    Timer();
    explicit Timer(uint32_t timeout);
    
    uint32_t Elapsed() const;
    uint32_t TimeLeft() const;
    bool ReachedTimeout() const;
    void Reset();

private:
    uint32_t start_tp;
    uint32_t timeout;
};
} // namespace PacketManager
} // namespace RealSenseID
