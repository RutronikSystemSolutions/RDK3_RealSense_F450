// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020-2021 Intel Corporation. All Rights Reserved.

#include "Timer.h"

#include "hal_timer.h"

namespace RealSenseID
{
namespace PacketManager
{

Timer::Timer(uint32_t timeout)
{
	this->timeout = timeout;
	this->start_tp = hal_timer_get_uticks();
}

Timer::Timer() : Timer {10000}
{

}

uint32_t Timer::Elapsed() const
{
	// TODO: need to check for overflow... (get_uticks < start_tp)
    uint32_t delta = hal_timer_get_uticks() - start_tp;
    return delta / 1000;
}

uint32_t Timer::TimeLeft() const
{
	uint32_t elapsed = Elapsed();
	if (elapsed > timeout) return 0;

	return timeout - elapsed;
}

bool Timer::ReachedTimeout() const
{
    return TimeLeft() <= 0;
}

void Timer::Reset()
{
    start_tp = hal_timer_get_uticks();
}
} // namespace PacketManager
} // namespace RealSenseID
