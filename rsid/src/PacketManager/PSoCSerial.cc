/*
 * PSoCSerial.cc
 *
 *  Created on: Feb 24, 2025
 *      Author: ROJ030
 */
#include "PSoCSerial.h"
#include "CommonTypes.h"
#include "SerialPacket.h"
#include "Timer.h"
#include "Logger.h"

#include "cyhal_uart.h"
#include "cycfg_pins.h"

#include <stdexcept>

static const char* LOG_TAG = "PSoCSerial";

namespace RealSenseID
{
namespace PacketManager
{
PSoCSerial::~PSoCSerial()
{
	if (this->handle == 0)
	{
		cyhal_uart_free(&(this->uart_obj));
	}
}

PSoCSerial::PSoCSerial(const SerialConfig& config) : _config {config}
{
    // TODO: open, read existing during 200ms
	LOG_DEBUG(LOG_TAG, "Calling PSoCSerial");

	const cyhal_uart_cfg_t uart_cfg =
	{
			.data_bits = 8,
			.stop_bits = 1,
			.parity = CYHAL_UART_PARITY_NONE,
			.rx_buffer = NULL,
			.rx_buffer_size = 0
	};

	// #define CANFD_RX (P5_0)
	// #define CANFD_TX (P5_1)
	 cy_rslt_t result = cyhal_uart_init(&(this->uart_obj), ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_cfg);
	//cy_rslt_t result = cyhal_uart_init(&(this->uart_obj), CANFD_TX, CANFD_RX, NC, NC, NULL, &uart_cfg);
	if (result != CY_RSLT_SUCCESS)
	{
		LOG_ERROR(LOG_TAG, "cyhal_uart_init error");
		this->handle = -1;
		throw std::runtime_error("PSoCSerial cannot init");
	}

	uint32_t baudrate = 115200;
	result = cyhal_uart_set_baud(&(this->uart_obj), baudrate, &baudrate);
	if (result != CY_RSLT_SUCCESS)
	{
		LOG_ERROR(LOG_TAG, "cyhal_uart_set_baud error");
		this->handle = -1;
		throw std::runtime_error("PSoCSerial cannot set baud rate");
	}

	this->handle = 0;

	LOG_DEBUG(LOG_TAG, "Init UART success");
}

SerialStatus PSoCSerial::SendBytes(const char* buffer, size_t n_bytes)
{
    size_t towrite = (size_t)n_bytes;
    // Workaround - else problems with delay...
    //Cy_SysLib_Delay(1);
    Cy_SysLib_DelayUs(500);
    cy_rslt_t result = cyhal_uart_write(&(this->uart_obj), (void*)buffer, &towrite);
    if (result != CY_RSLT_SUCCESS)
    {
    	LOG_ERROR(LOG_TAG, "cyhal_uart_write error");
    	return SerialStatus::SendFailed;
    }

    return SerialStatus::Ok;
}

// receive all bytes and copy to the buffer or return error status
SerialStatus PSoCSerial::RecvBytes(char* buffer, size_t n_bytes)
{
    if (n_bytes == 0)
    {
        LOG_ERROR(LOG_TAG, "Attempt to recv 0 bytes");
        return SerialStatus::RecvFailed;
    }

    // set timeout to depend on number of bytes needed
    //Timer timer {200 + 4 * n_bytes};
    Timer timer {1000 + 4 * n_bytes};
    //PacketManager::Timer timer {1000 + 4 * n_bytes};
    unsigned int total_bytes_read = 0;
    while (!timer.ReachedTimeout())
    {
    	size_t toread = n_bytes - total_bytes_read;
    	cy_rslt_t result = cyhal_uart_read(&(this->uart_obj), (void*)&buffer[total_bytes_read], &toread);
    	if (result != CY_RSLT_SUCCESS)
    	{
    		LOG_ERROR(LOG_TAG, "cyhal_uart_read error");
    		return SerialStatus::RecvFailed;
    	}

    	total_bytes_read += toread;

		if (total_bytes_read == n_bytes)
		{
			return SerialStatus::Ok;
		}
    }

    // reached here on timeout
    if (n_bytes != 1)
    {
        LOG_DEBUG(LOG_TAG, "Timeout recv");
    }

    return SerialStatus::RecvTimeout;
}
} // namespace PacketManager
} // namespace RealSenseID



