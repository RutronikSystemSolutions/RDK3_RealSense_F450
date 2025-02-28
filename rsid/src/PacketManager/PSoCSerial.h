/*
 * PSoCSerial.h
 *
 *  Created on: Feb 24, 2025
 *      Author: ROJ030
 */

#pragma once
#include "SerialConnection.h"
#include "cyhal.h" // Need to keep cyhal_uart_t

namespace RealSenseID
{
namespace PacketManager
{
class PSoCSerial : public SerialConnection
{
public:
    explicit PSoCSerial(const SerialConfig& config);
    ~PSoCSerial() override;

    // prevent copy or assignment
    // only single connection is allowed to a serial port.
    PSoCSerial(const PSoCSerial&) = delete;
    PSoCSerial operator=(const PSoCSerial&) = delete;

    // send all bytes and return status
    SerialStatus SendBytes(const char* buffer, size_t n_bytes) final;

    // receive all bytes and copy to the buffer
    SerialStatus RecvBytes(char* buffer, size_t n_bytes) final;

private:
    SerialConfig _config;
    cyhal_uart_t uart_obj;
    int handle;
};
} // namespace PacketManager
} // namespace RealSenseID
