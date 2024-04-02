#pragma once
#ifndef __CANBase_SOCKET_H
#define __CANBase_SOCKET_H
#include <thread>
#include "CANBase.h"
#include <linux/can.h>
#include <linux/can/raw.h>

namespace ZCANBusSocket {
class CANSocket : public CANBase {
   private:
    std::thread* th;
    bool loopOn;
    int handle;
    int s;
    // static char m_can_name[2][16];
    // static int  m_can[2] = { -1, -1};
    // UINT can_index = 0;
    // UINT device_index = 0;
    // UINT device_type = 0;

   public:
    CANSocket();
    ~CANSocket();
    /**
     * @param channel Indicates device_index
     * @param type Indicates device_type
     * @see <controlcan.h>
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int type = 0) override;

    /**
     * @param channel Indicates device_index
     * @param argv Indicates device_type, can_index.
     * Default 0.
     * @see <controlcan.h>
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int argc,
                          char* argv[]) override;
    void ReadLoop(
        std::function<void(const CANMessage* msg, CANStatus status)> callback,
        uint64_t interval) override;
    void EndReadLoop() override;
    CANStatus ReadOnce(CANMessage& msg, uint64_t timeout) override;
    CANStatus Write(const CANMessage& msg) override;
    CANStatus Write(CANMessage* msg, int count) override;
    CANStatus CloseChannel() override;
    CANStatus FlushQueue() override;
    std::string GetErrorText(CANStatus& status) override;
};
}  // namespace ZCANBusSocket
#endif