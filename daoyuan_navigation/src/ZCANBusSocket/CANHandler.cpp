//
// Created by ShizengChen on 2017/12/24.
//

#include "CANHandler.h"
#include "CANBase.h"
#include "CANSocket.h"

namespace ZCANBusSocket {
CANStatus CANHandler::OpenChannel(int channel, CANRate baudRate, int type) {
    return baseCan->OpenChannel(channel, baudRate, type);
}

CANStatus CANHandler::OpenChannel(int channel, CANRate baudRate, int argc,
                                  char *argv[]) {
    return baseCan->OpenChannel(channel, baudRate, argc, argv);
}

void CANHandler::ReadLoop(
    std::function<void(const CANMessage *msg, CANStatus status)> callback,
    uint64_t interval) {
    baseCan->ReadLoop(callback, interval);
}

void CANHandler::EndReadLoop() { baseCan->EndReadLoop(); }

CANStatus CANHandler::ReadOnce(CANMessage &msg, uint64_t timeout) {
    return baseCan->ReadOnce(msg);
}

CANStatus CANHandler::Write(const CANMessage &msg) {
    return baseCan->Write(msg);
};

CANStatus CANHandler::Write(CANMessage *msg, int count) {
    return baseCan->Write(msg, count);
}

CANStatus CANHandler::CloseChannel() { return baseCan->CloseChannel(); }

CANStatus CANHandler::FlushQueue() { return baseCan->FlushQueue(); }

std::string CANHandler::GetErrorText(CANStatus &status) {
    return baseCan->GetErrorText(status);
}

CANHandler::CANHandler(CANType canType) {
    switch (canType) {
        case CANType::SOCKET_CAN:
            baseCan = new CANSocket();
            break;
        default:
            throw "Unsupported CANType";
    }
}

CANHandler::~CANHandler() { delete baseCan; }
}  // namespace ZCANBusSocket