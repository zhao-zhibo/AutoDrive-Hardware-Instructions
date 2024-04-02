#include  "CANSocket.h"
#include <cstring>
#include <stdio.h>
#include <errno.h>
#include <string.h>
// #include <assert.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <iostream>
//#define CAN1TOCAN2
namespace ZCANBusSocket {


CANSocket::CANSocket() : loopOn(false) {
}

CANSocket::~CANSocket() {}

CANStatus CANSocket::OpenChannel(int channel, CANRate baudRate, int type) {
    char* argv[] = {(char*)&type};
    return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANSocket::OpenChannel(int channel, CANRate baudRate, int argc,
                              char* argv[]) {
    struct sockaddr_can addr;
    struct ifreq ifr = {0};

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s == -1) {
        printf("socket() error.\n");
        return -1;
    }

    static char name[16];
    if (channel == 0) {
        std::strcpy(name, "can0");
    } else if (channel == 1) {
        std::strcpy(name, "can1");
    } else {
        std::strcpy(name, "can0");
    }
    // static char name[16] = "can0";
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", name);
    
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
        printf("device can0 error.\n");
        close(s);
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    /* bind */
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        printf("bind() error, %s", strerror(errno));
        close(s);
        return -1;
    }
    printf("open channel check.\n");
    return 0;
}

void CANSocket::EndReadLoop() {
    loopOn = false;
    close(s);
    th->join();
    delete th;
}

void CANSocket::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
        th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat;
        while (loopOn) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(s, &fds);

            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 1000; // 1 ms

            int rv = select(s +1, &fds, NULL, NULL, &tv);
            
            if (rv == 0) {
                // return 0;
            } else if (rv > 0) {
                struct can_frame canmsg = {0};
                int ret = read(s, &canmsg, sizeof(canmsg));
                if (ret == -1) {
                    printf("read() error on can0, %s",  strerror(errno));
                } else if (ret == sizeof(canmsg)) {
                    msg.length = canmsg.can_dlc;
                    msg.id = canmsg.can_id;
                    memcpy(msg.msg, canmsg.data, msg.length);
                    callback(&msg, 0);
                    // printf("read #0 0x%x (%d):[%x %x %x %x %x %x %x %x]\n", canmsg.can_id, canmsg.can_dlc, canmsg.data[0], canmsg.data[1], canmsg.data[2], canmsg.data[3], canmsg.data[4], canmsg.data[5], canmsg.data[6], canmsg.data[7]);
                } else {
                    printf("read() return %d on can0", ret);
                }
            } else {
                printf("select() error on can0, %s",  strerror(errno));
            }
        }
        });
    }


CANStatus CANSocket::ReadOnce(CANMessage& msg, uint64_t timeout) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000; // 1 ms

    int rv = select(0, &fds, NULL, NULL, &tv);
    if (rv == 0) {
        return 0;
    } else if (rv > 0) {
        struct can_frame canmsg = {0};
        int ret = read(0, &canmsg, sizeof(canmsg));
        if (ret == -1) {
            printf("read() error on can0, %s",  strerror(errno));
            return -1;
        } else if (ret == sizeof(canmsg)) {
            // *frame = canmsg;
            return 1;
        } else {
            printf("read() return %d on can0", ret);
            return -1;
        }
    } else {
        printf("select() error on can0, %s",  strerror(errno));
        return -1;
    }
    return -1;
}

CANStatus CANSocket::Write(const CANMessage& msg) {
    struct can_frame data = {0};
    data.can_id = msg.id;
    data.can_dlc = msg.length;
    memcpy(data.data, msg.msg, msg.length);
    if (write(s, &data, sizeof(struct can_frame)) == -1) {
        return -1;
    }
    return 1;
};

CANStatus CANSocket::Write(CANMessage* msg, int count) {
    for (int i = 0; i < count; i++) {
        struct can_frame data = {0};
        data.can_id = msg[i].id;
        data.can_dlc = msg[i].length;
        memcpy(data.data, msg[i].msg, msg[i].length);
        if (write(s, &data, sizeof(struct can_frame)) == -1) {
            return -1;
        }

        // printf("write #%d 0x%x (%d):[%x %x %x %x %x %x %x %x]\n", i, data.can_id, data.can_dlc, data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5], data.data[6], data.data[7]);
    }
    return 1;
}

CANStatus CANSocket::CloseChannel() {
    if (loopOn) {
        EndReadLoop();
    }
    return 0;
}

CANStatus CANSocket::FlushQueue() {
}

std::string CANSocket::GetErrorText(CANStatus& status) {
   
}
}  // namespace ZCANBusSocket