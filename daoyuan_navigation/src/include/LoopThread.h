#pragma once
#ifndef __LOOP_THREAD_H
#define __LOOP_THREAD_H

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

class ZBaseLoopThread {
   public:
    ZBaseLoopThread() {
        loopOn = false;
        suspendOn = false;
        threadEnd = true;
        threadStart = false;
        loopInterval = 1;
        isLoopInterval = false;
        this->funcBeforeLoop = std::bind(&ZBaseLoopThread::BeforeLoop, this);
        this->funcRunLoop = std::bind(&ZBaseLoopThread::RunLoop, this);
        this->funcAfterLoop = std::bind(&ZBaseLoopThread::AfterLoop, this);
    }
    ZBaseLoopThread(std::function<bool()> funcRunLoop,
                    std::function<bool()> funcBeforeLoop = []() -> bool {
                        return true;
                    },
                    std::function<int()> funcAfterLoop = []() -> int {
                        return 0;
                    }) {
        loopOn = false;
        suspendOn = false;
        threadEnd = true;
        threadStart = false;
        loopInterval = 1;
        this->funcBeforeLoop = std::move(funcBeforeLoop);
        this->funcRunLoop = std::move(funcRunLoop);
        this->funcAfterLoop = std::move(funcAfterLoop);
    }
    ZBaseLoopThread(const ZBaseLoopThread& th) {
        this->funcAfterLoop = th.funcAfterLoop;
        this->funcBeforeLoop = th.funcBeforeLoop;
        this->funcRunLoop = th.funcRunLoop;
        this->loopInterval = th.loopInterval;
        loopOn = false;
        suspendOn = false;
        threadEnd = true;
        threadStart = false;
    }
    ZBaseLoopThread& operator=(const ZBaseLoopThread& th) {
        this->funcAfterLoop = th.funcAfterLoop;
        this->funcBeforeLoop = th.funcBeforeLoop;
        this->funcRunLoop = th.funcRunLoop;
        this->loopInterval = th.loopInterval;
        loopOn = false;
        suspendOn = false;
        threadEnd = true;
        threadStart = false;
        return *this;
    }
    virtual ~ZBaseLoopThread() {
        if (t.joinable()) {
            t.detach();
        }
    }
    void Start() {
        if (loopOn) {
            return;
        }
        if (t.joinable()) {
            t.join();
        }
        loopOn = true;
        suspendOn = false;
        t = std::thread(std::bind(&ZBaseLoopThread::ThreadBody, this, this));
    }
    bool Stop(int milliseconds = -1) {
        if (threadStart && !loopOn) {
            return false;
        } else if (!threadStart) {
            return true;
        }
        loopOn = false;
        if (milliseconds < 0) {
            if (t.joinable()) {
                t.join();
            }
            return true;
        } else {
            std::unique_lock<std::mutex> lck(endmtx);
            bool succeed =
                (endcv.wait_for(lck, std::chrono::milliseconds(milliseconds),
                                [&] { return threadEnd; }));
            if (succeed) {
                t.detach();
            }
            return succeed;
        }
    }
    bool Wait(int milliseconds = -1) { return Stop(milliseconds); }
    void Suspend() {
        if (!threadStart) {
            return;
        }
        if (suspendOn) {
            return;
        }
        // 不知道之前为什么要加这个
        // if (isLoopInterval) {
        //     return;
        // }
        std::unique_lock<std::mutex> lck(suspendmtx);
        realInSuspend = false;
        suspendOn = true;
        suspendcv.wait(
            lck, [&] { return realInSuspend || !threadStart || !suspendOn; });
    }
    void Resume() { suspendOn = false; }
    bool Started() { return threadStart; }
    bool Ended() { return !threadStart; }
    bool Suspended() { return loopOn && suspendOn; }
    void SetLoopInterval(int milliseconds) { loopInterval = milliseconds; }

   private:
    std::function<bool()> funcBeforeLoop;
    std::function<bool()> funcRunLoop;
    std::function<int()> funcAfterLoop;

   protected:
    int loopInterval;
    bool loopOn;
    bool isLoopInterval;
    bool suspendOn;
    bool threadStart;
    bool threadEnd;
    bool realInSuspend;
    std::mutex endmtx;
    std::condition_variable endcv;
    std::mutex suspendmtx;
    std::condition_variable suspendcv;
    std::thread t;
    int ThreadBody(ZBaseLoopThread* loopThread) {
        loopThread->threadStart = true;
        loopThread->threadEnd = false;
        if (!loopThread->funcBeforeLoop()) {
            loopThread->loopOn = false;
            loopThread->threadEnd = true;
            loopThread->threadStart = false;
            return 0;
        }
        while (loopThread->loopOn) {
            if (loopThread->suspendOn) {
                if (!loopThread->realInSuspend) {
                    std::lock_guard<std::mutex> lck(loopThread->suspendmtx);
                    loopThread->realInSuspend = true;
                    loopThread->suspendcv.notify_all();
                }
                // std::this_thread::yield();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!loopThread->funcRunLoop()) {
                loopThread->loopOn = false;
                break;
            }
            loopThread->isLoopInterval = true;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(loopThread->loopInterval));
            loopThread->isLoopInterval = false;
        }
        int ret = loopThread->funcAfterLoop();
        std::lock_guard<std::mutex> lck(loopThread->endmtx);
        loopThread->threadEnd = true;
        loopThread->threadStart = false;
        loopThread->loopOn = false;
        loopThread->realInSuspend = false;
        loopThread->suspendOn = false;
        loopThread->isLoopInterval = false;
        loopThread->endcv.notify_all();
        std::lock_guard<std::mutex> lcck(loopThread->suspendmtx);
        loopThread->suspendcv.notify_all();
        return ret;
    }
    /**
     * @return true enter the loop, false exit thread.
     **/
    virtual bool BeforeLoop() { return true; }
    /**
     * @return the value returned while thread ended.
     **/
    virtual int AfterLoop() { return 0; }
    /**
     * The function running in loop.
     * @return true continue loop, false exit loop.
     **/
    virtual bool RunLoop() { return true; }
};
#endif
