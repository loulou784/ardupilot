#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include <pthread.h>

#include "AP_HAL_QNX_Namespace.h"

class QNX::Semaphore : public AP_HAL::Semaphore {
public:
    friend class QNX::BinarySemaphore;
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
protected:
    pthread_mutex_t _lock;
};


class QNX::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state=false);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;

protected:
    QNX::Semaphore mtx;
    pthread_cond_t cond;
    bool pending;
};
