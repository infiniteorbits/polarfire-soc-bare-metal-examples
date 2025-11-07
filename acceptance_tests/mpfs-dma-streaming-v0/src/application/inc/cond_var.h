///
/// File            | cond_var.h
/// Description     | Simple condition variable for intreprocess synchronization
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
///                 | for this one it is actually meta.ai
/// Date            | October 2025
///
/// Copyright 2025  | RFIM Space
///

#ifndef APPLICATION_INC_COND_VAR_H_
#define APPLICATION_INC_COND_VAR_H_

#define true    1
#define false   0
#define TIMEOUT 10000000u

// Structure to represent a condition variable
struct cond_var_t{
    volatile int64_t timeout;
    volatile bool signaled;
} ;

extern volatile struct cond_var_t g_cond_var;

// Initialize a condition variable
inline void cond_var_init(volatile struct cond_var_t* cv) {
    cv->timeout = TIMEOUT;
    cv->signaled = false;
}

// Signal a condition variable
inline void cond_var_signal(volatile struct cond_var_t* cv) {
    cv->signaled = true;
}

// Wait on a condition variable
inline void cond_var_wait(volatile struct cond_var_t* cv) {
    while (!cv->signaled && cv->timeout-- > 0u) {

    }
    cv->signaled = false;
    cv->timeout = TIMEOUT;
}

#endif /* APPLICATION_INC_COND_VAR_H_ */
