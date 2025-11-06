/*
 * cond_var.h
 *
 *  Created on: 29 Oct 2025
 *      Author: TRAJCE Nikolov
 *      Actually: meta.ai for this specific one
 */

#ifndef APPLICATION_INC_COND_VAR_H_
#define APPLICATION_INC_COND_VAR_H_

#define true 1
#define false 0

// Structure to represent a condition variable
struct cond_var_t{
    volatile bool signaled;
} ;

extern volatile struct cond_var_t g_cond_var;

// Initialize a condition variable
inline void cond_var_init(volatile struct cond_var_t* cv) {
    cv->signaled = false;
}

// Signal a condition variable
inline void cond_var_signal(volatile struct cond_var_t* cv) {
    cv->signaled = true;
}

// Wait on a condition variable
inline void cond_var_wait(volatile struct cond_var_t* cv) {
    while (!cv->signaled) { /// Add timeout
        // Busy-waiting, not recommended for production use
    }
    cv->signaled = false;
}

#endif /* APPLICATION_INC_COND_VAR_H_ */
