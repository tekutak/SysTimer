#ifndef MY_DEVICE_DRIVER_H_
#define MY_DEVICE_DRIVER_H_

#include <linux/ioctl.h>

/* System Timer ID */
typedef enum {
    // SYS_TIMER_ID_0,  //Unavailable (for GPU)
    ENUM_SYS_TIMER_ID_1,
    // SYS_TIMER_ID_2,  //Unavailable (for GPU)
    ENUM_SYS_TIMER_ID_3,
    END_OF_ENUM_SYS_TIMER_ID
}et_systimer_id;
#define NUM_OF_SYS_TIMER (END_OF_ENUM_SYS_TIMER_ID)
#define SYS_TIMER_ID_1 (ENUM_SYS_TIMER_ID_1)
#define SYS_TIMER_ID_3 (ENUM_SYS_TIMER_ID_3)

/* ioctl arguments */
typedef struct {
    pid_t  user_pid;
    et_systimer_id timer_id;
    int user_sig;
    unsigned int oneshot_ms;
}st_systimer_user_data;


/* ioctl comand groupe */
#define SYS_TIMER_IOCTL_TYPE 'S'

/* ioctl comand */
#define SYS_TIMER_SET_ONESHOT _IOW(SYS_TIMER_IOCTL_TYPE, 1, st_systimer_user_data)


#endif /* MY_DEVICE_DRIVER_H_ */