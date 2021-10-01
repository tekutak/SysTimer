#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include "systimer.h"

#define SIG_NOTICE_1 (SIGRTMIN + 1)
#define SIG_NOTICE_2 (SIGRTMIN + 2)

#define TIMER_MS (10000)

void thread1(void)
{
    int fd;
    sigset_t ss;
    int signo;
    struct timespec ts;
    struct tm tm;

    if ((fd = open("/dev/systimer0", O_RDWR)) < 0)
        perror("open");

    st_systimer_user_data user_data;
    user_data.user_sig = SIG_NOTICE_1;
    user_data.timer_id = SYS_TIMER_ID_1;
    user_data.user_pid = getpid();
    user_data.oneshot_ms = TIMER_MS;
    if (ioctl(fd, SYS_TIMER_SET_ONESHOT, &user_data) < 0)
        perror("ioctl_set");

    sigemptyset(&ss);
    sigaddset(&ss, user_data.user_sig);
    while (1)
    {
        if (sigwait(&ss, &signo) == 0)
        {
            /* 次のタイマースタートしておく */
            ioctl(fd, SYS_TIMER_SET_ONESHOT, &user_data);

            /* 表示 */
            clock_gettime(CLOCK_REALTIME, &ts); //時刻の取得
            localtime_r(&ts.tv_sec, &tm);       //取得時刻をローカル時間に変換
            printf("sig[%d], %d/%02d/%02d %02d:%02d:%02d.%09ld\n",
                   signo,
                   tm.tm_year + 1900,
                   tm.tm_mon + 1,
                   tm.tm_mday,
                   tm.tm_hour,
                   tm.tm_min,
                   tm.tm_sec,
                   ts.tv_nsec);
        }
    }
}

void thread2(void)
{
    int fd;
    sigset_t ss;
    int signo;
    struct timespec ts;
    struct tm tm;

    if ((fd = open("/dev/systimer0", O_RDWR)) < 0)
        perror("open");

    st_systimer_user_data user_data;
    user_data.user_sig = SIG_NOTICE_2;
    user_data.timer_id = SYS_TIMER_ID_3;
    user_data.user_pid = getpid();
    user_data.oneshot_ms = TIMER_MS;
    if (ioctl(fd, SYS_TIMER_SET_ONESHOT, &user_data) < 0)
        perror("ioctl_set");

    sigemptyset(&ss);
    sigaddset(&ss, user_data.user_sig);
    while (1)
    {
        if (sigwait(&ss, &signo) == 0)
        {
            /* 次のタイマースタートしておく */
            ioctl(fd, SYS_TIMER_SET_ONESHOT, &user_data);

            /* 表示 */
            clock_gettime(CLOCK_REALTIME, &ts); //時刻の取得
            localtime_r(&ts.tv_sec, &tm);       //取得時刻をローカル時間に変換
            printf("sig[%d], %d/%02d/%02d %02d:%02d:%02d.%09ld\n",
                   signo,
                   tm.tm_year + 1900,
                   tm.tm_mon + 1,
                   tm.tm_mday,
                   tm.tm_hour,
                   tm.tm_min,
                   tm.tm_sec,
                   ts.tv_nsec);
        }
    }
}

int main()
{
    pthread_t p_th1, p_th2;

    /* Block Signal */
    /* Signalによってプロセスが終了しないようにブロックしておく */
    sigset_t ss;
    sigemptyset(&ss);
    sigaddset(&ss, SIG_NOTICE_1);
    sigaddset(&ss, SIG_NOTICE_2);
    sigprocmask(SIG_BLOCK, &ss, NULL);

    /* Thread Create */
    /* スレッド起動 */
    pthread_create(&p_th1, NULL, (void *)thread1, NULL);
    pthread_create(&p_th2, NULL, (void *)thread2, NULL);

    /* Wait */
    /* 待機 */
    pthread_join(p_th1, NULL);
    pthread_join(p_th2, NULL);

    return 0;
}