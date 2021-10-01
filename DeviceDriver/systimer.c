#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/current.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include "systimer.h"

/* -----------------------------------------------------------------------------
 Define
------------------------------------------------------------------------------*/
//Device Tree Compatible Name
#define SYS_TIMER_DEVICE_TREE_COMPATIBLE ("brcm,bcm2835-system-timer-mydev")

//Peripheral Physical Address (from https://cs140e.sergio.bz/docs/BCM2837-ARM-Peripherals.pdf)
#define REG_ADDR_BASE (0x3F000000)

//Peripheral System Timer Address
#define REG_ADDR_SYS_TIMER_BASE (0x00003000)
#define REG_ADDR_SYS_TIMER_CTRL_STATUS (REG_ADDR_SYS_TIMER_BASE + 0x0)
#define REG_ADDR_SYS_TIMER_CONTER_LOWER32 (REG_ADDR_SYS_TIMER_BASE + 0x4)
#define REG_ADDR_SYS_TIMER_CONTER_UPPER32 (REG_ADDR_SYS_TIMER_BASE + 0x8)
#define REG_ADDR_SYS_TIMER_COMPARE_0 (REG_ADDR_SYS_TIMER_BASE + 0xC)
#define REG_ADDR_SYS_TIMER_COMPARE_1 (REG_ADDR_SYS_TIMER_BASE + 0x10)
#define REG_ADDR_SYS_TIMER_COMPARE_2 (REG_ADDR_SYS_TIMER_BASE + 0x14)
#define REG_ADDR_SYS_TIMER_COMPARE_3 (REG_ADDR_SYS_TIMER_BASE + 0x18)

//Peripheral Interrupt Address
#define REG_ADDR_IRQ_BASE (0x0000B000)
#define REG_ADDR_IRQ_INT_ENABLE_0_31 (REG_ADDR_IRQ_BASE + 0x210)
#define REG_ADDR_IRQ_INT_ENABLE_32_63 (REG_ADDR_IRQ_BASE + 0x214)
#define REG_ADDR_IRQ_INT_DISABLE_0_31 (REG_ADDR_IRQ_BASE + 0x21C)
#define REG_ADDR_IRQ_INT_DISABLE_32_63 (REG_ADDR_IRQ_BASE + 0x220)

//IRQ Info
#define IRQ_HWIRQ_NO_SYS_TIMER_0 (0x0)
#define IRQ_HWIRQ_NO_SYS_TIMER_1 (0x1)
#define IRQ_HWIRQ_NO_SYS_TIMER_2 (0x2)
#define IRQ_HWIRQ_NO_SYS_TIMER_3 (0x3)
#define IRQ_ENABLE_BIT_SYS_TIMER_0 (1 << 0)
#define IRQ_ENABLE_BIT_SYS_TIMER_1 (1 << 1)
#define IRQ_ENABLE_BIT_SYS_TIMER_2 (1 << 2)
#define IRQ_ENABLE_BIT_SYS_TIMER_3 (1 << 3)

//System Timer
#define SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_0 (1 << 0) //Unavailable(for GPU)
#define SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_1 (1 << 1) //Available
#define SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_2 (1 << 2) //Unavailable(for GPU)
#define SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_3 (1 << 3) //Available

//Timer Clock Info
#define SYS_TIMER_CLK_PER_S (1000000) //1MHz
#define SYS_TIMER_CLK_PER_MS (SYS_TIMER_CLK_PER_S / 1000)
#define CONVERT_MS_TO_CLK(ms) (ms * SYS_TIMER_CLK_PER_MS)

//Virtual Address Setting
#define VIRTUAL_PAGE_SIZE (50 * 1024) //page size

//Device Driver Info
MODULE_LICENSE("Dual BSD/GPL");
#define DRIVER_NAME ("systimer")
#define DRIVER_INT_HANDLER ("systimer_intr")

//Return
#define ERROR_RET (-1)

/* -----------------------------------------------------------------------------
 Macro
------------------------------------------------------------------------------*/
//Register Access
#define REG(addr) (*((volatile unsigned int *)(addr)))

/* -----------------------------------------------------------------------------
 Typedef
------------------------------------------------------------------------------*/
//Timer Status
//タイマー使用状況
typedef enum
{
    TIMER_STATUS_IDLE,
    TIMER_STATUS_BUSY,
    END_OF_TIMER_STATUS
} et_timer_status;

//Device Info
//割り込みハンドラがどのタイマーの割り込みかを判別するための情報
typedef struct
{
    et_systimer_id timer_id;
    int irq;
} st_dev_info;

//Timer Info
//現在のタイマーの使用状況とユーザーデータの参照
typedef struct
{
    et_timer_status status;
    st_systimer_user_data user_data;
} st_timer_info;

//Irq Info
//各タイマーの設定情報
typedef struct
{
    int hwirq;
    int int_enable_bit;
    int ctrl_status_bit;
    int compare_reg_adr;
} st_irq_info;

/* -----------------------------------------------------------------------------
 Static global
------------------------------------------------------------------------------*/
//Device Driver Info
static const unsigned int MINOR_BASE = 0;   // このデバイスドライバで使うマイナー番号の開始番号と個数(=デバイス数)
static const unsigned int MINOR_NUM = 1;    // マイナー番号は 0のみ
static unsigned int mydevice_major;         // このデバイスドライバのメジャー番号(動的に決める)
static struct cdev mydevice_cdev;           // キャラクタデバイスのオブジェクト
static struct class *mydevice_class = NULL; // デバイスドライバのクラスオブジェクト

//Peripheral Virtual Address
static void __iomem *vaddr_reg_base = 0;

//Device Tree System Timer Node
static struct device_node *timer_node; //デバイスツリー参照先

//Device Info
//割り込みハンドラがどのタイマーの割り込みかを判別するための情報
static st_dev_info Dev_info[NUM_OF_SYS_TIMER] = {
    {SYS_TIMER_ID_1, 0},
    {SYS_TIMER_ID_3, 0},
};

//User_Data Initial Value
//初期化用ユーザーデータ
static const st_systimer_user_data User_data_init = {
    0, SYS_TIMER_ID_1, 0, 0};

//Timer Info
//現在のタイマーの使用状況とユーザーデータの参照
static st_timer_info Timer_info[NUM_OF_SYS_TIMER] = {
    {TIMER_STATUS_IDLE, User_data_init},
    {TIMER_STATUS_IDLE, User_data_init}};

//Irq Info
//各タイマーの設定情報
static const st_irq_info Irq_info_table[NUM_OF_SYS_TIMER] = {
    {IRQ_HWIRQ_NO_SYS_TIMER_1, IRQ_ENABLE_BIT_SYS_TIMER_1, SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_1, REG_ADDR_SYS_TIMER_COMPARE_1},
    {IRQ_HWIRQ_NO_SYS_TIMER_3, IRQ_ENABLE_BIT_SYS_TIMER_3, SYS_TIMER_CTRL_STATUS_CLEAR_BIT_TIMER_3, REG_ADDR_SYS_TIMER_COMPARE_3}};

/* -----------------------------------------------------------------------------
 Function   : open, close, read, write
 Memo       : 
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
static int systimer_open(struct inode *inode, struct file *file)
{
    printk("systimer_open\n");
    return 0;
}
static int systimer_close(struct inode *inode, struct file *file)
{
    printk("systimer_close\n");

    /* Clear Timer Info */
    for (int i = 0; i < NUM_OF_SYS_TIMER; i++)
    {
        Timer_info[i].status = TIMER_STATUS_IDLE;
        Timer_info[i].user_data = User_data_init;
    }

    return 0;
}
static ssize_t systimer_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    printk("systimer not support read\n");
    return count;
}
static ssize_t systimer_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    printk("systimer not support write\n");
    return count;
}

/* -----------------------------------------------------------------------------
 Function   : systimer_set_timer
 Memo       : タイマー設定
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
static long systimer_set_timer(st_systimer_user_data *user_data)
{
    et_systimer_id timer_id = user_data->timer_id;

    /* Check Timer Info */
    if (Timer_info[timer_id].status == TIMER_STATUS_BUSY)
    {
        printk("Timer BUSY. Timer already running.\n");
        return ERROR_RET;
    }

    /* Clear Interrupt (Contrl Status) */
    REG(vaddr_reg_base + REG_ADDR_SYS_TIMER_CTRL_STATUS) = Irq_info_table[timer_id].ctrl_status_bit; //write clear

    /* Set Compare Value */
    int count = REG(vaddr_reg_base + REG_ADDR_SYS_TIMER_CONTER_LOWER32);
    REG(vaddr_reg_base + Irq_info_table[timer_id].compare_reg_adr) = count + CONVERT_MS_TO_CLK(user_data->oneshot_ms);

    /* Update Timer Info */
    Timer_info[timer_id].status = TIMER_STATUS_BUSY;
    Timer_info[timer_id].user_data = *user_data;

    printk("System Timer Set\n");
    printk("SYS_TIMER_COUNT : 0x%08X\n", REG(vaddr_reg_base + REG_ADDR_SYS_TIMER_CONTER_LOWER32));
    printk("SYS_TIMER_COMP : 0x%08X\n", REG(vaddr_reg_base + Irq_info_table[timer_id].compare_reg_adr));

    return 0;
}

/* -----------------------------------------------------------------------------
 Function   : systimer_ioctl
 Memo       : ioctl窓口
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
static long systimer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long res;
    st_systimer_user_data user_data;

    switch (cmd)
    {
    case SYS_TIMER_SET_ONESHOT:
        // printk("SYS_TIMER_ONESHOT\n");
        if (copy_from_user(&user_data, (void __user *)arg, sizeof(user_data)))
        {
            return -EFAULT;
        }
        res = systimer_set_timer(&user_data);
        break;

    default:
        printk(KERN_WARNING "unsupported command %d\n", cmd);
        return -EFAULT;
    }
    return res;
}

/* -----------------------------------------------------------------------------
 Function   : systimer_intr
 Memo       : タイマー割り込みハンドラ
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
static irqreturn_t systimer_intr(int irq, void *dev_id)
{
    st_dev_info *dev_info = (st_dev_info *)dev_id;
    et_systimer_id timer_id = dev_info->timer_id;

    /* --- Caution 注意 ---
       割り込みハンドラ内では割り込みdisableにできないため、
       COMPAREレジスタの値がカウンターとマッチした場合は意図せず割り込みが発生する可能性がある。
       上記理由により、自分が発行した割り込みでなければすぐに抜けるが、
       要因bitはクリアしないと延々と割り込みが入るため常にクリアする。
     */

    /* Clear Interrupt (Contrl Status) */
    REG(vaddr_reg_base + REG_ADDR_SYS_TIMER_CTRL_STATUS) = Irq_info_table[timer_id].ctrl_status_bit; //write clear

    /* Check this interrut */
    /* 自分が発行した割り込みでなければすぐに抜ける */
    if (Timer_info[timer_id].status == TIMER_STATUS_IDLE)
    {
        return IRQ_HANDLED;
    }

    printk("Wake Up: %d\n", timer_id);
    printk("set_time=%d, pid=%d, signal=%d\n",
           Timer_info[timer_id].user_data.oneshot_ms,
           Timer_info[timer_id].user_data.user_pid,
           Timer_info[timer_id].user_data.user_sig);

    /* Send Signal */
    struct pid *pid = find_get_pid(Timer_info[timer_id].user_data.user_pid);
    if (pid == NULL)
    {
        printk("signal not found\n");
        return IRQ_HANDLED;
    }
    kill_pid(pid, Timer_info[timer_id].user_data.user_sig, 1);

    /* Clear Timer Info */
    Timer_info[timer_id].status = TIMER_STATUS_IDLE;
    Timer_info[timer_id].user_data = User_data_init;

    return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 Function   : systimer_init
 Memo       : 初期化(insmod時)
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
//各種システムコールに対応するハンドラテーブル
static struct file_operations mydevice_fops = {
    .open = systimer_open,
    .release = systimer_close,
    .read = systimer_read,
    .write = systimer_write,
    .unlocked_ioctl = systimer_ioctl,
    .compat_ioctl = systimer_ioctl, // for 32-bit App
};
static int systimer_init(void)
{
    printk("systimer_init\n");

    int alloc_ret = 0;
    int cdev_err = 0;
    dev_t dev;

    /* 空いているメジャー番号を確保する */
    alloc_ret = alloc_chrdev_region(&dev, MINOR_BASE, MINOR_NUM, DRIVER_NAME);
    if (alloc_ret != 0)
    {
        printk(KERN_ERR "alloc_chrdev_region = %d\n", alloc_ret);
        return ERROR_RET;
    }

    /* 取得したdev( = メジャー番号 + マイナー番号)からメジャー番号を取得して保持しておく */
    mydevice_major = MAJOR(dev);
    dev = MKDEV(mydevice_major, MINOR_BASE); /* 不要? */

    /* cdev構造体の初期化とシステムコールハンドラテーブルの登録 */
    cdev_init(&mydevice_cdev, &mydevice_fops);
    mydevice_cdev.owner = THIS_MODULE;

    /* このデバイスドライバ(cdev)をカーネルに登録する */
    cdev_err = cdev_add(&mydevice_cdev, dev, MINOR_NUM);
    if (cdev_err != 0)
    {
        printk(KERN_ERR "cdev_add = %d\n", alloc_ret);
        unregister_chrdev_region(dev, MINOR_NUM);
        return ERROR_RET;
    }

    /* このデバイスのクラス登録をする(/sys/class/mydevice/ を作る) */
    mydevice_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(mydevice_class))
    {
        printk(KERN_ERR "class_create\n");
        cdev_del(&mydevice_cdev);
        unregister_chrdev_region(dev, MINOR_NUM);
        return ERROR_RET;
    }

    /* /sys/class/mydevice/mydevice* を作る */
    for (int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++)
    {
        device_create(mydevice_class, NULL, MKDEV(mydevice_major, minor), NULL, "%s%d", DRIVER_NAME, minor);
    }

    /* Find Device Node */
    timer_node = of_find_compatible_node(NULL, NULL, SYS_TIMER_DEVICE_TREE_COMPATIBLE);
    if (timer_node == NULL)
    {
        printk("device not found\n");
        return ERROR_RET;
    }
    printk("node: %s\n", timer_node->full_name);

    /* Set Virtual Address */
    vaddr_reg_base = ioremap(REG_ADDR_BASE, VIRTUAL_PAGE_SIZE);
    printk("base: %08X", (unsigned int)vaddr_reg_base);

    /* Setup Timer */
    int irq;
    for (int i = 0; i < NUM_OF_SYS_TIMER; i++)
    {
        /* Register Irq Handler */
        irq = irq_of_parse_and_map(timer_node, Irq_info_table[i].hwirq);
        if (request_irq(irq, (void *)systimer_intr, IRQF_TIMER | IRQF_SHARED, DRIVER_INT_HANDLER, (void *)&Dev_info[i]) < 0)
        {
            printk(KERN_ERR "request_irq\n");
            return ERROR_RET;
        }

        /* Init Dev_info table */
        Dev_info[i].irq = irq;

        /* Enable Interrupt */
        REG(vaddr_reg_base + REG_ADDR_IRQ_INT_ENABLE_0_31) = Irq_info_table[i].int_enable_bit;
    }
    printk("IRQ_ENABLE: 0x%08X\n", REG(vaddr_reg_base + REG_ADDR_IRQ_INT_ENABLE_0_31));
    printk("SYS_TIMER_CTRL_STATUS: 0x%08X\n", REG(vaddr_reg_base + REG_ADDR_SYS_TIMER_CTRL_STATUS));

    return 0;
}

/* -----------------------------------------------------------------------------
 Function   : systimer_exit
 Memo       : アンロード処理(rmmod時)
 Date       : 2021.09.05
------------------------------------------------------------------------------*/
static void systimer_exit(void)
{
    printk("systimer_exit\n");

    dev_t dev = MKDEV(mydevice_major, MINOR_BASE);

    /* /sys/class/mydevice/mydevice* を削除する */
    for (int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++)
    {
        device_destroy(mydevice_class, MKDEV(mydevice_major, minor));
    }

    /* このデバイスのクラス登録を取り除く(/sys/class/mydevice/を削除する) */
    class_destroy(mydevice_class);

    /* このデバイスドライバ(cdev)をカーネルから取り除く */
    cdev_del(&mydevice_cdev);

    /* このデバイスドライバで使用していたメジャー番号の登録を取り除く */
    unregister_chrdev_region(dev, MINOR_NUM);

    /* 割り込みハンドラ解除 */
    for (int i = 0; i < NUM_OF_SYS_TIMER; i++)
    {
        free_irq(Dev_info[i].irq, (void *)&Dev_info[i]);
    }
}

module_init(systimer_init);
module_exit(systimer_exit);