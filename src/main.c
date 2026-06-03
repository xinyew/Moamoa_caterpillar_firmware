#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_MYFUNC
#include "myfunc.h"
#endif

LOG_MODULE_REGISTER(Less4_Exer2, LOG_LEVEL_DBG);

int main(void)
{
    while (1)
    {
        printk("Hello World!\n");
#ifdef CONFIG_MYFUNC
        int a = 1;
        int b = 3;
        printk("%d + %d = %d\n", a, b, sum(a, b));
#else
        printk("MYFUNC is not configured\n");
#endif
        int exercise_num = 2;
        uint8_t data[] = {0x00, 0x01, 0x02, 0x03,
                          0x04, 0x05, 0x06, 0x07,
                          'H', 'e', 'l', 'l', 'o'};
        // Printf-like messages
        LOG_INF("nRF Connect SDK Fundamentals");
        LOG_INF("Exercise %d", exercise_num);
        LOG_DBG("A log message in debug level");
        LOG_WRN("A log message in warning level!");
        LOG_ERR("A log message in Error level!");
        // Hexdump some data
        LOG_HEXDUMP_INF(data, sizeof(data), "Sample Data!");

        k_msleep(1000);
    }
}