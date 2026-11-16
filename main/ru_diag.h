#pragma once

#include <stdint.h>

/**
 * 3 路并发持续收发压测。
 *
 * @param seconds      压测时长
 * @param baud         波特率
 * @param irq_khz      额外 GPTimer 中断频率（kHz），0 = 关闭
 * @param hog_percent  额外忙等任务的占空比（%），0 = 关闭
 */
void ru_diag_run(uint32_t seconds, uint32_t baud, uint32_t irq_khz, uint32_t hog_percent);

/** 只启动额外的高频中断（用于在正常自测/示例上叠加干扰），0 = 关闭 */
void ru_diag_irq_start(uint32_t irq_khz);
void ru_diag_irq_stop(void);
