#ifndef RUN_LOG_H
#define RUN_LOG_H

/* 车上运行记录：跑的时候 PWLink 连不住，每 20 ms 记一帧进 CCM RAM 环形缓冲（最近 10 s），
 * 停车即冻结，停车后不断电插 PWLink 用 scripts/line_log.py 读出。复位不丢，断电才丢。
 * 纯循迹调试模式和比赛状态机（比赛、避障、找库单项）共用。 */

#include <stdbool.h>
#include <stdint.h>

#define RUN_LOG_ADDR   0x10000000UL      /* CCM RAM 开头，scripts/line_log.py 按这个地址读 */
#define RUN_LOG_N      500U              /* 500 × 20 ms = 10 s */
#define RUN_LOG_MAGIC  0x324E4C47UL      /* "GLN2"：帧格式改了就换 magic */

/* 停车原因：小于 0x100 为下面的编号；否则是指向故障原因字符串（Flash 里）的地址 */
#define RUN_LOG_STOP_NONE   0U
#define RUN_LOG_STOP_KEY    1U
#define RUN_LOG_STOP_LOST   2U
#define RUN_LOG_STOP_FAULT  3U
#define RUN_LOG_STOP_DONE   4U

void RunLog_Init(void);                   /* 上电后调一次：magic 不对才清头部，复位后保留上次记录 */
void RunLog_Start(uint8_t mode);          /* 起跑：清空并开始记录。mode 1 纯循迹，2 比赛状态机 */
void RunLog_Put(uint32_t now_ms, uint8_t state, uint8_t step);  /* 记一帧（已冻结时不记） */
void RunLog_Stop(uint32_t reason);        /* 冻结并记下停车原因 */
bool RunLog_Active(void);

#endif
