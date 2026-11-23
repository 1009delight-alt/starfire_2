#ifndef BSP_OPENMV_H
#define BSP_OPENMV_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OPENMV_TASK_IDLE = 0,
    OPENMV_TASK_TURN,
    OPENMV_TASK_COLOR,
    OPENMV_TASK_PARK
} openmv_task_t;

typedef struct {
    openmv_task_t target_task;      /* 主控期望模式 */
    openmv_task_t ack_task;         /* OpenMV 已确认的当前模式 */
    bool          online;           /* 心跳/收帧是否在线 */
    uint8_t       fps;              /* OpenMV 最近上报帧率 */
    uint32_t      last_rx_ms;       /* 最近收到 OpenMV 有效帧时刻 */

    /* 1. 转向牌结果 */
    char          turn_dir;         /* 0=未识别, 'L'=左转, 'R'=右转 */

    /* 2. 四色顺序结果（实时锁存，绝不被后续空帧覆盖） */
    uint8_t       color_count;      /* 已识别颜色个数 0~4 */
    char          color_seq[12];    /* 如 "R-G-B-Y" 或 "----" */
    bool          color_locked;     /* 满 4 色后永久锁定 */

    /* 3. 车库 P 牌结果 */
    bool          park_seen;        /* 是否确认看到 P 牌 */
    int16_t       park_cx;          /* -100 ~ +100 相对水平位置 */
    uint16_t      park_area;        /* 蓝牌像素面积 */
    uint32_t      park_last_ms;     /* 最近一次 PARK 帧（含 flag=0）的时间 */

    /* 诊断计数 */
    uint16_t      resync_count;       /* 因心跳任务不符而重新下发任务的次数 */
    uint16_t      color_reject_count; /* 因与已有序列不连续而丢弃的 COLOR 帧数 */
    uint16_t      bad_frame_count;    /* 格式不合法而整帧丢弃的帧数 */
} openmv_state_t;

void BspOpenmv_Init(void);
void BspOpenmv_Poll(uint32_t now_ms);

void BspOpenmv_SetTask(openmv_task_t task);
void BspOpenmv_ResetAll(void);
void BspOpenmv_Query(void);
void BspOpenmv_ClearTurn(void);    /* 清掉驱动层缓存的方向（分岔重跑时用） */

const openmv_state_t *BspOpenmv_GetState(void);
const char *BspOpenmv_TaskName(openmv_task_t task);

#endif /* BSP_OPENMV_H */
