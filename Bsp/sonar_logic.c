#include "sonar_logic.h"

#include <string.h>

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

void SonarLogic_Reset(sonar_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->raw_mm = SONAR_FAR_MM;
    s->dist_mm = SONAR_FAR_MM;
    s->med_buf[0] = SONAR_FAR_MM;
    s->med_buf[1] = SONAR_FAR_MM;
    s->med_buf[2] = SONAR_FAR_MM;
}

void SonarLogic_FeedEcho(sonar_state_t *s, uint32_t echo_us, uint32_t now_ms,
                         float mm_per_us, uint16_t max_mm, uint16_t trig_mm)
{
    float mm_f = (float)echo_us * mm_per_us;
    uint16_t mm = ((mm_f > (float)max_mm) || (mm_f >= (float)SONAR_FAR_MM)) ? SONAR_FAR_MM : (uint16_t)(mm_f + 0.5f);

    s->online = true;
    s->echo_us = (echo_us > 0xFFFFU) ? 0xFFFFU : (uint16_t)echo_us;
    s->raw_mm = mm;
    s->last_update_ms = now_ms;

    /* 超量程也进中值：前方空旷是正常读数；超声偶尔漏一次回波（斜面、软表面）由中值滤掉 */
    s->med_buf[s->med_idx] = mm;
    s->med_idx = (uint8_t)((s->med_idx + 1U) % 3U);
    s->dist_mm = median3(s->med_buf[0], s->med_buf[1], s->med_buf[2]);
    s->valid = (s->dist_mm < SONAR_FAR_MM);

    if (s->valid && (s->dist_mm <= trig_mm)) {
        if (s->near_hits < 255U) { s->near_hits++; }
    } else {
        s->near_hits = 0U;
    }
}
