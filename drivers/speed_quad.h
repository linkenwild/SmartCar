#ifndef __SPEED_QUAD_H
#define __SPEED_QUAD_H


void speed1_quad_init(void);
void speed2_quad_init(void);
int32_t speed1_quad_get(void);
int32_t speed2_quad_get(void);

void speed_quad_get(uint32_t *speed1, uint32_t *speed2);//不用定时器2，使用主程序中的 PIT0 1ms 定时计数
#endif
