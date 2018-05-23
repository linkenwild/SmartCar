#ifndef _OV7725_H_
#define _OV7725_H_

#include <stdint.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

typedef enum
{
    H_80_W_60, /* hight:80  width:60 */
    H_120_W_160,
    H_180_W_240,
    H_240_W_320,
}ov7725_size;

void camera_init(void);
#endif

