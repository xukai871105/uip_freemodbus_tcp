#ifndef __TAPDEV_H
#define __TAPDEV_H

#include <stdint.h>

void tapdev_init(void);
uint16_t tapdev_read(void);
void tapdev_send(void);

#endif
