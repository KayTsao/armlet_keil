#ifndef __IMU_HEADER_H
#define __IMU_HEADER_H

extern void imu_init(void);
extern void imu_measure(void);
extern bool imu_sleep(void);
extern bool imu_wakeup(void);
extern bool imu_is_active(void);
extern void imu_getraw(int16_t* pbuf);

#endif //ifndef __IMU_HEADER_H

