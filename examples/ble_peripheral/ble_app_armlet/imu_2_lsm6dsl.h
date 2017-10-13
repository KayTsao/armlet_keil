#ifndef __IMU_2_HEADER_H
#define __IMU_2_HEADER_H

extern void imu_1_init(void);
extern void imu_1_measure(void);
extern bool imu_1_sleep(void);
extern bool imu_1_wakeup(void);
extern bool imu_1_is_active(void);
extern void imu_1_getraw(int16_t* pbuf);

#endif //ifndef __IMU_HEADER_H

