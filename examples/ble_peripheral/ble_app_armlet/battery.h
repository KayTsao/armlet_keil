#ifndef __BATTERY_H
#define __BATTERY_H

extern void    bat_measure(void);
extern uint8_t bat_level_percent(void);

extern uint8_t GetAdcBatteryLevel(void);

#endif //__BATTERY_H

