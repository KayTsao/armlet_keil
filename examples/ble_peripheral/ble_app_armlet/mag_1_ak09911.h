#ifndef __MAG_1_HEADER_H
#define __MAG_1_HEADER_H

extern void mag_1_init(void);
extern void mag_1_measure(void);
extern bool mag_1_sleep(void);
extern void mag_1_getraw(int16_t* pbuf);

#endif //__MAG_HEADER_H

