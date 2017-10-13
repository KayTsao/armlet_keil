#ifndef __MAG_2_HEADER_H
#define __MAG_2_HEADER_H

extern void mag_2_init(void);
extern void mag_2_measure(void);
extern bool mag_2_sleep(void);
extern void mag_2_getraw(int16_t* pbuf);

#endif //__MAG_HEADER_H

