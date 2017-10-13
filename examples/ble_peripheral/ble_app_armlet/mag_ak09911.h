#ifndef __MAG_HEADER_H
#define __MAG_HEADER_H

#define MAG_RESET_PORT	13

extern void mag_init(void);
extern void mag_measure(void);
extern bool mag_sleep(void);
extern void mag_getraw(int16_t* pbuf);

#endif //__MAG_HEADER_H

