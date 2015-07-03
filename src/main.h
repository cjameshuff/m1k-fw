#ifndef _MAIN_H_
#define _MAIN_H_

#include <asf.h>

#define stringify(x)            #x
#define xstringify(s) stringify(s)
#define SWAP16(x)        ((((x) & 0xff00)>> 8) | (((x) & 0x00ff) << 8))

// #define cal_table_base = 0x00080000 + 256*254;

extern uint32_t frame_number;

bool main_vendor_enable(void);

void main_vendor_disable(void);

void main_sof_action(void);

void main_suspend_action(void);

void main_resume_action(void);

bool main_setup_handle(void);
bool msft_string_handle(void);


#endif
