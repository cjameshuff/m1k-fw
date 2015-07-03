
#ifndef _BOARD_IO_H_
#define _BOARD_IO_H_

#define A 0
#define B 1

typedef enum chan_mode {
    DISABLED = 0,
    SVMI = 1,
    SIMV = 2,
} chan_mode;

void board_io_init(void);

void config_hardware(void);

void write_ad5122(uint32_t ch, uint8_t r1, uint8_t r2);
void write_adm1177(uint8_t v);
void write_ad5663(uint8_t conf, uint16_t data);
void read_adm1177(uint8_t b[], uint8_t c);
void set_mode(uint32_t chan, chan_mode m);

#endif // _BOARD_IO_H_
