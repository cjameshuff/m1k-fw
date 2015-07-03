
#include <asf.h>
#include "board_io.h"
#include "conf_board.h"


// default values for DAC, pots
typedef enum ch_params {
    i0_dac = 0,
    v0_adc = 1,
    i0_adc = 2,
    p1_simv = 3,
    p2_simv = 4
} ch_params;
uint16_t def_data[5] = {26600, 0, 0, 0x30, 0x40};

chan_mode ma = DISABLED;
chan_mode mb = DISABLED;

void board_io_init(void)
{
    write_ad5663(0, def_data[i0_dac]);
    write_ad5663(1, def_data[i0_dac]);
    // set pots for a sensible default
    cpu_delay_us(100, F_CPU);
    write_ad5122(0, def_data[p1_simv], def_data[p2_simv]);
    cpu_delay_us(100, F_CPU);
    write_ad5122(1, def_data[p1_simv], def_data[p2_simv]);
    cpu_delay_us(100, F_CPU);
}

/// post-setup, write necessary configurations to hotswap and DAC
void config_hardware() {
    // continuous V&I conversion
    write_adm1177(0b00010101);
    cpu_delay_us(100, F_CPU);
    // DAC internal reference
    write_ad5663(0xFF, 0xFFFF);
}

/// write resistance values to digipots
void write_ad5122(uint32_t ch, uint8_t r1, uint8_t r2) {
    twi_packet_t p;

    uint8_t v;
    if (ch == A) {
        p.chip = 0x2f;
    }
    if (ch == B) {
        p.chip = 0x23;
    }
    p.length = 1;
    p.addr_length = 1;
    p.buffer = &v;
    p.addr[0] = 0x10;
    v = r1&0x7f;
    twi_master_write(TWI0, &p);
    p.addr[0] = 0x11;
    v = r2&0x7f;
    twi_master_write(TWI0, &p);
}

/// write controller register
void write_adm1177(uint8_t v) {
    twi_packet_t p;
    p.chip = 0x58; // 7b addr of '1177 w/ addr p grounded
    p.addr_length = 1;
    p.addr[0] = v;
    p.length = 0;
    twi_master_write(TWI0, &p);

}

/// read controller register
void read_adm1177(uint8_t* b, uint8_t ct) {
    twi_packet_t p;
    p.chip = 0x58;
    p.length = ct;
    p.buffer = b;
    p.addr_length = 0;
    twi_master_read(TWI0, &p);
}

/// synchronous write to DAC
void write_ad5663(uint8_t conf, uint16_t data) {
    USART0->US_TPR = (uint32_t)(&conf);
    USART0->US_TNPR = (uint32_t)(&data);
    pio_clear(PIOA, N_SYNC);
    cpu_delay_us(10, F_CPU);
    USART0->US_TCR = 1;
    USART0->US_TNCR = 2;
    while(!((USART0->US_CSR&US_CSR_TXEMPTY) > 0));
    cpu_delay_us(100, F_CPU);
    pio_set(PIOA, N_SYNC);
}

/// configure device channel modes
void set_mode(uint32_t chan, chan_mode m) {
    switch (chan) {
        case A: {
            switch (m) {
                case DISABLED: {
                    ma = DISABLED;
                    pio_set(PIOB, PIO_PB19); // simv
                    pio_clear(PIOB, PIO_PB2);
                    pio_set(PIOB, PIO_PB3);
                    write_ad5663(0, SWAP16(def_data[i0_dac]));
                    break;
                    }
                case SVMI: {
                    ma = SVMI;
                    pio_clear(PIOB, PIO_PB19); // modeswitch = svmi
                    pio_clear(PIOB, PIO_PB2);
                    pio_clear(PIOB, PIO_PB3); // enable output
                    break;
                }
                case SIMV: {
                    ma = SIMV;
                    pio_set(PIOB, PIO_PB19); // simv
                    pio_clear(PIOB, PIO_PB2);
                    pio_clear(PIOB, PIO_PB3); // enable output
                    break;
                }
                default: {}
            }
            break;
        }
        case B: {
            switch (m) {
                case DISABLED: {
                    mb = DISABLED;
                    pio_set(PIOB, PIO_PB20); // simv
                    pio_clear(PIOB, PIO_PB7);
                    pio_set(PIOB, PIO_PB8); // disconnect output
                    write_ad5663(1, SWAP16(def_data[i0_dac]));
                    break;
                }
                case SVMI: {
                    mb = SVMI;
                    pio_clear(PIOB, PIO_PB20); // modeswitch = svmi
                    pio_clear(PIOB, PIO_PB7);
                    pio_clear(PIOB, PIO_PB8); // enable output
                    break;
                }
                case SIMV: {
                    mb = SIMV;
                    pio_set(PIOB, PIO_PB20); // simv
                    pio_clear(PIOB, PIO_PB7);
                    pio_clear(PIOB, PIO_PB8); // enable output
                    break;
                }
                default: {}
            }
            break;
        }
        default : {}
    }
}
