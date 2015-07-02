#include <asf.h>
#include "main.h"
#include "conf_usb.h"
#include "conf_board.h"

const char hwversion[] = xstringify(HW_VERSION);
const char fwversion[] = xstringify(FW_VERSION);
chan_mode ma = DISABLED;
chan_mode mb = DISABLED;
static int interleave_data;

// default values for DAC, pots
uint16_t def_data[5] = {26600, 0, 0, 0x30, 0x40};

// reversed endianness from AD7682 datasheet
uint16_t v_adc_conf = 0x20F1;
uint16_t i_adc_conf = 0x20F7;

uint8_t da = 0;
uint8_t db = 1;
uint32_t frame_number = 0;
pwm_channel_t PWM_CH;

static pwm_clock_t PWM_SETTINGS = {
    .ul_clka = 1e6,
    .ul_clkb = 0,
    .ul_mck = F_CPU
};

static  usart_spi_opt_t USART_SPI_ADC =
{
    .baudrate    = 24000000,
    .char_length   = US_MR_CHRL_8_BIT,
    .spi_mode     = SPI_MODE_3,
    .channel_mode  = US_MR_CHMODE_NORMAL | US_MR_INACK
};

static  usart_spi_opt_t USART_SPI_DAC =
{
    .baudrate    = 24000000,
    .char_length   = US_MR_CHRL_8_BIT,
    .spi_mode     = SPI_MODE_1,
    .channel_mode  = US_MR_CHMODE_NORMAL
};

static twi_options_t TWIM_CONFIG =
{
    .master_clk = F_CPU,
    .speed = 100000,
    .chip = 0,
    .smbus = 0,
};

/* Credit to Tod E. Kurt, ThingM, tod@todbot.com
 * Given a variable hue 'h', that ranges from 0-252,
 * set RGB color value appropriately.
 * Assumes maximum Saturation & maximum Value (brightness)
 * Performs purely integer math, no floating point.
 */
void h_to_rgb(uint8_t h, rgb* c)
{
    uint8_t hd = h / 42;   // 42 == 252/6,  252 == H_MAX
    uint8_t hi = hd % 6;   // gives 0-5
    uint8_t f = h % 42;
    uint8_t fs = f * 6;
    switch( hi ) {
        case 0:
            c->r = 252;    c->g = fs;     c->b = 0;
           break;
        case 1:
            c->r = 252-fs; c->g = 252;    c->b = 0;
            break;
        case 2:
            c->r = 0;      c->g = 252;    c->b = fs;
            break;
        case 3:
            c->r = 0;      c->g = 252-fs; c->b = 252;
            break;
        case 4:
            c->r = fs;     c->g = 0;      c->b = 252;
            break;
        case 5:
            c->r = 252;    c->g = 0;      c->b = 252-fs;
            break;
    }
}

void init_build_usb_serial_number(void) {
    uint32_t uid[4];
    flash_read_unique_id(uid, 4);
    for (uint8_t i = 0; i < 16; i++) {
        serial_number[i*2+1] = "0123456789ABCDEF"[((uint8_t *)uid)[i]&0x0F];
        serial_number[i*2] = "0123456789ABCDEF"[(((uint8_t *)uid)[i]&0xF0) >> 4];
    }
}

void TC2_Handler(void) {
    // clear status register
    ((TC0)->TC_CHANNEL+2)->TC_SR;
    PIOA->PIO_SODR = N_SYNC;
    if ((!sent_out))
        return;
    switch (current_chan) {
        case A: {
            switch (slot_offset) {
                case 127: {
                    packet_index_send_out = packet_index_out^1;
                    send_out = true;
                    break;
                }
                case 256: {
                    slot_offset = 0;
                    packet_index_send_in = packet_index_in;
                    packet_index_in ^= 1;
                    packet_index_out ^= 1;
                    send_in = true;
                }
            }
            USART0->US_TPR = (uint32_t)(&da);
            if (unlikely(interleave_data)) {
                USART0->US_TNPR = (uint32_t)(&packets_out[packet_index_out].data[slot_offset*2+0]);
                USART1->US_TPR = (uint32_t)(&v_adc_conf);
                USART1->US_RPR = (uint32_t)(&packets_in[packet_index_in].data[slot_offset*4+0]);
                USART2->US_TPR = (uint32_t)(&i_adc_conf);
                USART2->US_RPR = (uint32_t)(&packets_in[packet_index_in].data[slot_offset*4+1]);
            } else {
                USART0->US_TNPR = (uint32_t)(&packets_out[packet_index_out].data_a[slot_offset]);
                USART1->US_TPR = (uint32_t)(&v_adc_conf);
                USART1->US_RPR = (uint32_t)(&packets_in[packet_index_in].data_a_v[slot_offset]);
                USART2->US_TPR = (uint32_t)(&i_adc_conf);
                USART2->US_RPR = (uint32_t)(&packets_in[packet_index_in].data_a_i[slot_offset]);
            }
            PIOA->PIO_CODR = N_SYNC;
            USART0->US_TCR = 1;
            USART0->US_TNCR = 2;
            USART1->US_RCR = 2;
            USART1->US_TCR = 2;
            USART2->US_RCR = 2;
            USART2->US_TCR = 2;
            current_chan ^= true;
            break;
        }
        case B: {
            USART0->US_TPR = (uint32_t)(&db);
            if (unlikely(interleave_data)) {
                USART0->US_TNPR = (uint32_t)(&packets_out[packet_index_out].data[slot_offset*2+1]);
                USART1->US_TPR = (uint32_t)(&i_adc_conf);
                USART1->US_RPR = (uint32_t)(&packets_in[packet_index_in].data[slot_offset*4+3]);
                USART2->US_TPR = (uint32_t)(&v_adc_conf);
                USART2->US_RPR = (uint32_t)(&packets_in[packet_index_in].data[slot_offset*4+2]);
            } else {
                USART0->US_TNPR = (uint32_t)(&packets_out[packet_index_out].data_b[slot_offset]);
                USART1->US_TPR = (uint32_t)(&i_adc_conf);
                USART1->US_RPR = (uint32_t)(&packets_in[packet_index_in].data_b_i[slot_offset]);
                USART2->US_TPR = (uint32_t)(&v_adc_conf);
                USART2->US_RPR = (uint32_t)(&packets_in[packet_index_in].data_b_v[slot_offset]);
            }
            PIOA->PIO_CODR = N_SYNC;
            USART0->US_TCR = 1;
            USART0->US_TNCR = 2;
            USART1->US_TCR = 2;
            USART1->US_RCR = 2;
            USART2->US_TCR = 2;
            USART2->US_RCR = 2;
            slot_offset += 1;
            current_chan ^= true;
        }
    }
}

/// initialise SAM subsystems for M1K operation
void init_hardware(void) {
// enable peripheral clock access
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);
    pmc_enable_periph_clk(ID_TWI0);
    pmc_enable_periph_clk(ID_USART0);
    pmc_enable_periph_clk(ID_USART1);
    pmc_enable_periph_clk(ID_USART2);
    pmc_enable_periph_clk(ID_TC0);
    pmc_enable_periph_clk(ID_TC1);
    pmc_enable_periph_clk(ID_PWM);
    pmc_enable_periph_clk(ID_TC2);


// GPIOs as inputs - OR of PIO_PA0..PA3
    pio_configure(PIOA, PIO_INPUT, 0x0F, PIO_DEFAULT);
// configurable pull resistors disabled
    pio_configure(PIOA, PIO_INPUT, 0xF0, PIO_DEFAULT);

// LED
    pio_configure(PIOA, PIO_PERIPH_B, LED_B, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_B, LED_G, PIO_DEFAULT);
    pio_configure(PIOB, PIO_PERIPH_B, LED_R, PIO_DEFAULT);

    pio_configure(PIOB, PIO_OUTPUT_1, PWR, PIO_DEFAULT);

// SDA
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA9A_TWD0, PIO_DEFAULT);
// SCL
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA10A_TWCK0, PIO_DEFAULT);

// DAC
    pio_configure(PIOA, PIO_PERIPH_B, N_LDAC, PIO_DEFAULT);
    pio_configure(PIOA, PIO_OUTPUT_1, N_CLR, PIO_DEFAULT);
    pio_configure(PIOA, PIO_OUTPUT_1, N_SYNC, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA17A_SCK0, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA18A_TXD0, PIO_DEFAULT);

// CHA_ADC
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA21A_RXD1, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA20A_TXD1, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_B, PIO_PA24B_SCK1, PIO_DEFAULT);

// ADC_CNV
    pio_configure(PIOA, PIO_PERIPH_B, CNV, PIO_DEFAULT);

// CHB_ADC
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA22A_TXD2, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_A, PIO_PA23A_RXD2, PIO_DEFAULT);
    pio_configure(PIOA, PIO_PERIPH_B, PIO_PA25B_SCK2, PIO_DEFAULT);

// CHA_SWMODE
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB19, PIO_DEFAULT);
// CHB_SWMODE
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB20, PIO_DEFAULT);

// 50o to 2v5
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB0, PIO_DEFAULT);
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB5, PIO_DEFAULT);
// 50o to gnd
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB1, PIO_DEFAULT);
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB6, PIO_DEFAULT);
// feedback / sense
    pio_configure(PIOB, PIO_OUTPUT_0, PIO_PB2, PIO_DEFAULT);
    pio_configure(PIOB, PIO_OUTPUT_0, PIO_PB7, PIO_DEFAULT);
// output
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB3, PIO_DEFAULT);
    pio_configure(PIOB, PIO_OUTPUT_1, PIO_PB8, PIO_DEFAULT);


    usart_init_spi_master(USART0, &USART_SPI_DAC, F_CPU);
    usart_enable_tx(USART0);
    usart_init_spi_master(USART1, &USART_SPI_ADC, F_CPU);
    usart_enable_tx(USART1);
    usart_enable_rx(USART1);
    usart_init_spi_master(USART2, &USART_SPI_ADC, F_CPU);
    usart_enable_tx(USART2);
    usart_enable_rx(USART2);
// enable pdc for USARTs
    USART0->US_PTCR = US_PTCR_TXTEN;
    USART1->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;
    USART2->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;


// 100khz I2C
    twi_reset(TWI0);
    twi_enable_master_mode(TWI0);
    twi_master_init(TWI0, &TWIM_CONFIG);


// CLOCK1 = MCLK/2
// RA takes LDAC H->L
// RB takes CNV L->H
// RC takes CNV H->L, LDAC L->H
    tc_init(TC0, 2, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_BCPB_SET | TC_CMR_BCPC_CLEAR | TC_CMR_EEVT_XC0 );
// CPAS doesn't matter, CPCS is triggered post-conversion
    tc_enable_interrupt(TC0, 2, TC_IER_CPCS);
    NVIC_EnableIRQ(TC2_IRQn);


// set RGB LED to hue generated from UID
    uint32_t uid[4];
    rgb c;
    flash_read_unique_id(uid, 4);
    uint8_t h = uid[3] % 252;
    h_to_rgb(h, &c);


    pwm_channel_disable(PWM, PWM_CHANNEL_0); // PA28 - blue - PWMH0
    pwm_channel_disable(PWM, PWM_CHANNEL_1); // PA29 - green - PWMH1
    pwm_channel_disable(PWM, PWM_CHANNEL_2); // PB15 - red - PWMH2
    pwm_init(PWM, &PWM_SETTINGS);
    PWM_CH.ul_prescaler = PWM_CMR_CPRE_CLKA;
    PWM_CH.ul_period = 256;
    PWM_CH.ul_duty = c.b<<3;
    PWM_CH.channel = PWM_CHANNEL_0;
    pwm_channel_init(PWM, &PWM_CH);
    PWM_CH.ul_duty = c.g<<3;
    PWM_CH.channel = PWM_CHANNEL_1;
    pwm_channel_init(PWM, &PWM_CH);
    PWM_CH.ul_duty = c.r<<3;
    PWM_CH.channel = PWM_CHANNEL_2;
    pwm_channel_init(PWM, &PWM_CH);
    pwm_channel_enable(PWM, PWM_CHANNEL_0);
    pwm_channel_enable(PWM, PWM_CHANNEL_1);
    pwm_channel_enable(PWM, PWM_CHANNEL_2);

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

int main(void)
{
    irq_initialize_vectors();
    cpu_irq_enable();
    sysclk_init();
    // convert chip UID to ascii string of hex representation
    init_build_usb_serial_number();
    // enable WDT for "fairly short"
    wdt_init(WDT, WDT_MR_WDRSTEN, 50, 50);
    // setup peripherals
    init_hardware();
    // start USB
    cpu_delay_us(100, F_CPU);

    udc_detach();
    udc_stop();
    udc_start();
    cpu_delay_us(10, F_CPU);
    udc_attach();
    write_ad5663(0, def_data[i0_dac]);
    write_ad5663(1, def_data[i0_dac]);
    // set pots for a sensible default
    cpu_delay_us(100, F_CPU);
    write_ad5122(0, def_data[p1_simv], def_data[p2_simv]);
    cpu_delay_us(100, F_CPU);
    write_ad5122(1, def_data[p1_simv], def_data[p2_simv]);
    cpu_delay_us(100, F_CPU);

    while (true) {
        if ((!sending_in) & send_in) {
            send_in = false;
            sending_in = true;
            udi_vendor_bulk_in_run((uint8_t *)&(packets_in[packet_index_send_in]), sizeof(IN_packet), main_vendor_bulk_in_received);
        }
        if ((!sending_out) & send_out) {
            send_out = false;
            sending_out = true;
            udi_vendor_bulk_out_run((uint8_t *)&(packets_out[packet_index_send_out]), sizeof(OUT_packet), main_vendor_bulk_out_received);
        }
        if (!reset)
            wdt_restart(WDT);
        else
            udc_detach();
    }
}

void main_suspend_action(void) { }

void main_resume_action(void) { }

void main_sof_action(void) {
    frame_number = UDPHS->UDPHS_FNUM;
    if (start_timer & ((frame_number == start_frame)|(start_frame == 0))){
        tc_start(TC0, 2);
        start_timer = false;
    }
    if (!main_b_vendor_enable)
        return;
}

bool main_vendor_enable(void) {
    main_b_vendor_enable = true;
    main_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0, 0);
    return true;
}

void main_vendor_disable(void) {
    main_b_vendor_enable = false;
}


/// WCID configuration information
bool msft_string_handle(void) {
    uint8_t udi_msft_magic[] = "MSFT1000";

    struct extra_strings_desc_t{
        usb_str_desc_t header;
        le16_t string[sizeof(udi_msft_magic)];
    };

    static UDC_DESC_STORAGE struct extra_strings_desc_t extra_strings_desc = {
        .header.bDescriptorType = USB_DT_STRING
    };

    uint8_t i;
    uint8_t *str;
    uint8_t str_lgt=0;

    if ((udd_g_ctrlreq.req.wValue & 0xff) == 0xEE) {
        str_lgt = sizeof(udi_msft_magic)-1;
        str = udi_msft_magic;
    }
    else {
        return false;
    }

    if (str_lgt!=0) {
        for( i=0; i<str_lgt; i++) {
            extra_strings_desc.string[i] = cpu_to_le16((le16_t)str[i]);
        }
        extra_strings_desc.header.bLength = 2+ (str_lgt)*2;
        udd_g_ctrlreq.payload_size = extra_strings_desc.header.bLength;
        udd_g_ctrlreq.payload = (uint8_t *) &extra_strings_desc;
    }

    // if the string is larger than request length, then cut it
    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength) {
        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
    }
    return true;
}
static USB_MicrosoftCompatibleDescriptor msft_compatible = {
    .dwLength = sizeof(USB_MicrosoftCompatibleDescriptor) + 1*sizeof(USB_MicrosoftCompatibleDescriptor_Interface),
    .bcdVersion = 0x0100,
    .wIndex = 0x0004,
    .bCount = 1,
    .reserved = {0, 0, 0, 0, 0, 0, 0},
    .interfaces = {
            {
            .bFirstInterfaceNumber = 0,
            .reserved1 = 0,
            .compatibleID = "WINUSB\0\0",
            .subCompatibleID = {0, 0, 0, 0, 0, 0, 0, 0},
            .reserved2 = {0, 0, 0, 0, 0, 0},
            },
        }
    };

/// handle control transfers
bool main_setup_handle(void) {
    uint8_t* ptr = 0;
    uint16_t size = 0;
    if (Udd_setup_type() == USB_REQ_TYPE_VENDOR) {
        switch (udd_g_ctrlreq.req.bRequest) {
            case 0x00: { // Info
                switch(udd_g_ctrlreq.req.wIndex){
                    case 0:
                        ptr = (uint8_t*)hwversion;
                        size = sizeof(hwversion);
                        break;
                    case 1:
                        ptr = (uint8_t*)fwversion;
                        size = sizeof(fwversion);
                        break;
                }
                break;
            }
            /// read ADM1177
            case 0x17: {
                size = udd_g_ctrlreq.req.wIndex&0xFF;
                read_adm1177((uint8_t*)(&ret_data), size);
                ptr = (uint8_t*)&ret_data;
                break;
            }
            /// Set pin 0
            case 0x50: {
                int32_t low = udd_g_ctrlreq.req.wValue & 0x1F;
                bool PB = udd_g_ctrlreq.req.wValue > 0x1F;
                pio_set_output(PB ? PIOB: PIOA, 1<<low, LOW, DISABLE, DISABLE);
                break;
            }
            /// Set pin 1
            case 0x51: {
                int32_t low = udd_g_ctrlreq.req.wValue & 0x1F;
                bool PB = udd_g_ctrlreq.req.wValue > 0x1F;
                pio_set_output(PB ? PIOB: PIOA, 1<<low, HIGH, DISABLE, DISABLE);
                break;
            }
            /// Set pin input, get pin value
            case 0x91: {
                int32_t low = udd_g_ctrlreq.req.wValue & 0x1F;
                bool PB = udd_g_ctrlreq.req.wValue > 0x1F;
                pio_set_input(PB ? PIOB: PIOA, 1<<low, 0);
                ret_data[0] = pio_get_pin_value(udd_g_ctrlreq.req.wValue&0xFF);
                ptr = (uint8_t*)&ret_data;
                size = 1;
                break;
            }
            /// set channel mode - wValue = channel, wIndex = value
            case 0x53: {
                set_mode(udd_g_ctrlreq.req.wValue&0xF, udd_g_ctrlreq.req.wIndex&0xF);
                break;
            }
            /// set potentiometer - wValue = channel, wIndex = values (0xAABB)
            case 0x59: {
                write_ad5122((udd_g_ctrlreq.req.wValue&0xF), (udd_g_ctrlreq.req.wIndex&0xFF00)>>8, (udd_g_ctrlreq.req.wIndex&0xFF));
                break;
            }
            /// erase and reset to bootloader
            case 0xBB: {
                flash_clear_gpnvm(1);
                reset = true;
                break;
            }
            /// setup hardware
            case 0xCC: {
                config_hardware();
                break;
            }
            /// Change interleave mode
            case 0xDD: {
                interleave_data = udd_g_ctrlreq.req.wValue & 1;
                break;
            }
            /// get USB microframe
            case 0x6F: {
                ret_data[0] = frame_number&0xFF;
                ret_data[1] = frame_number>>8;
                ptr = (uint8_t*)&ret_data;
                size = 2;
                break;
            }
            /// configure sampling
            case 0xC5: {
                if (udd_g_ctrlreq.req.wValue < 1) {
                    tc_stop(TC0, 2);
                }
                else {
                    // how much state to reset?
                    udd_ep_abort(UDI_VENDOR_EP_BULK_IN);
                    udd_ep_abort(UDI_VENDOR_EP_BULK_OUT);
                    current_chan = A;
                    sent_out = false;
                    sent_in = false;
                    sending_in = false;
                    sending_out = false;
                    send_out = true;
                    send_in = false;
                    slot_offset = 0;
                    packet_index_in = 0;
                    packet_index_out = 0;
                    packet_index_send_out = 0;
                    packet_index_send_in = 0;
                    // so much
                    tc_write_ra(TC0, 2, 10);
                    tc_write_rb(TC0, 2, udd_g_ctrlreq.req.wValue-10);
                    tc_write_rc(TC0, 2, udd_g_ctrlreq.req.wValue);
                    start_frame = udd_g_ctrlreq.req.wIndex;
                }
                break;
            }
            /// windows compatible ID handling for autoinstall
            case 0x30: {
                if (udd_g_ctrlreq.req.wIndex == 0x04) {
                    ptr = (uint8_t*)&msft_compatible;
                    size = (udd_g_ctrlreq.req.wLength);
                    if (size > msft_compatible.dwLength) {
                        size = msft_compatible.dwLength;
                    }
                }
                else {
                    return false;
                }
                break;
            }
        }
    }
    udd_g_ctrlreq.payload_size = size;
    if ( size == 0 ) {
        udd_g_ctrlreq.callback = 0;
        udd_g_ctrlreq.over_under_run = 0;
    }
    else {
        udd_g_ctrlreq.payload = ptr;
    }
    return true;
}

void main_vendor_bulk_in_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep)
{
    UNUSED(nb_transfered);
    UNUSED(ep);
    if (UDD_EP_TRANSFER_OK != status) {
        return;
    }
    else {
        sending_in = false;
    }
}

void main_vendor_bulk_out_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep)
{
    UNUSED(ep);
    if (UDD_EP_TRANSFER_OK != status) {
        return;
    }
    else {
        if (sent_out == false) {
            start_timer = true;
        }
        sent_out = true;
        sending_out = false;
    }
}
