
#include <asf.h>
#include "init.h"
#include "conf_board.h"

// *************************************************************************************************
// Types
// *************************************************************************************************

typedef struct rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb;


// *************************************************************************************************
// Static and global variables
// *************************************************************************************************

/// Hooked into USB code via USB_DEVICE_GET_SERIAL_NAME_POINTER #define
uint8_t serial_number[USB_DEVICE_GET_SERIAL_NAME_LENGTH];



static pwm_clock_t PWM_SETTINGS = {
    .ul_clka = 1e6,
    .ul_clkb = 0,
    .ul_mck = F_CPU
};

static usart_spi_opt_t USART_SPI_ADC =
{
    .baudrate     = 24000000,
    .char_length  = US_MR_CHRL_8_BIT,
    .spi_mode     = SPI_MODE_3,
    .channel_mode = US_MR_CHMODE_NORMAL | US_MR_INACK
};

static usart_spi_opt_t USART_SPI_DAC =
{
    .baudrate     = 24000000,
    .char_length  = US_MR_CHRL_8_BIT,
    .spi_mode     = SPI_MODE_1,
    .channel_mode = US_MR_CHMODE_NORMAL
};

static twi_options_t TWIM_CONFIG =
{
    .master_clk = F_CPU,
    .speed = 100000,
    .chip = 0,
    .smbus = 0,
};


// *************************************************************************************************
// Static function prototypes
// *************************************************************************************************

static void h_to_rgb(uint8_t h, rgb * c);


// *************************************************************************************************
// Functions
// *************************************************************************************************

/* Credit to Tod E. Kurt, ThingM, tod@todbot.com
 * Given a variable hue 'h', that ranges from 0-252,
 * set RGB color value appropriately.
 * Assumes maximum Saturation & maximum Value (brightness)
 * Performs purely integer math, no floating point.
 */
static void h_to_rgb(uint8_t h, rgb* c)
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
    tc_init(TC0, 2, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE |
                    TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR |
                    TC_CMR_BCPB_SET | TC_CMR_BCPC_CLEAR |
                    TC_CMR_EEVT_XC0 );
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
    
    pwm_channel_t PWM_CH = {0};
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


void init_build_usb_serial_number(void) {
    uint32_t uid[4];
    flash_read_unique_id(uid, 4);
    for (uint8_t i = 0; i < 16; i++) {
        serial_number[i*2+1] = "0123456789ABCDEF"[((uint8_t *)uid)[i]&0x0F];
        serial_number[i*2] = "0123456789ABCDEF"[(((uint8_t *)uid)[i]&0xF0) >> 4];
    }
}

