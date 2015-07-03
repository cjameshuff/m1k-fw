
#include <asf.h>
#include "bulk_sampling.h"
#include "board_io.h"
#include "main.h" // for frame_number
#include "conf_board.h"

typedef union IN_packet {
    // struct {
    //     uint16_t data_a_v[256];
    //     uint16_t data_a_i[256];
    //     uint16_t data_b_v[256];
    //     uint16_t data_b_i[256];
    // };
    uint16_t data[1024];
} IN_packet;

typedef union OUT_packet {
    // struct {
    //     uint16_t data_a[256];
    //     uint16_t data_b[256];
    // };
    uint16_t data[512];
} OUT_packet;


static bool start_timer = false;
static volatile uint16_t start_frame = 0;

static uint32_t buffer_index;
static volatile uint32_t packet_index_send_in;
static volatile uint32_t packet_index_send_out;
static volatile bool send_in;
static volatile bool send_out;
static volatile bool sending_in;
static volatile bool sending_out;
static volatile bool sent_in;
static volatile bool sent_out;

static IN_packet packets_in[2];
static OUT_packet packets_out[2];

static bool interleave_data = false;

// reversed endianness from AD7682 datasheet
static uint16_t v_adc_conf = 0x20F1;
static uint16_t i_adc_conf = 0x20F7;

static uint8_t current_chan;
static uint32_t sample_ctr;
static uint8_t output_chan_id;
static uint16_t * signal_out;
static uint16_t * meas_v_in;
static uint16_t * meas_i_in;


static void main_vendor_bulk_out_received(udd_ep_status_t status,
                                          iram_size_t nb_transfered,
                                          udd_ep_id_t ep);
static void main_vendor_bulk_in_received(udd_ep_status_t status,
                                         iram_size_t nb_transfered,
                                         udd_ep_id_t ep);


void config_bulk_sampling(uint16_t period, uint16_t sync)
{
    if (period < 1)
    {
        tc_stop(TC0, 2);
    }
    else
    {
        start_timer = false;
        udd_ep_abort(UDI_VENDOR_EP_BULK_IN);
        udd_ep_abort(UDI_VENDOR_EP_BULK_OUT);
        sent_out = false;
        sent_in = false;
        sending_in = false;
        sending_out = false;
        send_out = true;
        send_in = false;
        
        buffer_index = 0;
        packet_index_send_out = 0;
        packet_index_send_in = 0;
        
        current_chan = A;
        sample_ctr = 0;
        signal_out = &packets_out[buffer_index].data[0];
        if (unlikely(interleave_data)) {
            meas_v_in = &packets_in[buffer_index].data[0];
            meas_i_in = &packets_in[buffer_index].data[1];
        }
        else {
            meas_v_in = &packets_in[buffer_index].data[0];
            meas_i_in = &packets_in[buffer_index].data[256];
        }
        
        tc_write_ra(TC0, 2, 10);
        tc_write_rb(TC0, 2, period-10);
        tc_write_rc(TC0, 2, period);
        start_frame = sync;
    }
}

void bulk_set_interleave(bool interleave)
{
    interleave_data = interleave;
}

void poll_trigger(void)
{
    if (start_timer && ((frame_number == start_frame) || (start_frame == 0)))
    {
        tc_start(TC0, 2);
        start_timer = false;
    }
}

void enable_bulk_transfers(void)
{
    main_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0, 0);
}

void handle_bulk_transfers(void)
{
    if ((!sending_in) & send_in) {
        send_in = false;
        sending_in = true;
        udi_vendor_bulk_in_run((uint8_t *)&(packets_in[packet_index_send_in]),
                               sizeof(IN_packet),
                               main_vendor_bulk_in_received);
    }
    if ((!sending_out) & send_out) {
        send_out = false;
        sending_out = true;
        udi_vendor_bulk_out_run((uint8_t *)&(packets_out[packet_index_send_out]),
                                sizeof(OUT_packet),
                                main_vendor_bulk_out_received);
    }
}

static void main_vendor_bulk_in_received(udd_ep_status_t status,
                                         iram_size_t nb_transfered,
                                         udd_ep_id_t ep)
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

static void main_vendor_bulk_out_received(udd_ep_status_t status,
                                          iram_size_t nb_transfered,
                                          udd_ep_id_t ep)
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


void TC2_Handler(void)
{
    // clear status register
    TC0->TC_CHANNEL[2].TC_SR;
    
    PIOA->PIO_SODR = N_SYNC;
    
    if(!sent_out)
        return;
    
    if(sample_ctr == 512)
    {
        // Set up for next packet
        packet_index_send_in = buffer_index;
        buffer_index ^= 1;
        
        sample_ctr = 0;
        signal_out = &packets_out[buffer_index].data[0];
        if (unlikely(interleave_data)) {
            meas_v_in = &packets_in[buffer_index].data[0];
            meas_i_in = &packets_in[buffer_index].data[1];
        }
        else {
            meas_v_in = &packets_in[buffer_index].data[0];
            meas_i_in = &packets_in[buffer_index].data[256];
        }
        send_in = true;
    }
    
    if(sample_ctr == 254)
    {
        packet_index_send_out = buffer_index^1;
        send_out = true;
    }
    
    output_chan_id = current_chan;
    USART0->US_TPR = (uint32_t)&output_chan_id;
    USART0->US_TNPR = (uint32_t)signal_out;
    USART1->US_TPR = (uint32_t)&v_adc_conf;
    USART1->US_RPR = (uint32_t)meas_v_in;
    USART2->US_TPR = (uint32_t)&i_adc_conf;
    USART2->US_RPR = (uint32_t)meas_i_in;
    
    PIOA->PIO_CODR = N_SYNC;
    USART0->US_TCR = 1;
    USART0->US_TNCR = 2;
    USART1->US_RCR = 2;
    USART1->US_TCR = 2;
    USART2->US_RCR = 2;
    USART2->US_TCR = 2;
    
    ++sample_ctr;
    
    // Set up for the next channel
    if(current_chan == A)
    {
        current_chan = B;
        if(unlikely(interleave_data)) {
            ++signal_out;
            meas_v_in += 2;
            meas_i_in = meas_v_in + 1;
        }
        else {
            signal_out += 256;
            meas_v_in += 512;
            meas_i_in = meas_v_in + 256;
        }
    }
    else
    {
        current_chan = A;
        if(unlikely(interleave_data)) {
            ++signal_out;
            meas_v_in += 2;
            meas_i_in = meas_v_in + 1;
        }
        else {
            signal_out -= 255;
            meas_v_in -= 511;
            meas_i_in = meas_v_in + 256;
        }
    }
}

