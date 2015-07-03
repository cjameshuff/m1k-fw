
#include <asf.h>
#include "bulk_sampling.h"
#include "board_io.h"
#include "main.h" // for frame_number
#include "conf_board.h"

#define CHUNK_SAMPLES  (256)
#define IN_PACKET_SIZE  (sizeof(uint16_t)*CHUNK_SAMPLES*4)
#define OUT_PACKET_SIZE  (sizeof(uint16_t)*CHUNK_SAMPLES*2)

typedef struct {
    uint16_t in[CHUNK_SAMPLES*4];
    uint16_t out[CHUNK_SAMPLES*2];
} bulk_buffer_t;


static bool start_timer = false;
static volatile uint16_t start_frame = 0;

static volatile bool send_in;
static volatile bool send_out;
static volatile bool sending_in;
static volatile bool sending_out;
static volatile bool sent_in;
static volatile bool sent_out;

static bulk_buffer_t buffers[2];
static bulk_buffer_t * active_buffer = &(buffers[0]);
static bulk_buffer_t * comm_buffer = &(buffers[1]);

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
    tc_stop(TC0, 2);
    
    if (period > 1)
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
        // Perform initial buffer swap: active_buffer is empty, comm_buffer has output signal data.
        bulk_buffer_t * tmp = active_buffer;
        active_buffer = comm_buffer;
        comm_buffer = tmp;
        
        // Set up pointers for first chunk of samples.
        current_chan = A;
        sample_ctr = 0;
        signal_out = active_buffer->out;
        meas_v_in = active_buffer->in;
        if(unlikely(interleave_data))
            meas_i_in = meas_v_in + 1;
        else
            meas_i_in = meas_v_in + CHUNK_SAMPLES;
        
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
        udi_vendor_bulk_in_run((uint8_t *)(comm_buffer->in), IN_PACKET_SIZE,
                               main_vendor_bulk_in_received);
    }
    if ((!sending_out) & send_out) {
        send_out = false;
        sending_out = true;
        udi_vendor_bulk_out_run((uint8_t *)(comm_buffer->out), OUT_PACKET_SIZE,
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
            signal_out += CHUNK_SAMPLES;
            meas_v_in += CHUNK_SAMPLES*2;
            meas_i_in = meas_v_in + CHUNK_SAMPLES;
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
            signal_out -= CHUNK_SAMPLES - 1;
            meas_v_in -= CHUNK_SAMPLES*2 - 1;
            meas_i_in = meas_v_in + CHUNK_SAMPLES;
        }
    }
    
    if(sample_ctr == 512)
    {
        // Set up for next packet
        bulk_buffer_t * tmp = active_buffer;
        active_buffer = comm_buffer;
        comm_buffer = tmp;
        
        sample_ctr = 0;
        signal_out = active_buffer->out;
        meas_v_in = active_buffer->in;
        if(unlikely(interleave_data))
            meas_i_in = meas_v_in + 1;
        else
            meas_i_in = meas_v_in + CHUNK_SAMPLES;
        send_in = true;
    }
    
    if(sample_ctr == 254)
        send_out = true;
}

