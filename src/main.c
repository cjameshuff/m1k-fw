
#include <asf.h>

#include "main.h"
#include "init.h"
#include "board_io.h"
#include "bulk_sampling.h"

#include "conf_usb.h"
#include "conf_board.h"


// *************************************************************************************************
// Static and global variables
// *************************************************************************************************
uint32_t frame_number = 0;

static const char hwversion[] = xstringify(HW_VERSION);
static const char fwversion[] = xstringify(FW_VERSION);

static volatile bool reset;
static bool main_b_vendor_enable;

static uint8_t ret_data[64];

static USB_MicrosoftCompatibleDescriptor msft_compatible = {
    .dwLength = sizeof(USB_MicrosoftCompatibleDescriptor) +
                1*sizeof(USB_MicrosoftCompatibleDescriptor_Interface),
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
        }
    }
};

// uint16_t cal_data[IFLASH0_PAGE_SIZE/sizeof(uint16_t)];


// *************************************************************************************************
// Functions
// *************************************************************************************************

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
    
    board_io_init();

    while (true) {
        handle_bulk_transfers();
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
    poll_trigger();
    if (!main_b_vendor_enable)// FIXME: this code does nothing
        return;
}

bool main_vendor_enable(void) {
    main_b_vendor_enable = true;
    enable_bulk_transfers();
    return true;
}

void main_vendor_disable(void) {
    main_b_vendor_enable = false;
}


/// WCID configuration information
/// Hooked into UDC via UDC_GET_EXTRA_STRING #define.
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
                write_ad5122((udd_g_ctrlreq.req.wValue&0xF),
                             (udd_g_ctrlreq.req.wIndex&0xFF00)>>8,
                             (udd_g_ctrlreq.req.wIndex&0xFF));
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
                bulk_set_interleave(udd_g_ctrlreq.req.wValue & 1);
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
                config_bulk_sampling(udd_g_ctrlreq.req.wValue, udd_g_ctrlreq.req.wIndex);
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

