#!/usr/bin/env ruby

require 'libusb'

def select_bootloader(dev)
    sernum = dev.serial_number
    handle = dev.open()
    handle.set_configuration(1)
    handle.claim_interface(0)
    
    puts "Switching device %s to bootloader..."%[sernum]
    # Clear GPNVM bit 1 to select bootloader.
    # Device will boot straight to bootloader after this.
    handle.control_transfer(
        bmRequestType: 0x40,
        bRequest: 0xBB,
        wValue: 0,
        wIndex: 0,
        timeout: 100
    )
    handle.close
end

begin
    $usb = LIBUSB::Context.new
    devs = $usb.devices(idVendor: 0x0456, idProduct: 0xCEE2) +
           $usb.devices(idVendor: 0x064B, idProduct: 0x784C)
    devs.each {|dev| select_bootloader(dev)}
    
rescue Exception => ex
    puts ex.message
    puts ex.backtrace
end

