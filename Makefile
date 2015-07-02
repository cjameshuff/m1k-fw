
build: all
	@ if [ -e ./obj/src/main.su ]; then echo "\nMaximum Stack Size: $$(cat $$(find . -name \*.su) | awk '{s+=$$2} END {print s}')\n\n"; fi

flash: cleanLocal all
	./set_bootloader.rb
	@ sleep 1
	bossac -e -w -v -b m1000.bin 
	@# @- python -c "import usb;usb.core.find(idVendor=0x0456,idProduct=0xcee2).ctrl_transfer(0x40|0x80, 0xBB, 0, 0, 1)" 2>&1 > /dev/null
	@# @- python -c "import usb;usb.core.find(idVendor=0x064b,idProduct=0x784c).ctrl_transfer(0x40|0x80, 0xBB, 0, 0, 1)" 2>&1 > /dev/null
	@# - python ./scripts/sam-ba.py

cleanLocal:
	rm -f *bin *elf *hex *lss *map *sym *.o *.d *.su

HW_VERSION=0
GIT_VERSION=$(shell git describe --always --dirty='*')

include Makefile.sam.in

cflags-gnu-y += -D'HW_VERSION=$(HW_VERSION)'
cflags-gnu-y += -D'FW_VERSION=$(GIT_VERSION)'

