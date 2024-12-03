# application flags:
# USTACK_BIG_ENDIAN		configures ustack to work on big endian machines
# USTACK_IP_ADDR		static configuration of IP address
# USTACK_NETMASK		static configuration of network mask
# USTACK_GW_ADDR		static configuration of gateway address
# USTACK_TAP_ADDR		TUN/TAP interface address
# USTACK_TAP_ROUTE		TUN/TAP interface default route

# configurable section
USTACK_IP_ADDR =	172.31.69.20
USTACK_NETMASK =	255.255.255.0
USTACK_GW_ADDR =	172.31.69.254
USTACK_TAP_ADDR =	172.31.69.254/24
USTACK_TAP_ROUTE =	172.31.69.0/24

# compiler flags section
AFLAGS = 	-DUSTACK_IP_ADDR=\"$(USTACK_IP_ADDR)\" \
		-DUSTACK_NETMASK=\"$(USTACK_NETMASK)\" \
		-DUSTACK_GW_ADDR=\"$(USTACK_GW_ADDR)\" \
		-DUSTACK_TAP_ADDR=\"$(USTACK_TAP_ADDR)\" \
		-DUSTACK_TAP_ROUTE=\"$(USTACK_TAP_ROUTE)\" 

CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
DUMP = arm-none-eabi-objdump
READ = arm-none-eabi-readelf
OBJ = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
AR = arm-none-eabi-ar

PROG		= firmware

PROJECT_DIR	= .
HAL_DIR		= $(PROJECT_DIR)/src/hal
CMSIS_DIR	= $(PROJECT_DIR)/src/cmsis
USBCDC_DIR	= $(PROJECT_DIR)/src/usb_cdc
COOS_DIR	= $(PROJECT_DIR)/src/coos
USTACK_DIR	= $(PROJECT_DIR)/src/ustack
APP_DIR		= $(PROJECT_DIR)/src

# this is stuff specific to this architecture
INC_DIRS  = \
	-I $(HAL_DIR) \
	-I $(CMSIS_DIR)/core \
	-I $(CMSIS_DIR)/device \
	-I $(USBCDC_DIR) \
	-I $(COOS_DIR) \
	-I $(USTACK_DIR)

# serial port
SERIAL_DEV = /dev/ttyACM0
# uart baud rate
SERIAL_BR = 115200
# uart port
SERIAL_PORT = 0

#remove unreferenced functions
CFLAGS_STRIP = -fdata-sections -ffunction-sections
LDFLAGS_STRIP = --gc-sections

# this is stuff used everywhere - compiler and flags should be declared (ASFLAGS, CFLAGS, LDFLAGS, LD_SCRIPT, CC, AS, LD, DUMP, READ, OBJ and SIZE).
MCU_DEFINES = -mcpu=cortex-m4 -mtune=cortex-m4 -mfloat-abi=hard -mthumb -fsingle-precision-constant -mfpu=fpv4-sp-d16 -Wdouble-promotion
#MCU_DEFINES = -mcpu=cortex-m4 -mtune=cortex-m4 -mfloat-abi=soft -mabi=atpcs -mthumb -fsingle-precision-constant
C_DEFINES = -D STM32F401xC -D HSE_VALUE=25000000 -D LITTLE_ENDIAN
#C_DEFINES = -D STM32F407xx -D HSE_VALUE=8000000 -D LITTLE_ENDIAN
CFLAGS = -Wall -O2 -c $(MCU_DEFINES) -mapcs-frame -fverbose-asm -nostdlib -ffreestanding $(C_DEFINES) $(INC_DIRS) -D USART_BAUD=$(SERIAL_BR) -D USART_PORT=$(SERIAL_PORT) $(CFLAGS_STRIP)

LDFLAGS = $(LDFLAGS_STRIP)
LDSCRIPT = $(HAL_DIR)/stm32f4_flash.ld

CMSIS_SRC = \
	$(CMSIS_DIR)/device/stm32f4xx_rcc.c    \
	$(CMSIS_DIR)/device/stm32f4xx_gpio.c   \
	$(CMSIS_DIR)/device/stm32f4xx_tim.c   \
	$(CMSIS_DIR)/device/stm32f4xx_adc.c   \
	$(CMSIS_DIR)/device/stm32f4xx_i2c.c \
	$(CMSIS_DIR)/device/stm32f4xx_spi.c \
	$(CMSIS_DIR)/device/stm32f4xx_usart.c \
	$(CMSIS_DIR)/device/stm32f4xx_syscfg.c \
	$(CMSIS_DIR)/device/stm32f4xx_exti.c   \
	$(CMSIS_DIR)/device/misc.c \
	$(CMSIS_DIR)/device/system_stm32f4xx.c

USBCBC_SRC = \
	$(USBCDC_DIR)/usb_bsp.c \
	$(USBCDC_DIR)/usb_core.c \
	$(USBCDC_DIR)/usb_dcd.c \
	$(USBCDC_DIR)/usb_dcd_int.c \
	$(USBCDC_DIR)/usbd_cdc_core.c \
	$(USBCDC_DIR)/usbd_cdc_vcp.c \
	$(USBCDC_DIR)/usbd_core.c \
	$(USBCDC_DIR)/usbd_desc.c \
	$(USBCDC_DIR)/usbd_ioreq.c \
	$(USBCDC_DIR)/usbd_req.c \
	$(USBCDC_DIR)/usbd_usr.c

USTACK_ETH_SRC = \
	$(USTACK_DIR)/utils.c \
	$(USTACK_DIR)/tuntap_if.c \
	$(USTACK_DIR)/eth_netif.c \
	$(USTACK_DIR)/bootp.c \
	$(USTACK_DIR)/arp.c \
	$(USTACK_DIR)/ip.c \
	$(USTACK_DIR)/icmp.c \
	$(USTACK_DIR)/udp.c

USTACK_SLIP_SRC = \
	$(USTACK_DIR)/utils.c \
	$(USTACK_DIR)/slip_netif.c \
	$(USTACK_DIR)/ip.c \
	$(USTACK_DIR)/icmp.c \
	$(USTACK_DIR)/udp.c
	
USTACK_IP_SRC = \
	$(USTACK_DIR)/utils.c \
	$(USTACK_DIR)/ip_netif.c \
	$(USTACK_DIR)/ip.c \
	$(USTACK_DIR)/icmp.c \
	$(USTACK_DIR)/udp.c

HAL_SRC = \
	$(HAL_DIR)/setjmp.s \
	$(HAL_DIR)/aeabi.s \
	$(HAL_DIR)/muldiv.c \
	$(HAL_DIR)/stm32f4_vector.c \
	$(HAL_DIR)/usart.c \
	$(HAL_DIR)/libc.c \
	$(HAL_DIR)/ieee754.c

COOS_SRC = \
	$(COOS_DIR)/coos.c
	
APPCOOS_SRC = \
	$(APP_DIR)/main_coos.c \
	$(APP_DIR)/dht.c \
	$(APP_DIR)/adc.c \
	$(APP_DIR)/pwm.c \
	$(APP_DIR)/hw_res.c
	
APP_SRC = \
	$(APP_DIR)/main.c


all: cmsis usbcdc coos ustack_eth hal app_coos link

cmsis:
	$(CC) $(CFLAGS) $(CMSIS_SRC)

usbcdc:
	$(CC) $(CFLAGS) $(USBCBC_SRC)

coos:
	$(CC) $(CFLAGS) $(COOS_SRC)
	
ustack_eth:
	$(CC) $(CFLAGS) $(AFLAGS) $(USTACK_ETH_SRC)
		
ustack_slip:
	$(CC) $(CFLAGS) $(AFLAGS) $(USTACK_SLIP_SRC)
	
ustack_ip:
	$(CC) $(CFLAGS) $(AFLAGS) $(USTACK_IP_SRC)

hal:
	$(CC) $(CFLAGS) $(HAL_SRC)

app:
	$(CC) $(CFLAGS) $(AFLAGS) $(APP_SRC)

app_coos:
	$(CC) $(CFLAGS) $(AFLAGS) $(APP_DIR)/dht.c
	$(CC) $(CFLAGS) $(AFLAGS) $(APP_DIR)/hw_res.c
	$(CC) $(CFLAGS) $(AFLAGS) $(APP_DIR)/adc.c
	$(CC) $(CFLAGS) $(AFLAGS) $(APP_DIR)/pwm.c
	$(CC) $(CFLAGS) $(AFLAGS) $(APPCOOS_SRC)

link:
	$(LD) $(LDFLAGS) -T$(LDSCRIPT) -Map $(PROG).map -o $(PROG).elf *.o 
	$(DUMP) --disassemble --reloc $(PROG).elf > $(PROG).lst	
	$(DUMP) -h $(PROG).elf > $(PROG).sec
	$(DUMP) -s $(PROG).elf > $(PROG).cnt
	$(OBJ) -O binary $(PROG).elf $(PROG).bin
	$(OBJ) -R .eeprom -O ihex $(PROG).elf $(PROG).hex
	$(SIZE) firmware.elf

flash:
	dfu-util -a 0 -s 0x08000000 -D $(PROG).bin

tuntap_host:
	gcc $(AFLAGS) -Wall $(USTACK_DIR)/tuntap_if_host.c -o tuntap_if_host

# needs setcap to allow non root access
# sudo apt-get install libcap2-bin
tuntap_cap:
	setcap cap_net_raw,cap_net_admin=eip tuntap_if_host

serial:
	stty -F ${SERIAL_DEV} ${SERIAL_BR} raw cs8 -echo

eth_up: serial
	./tuntap_if_host ${SERIAL_DEV}

slip_up: serial
	slattach -L -d -p slip -s ${SERIAL_BR} ${SERIAL_DEV} &
	sleep 1
	ifconfig sl0 ${USTACK_GW_ADDR} pointopoint ${USTACK_IP_ADDR} up mtu 576

slip_down:
	killall slattach

ip_up:
	socat -ddd -x ${SERIAL_DEV},raw,echo=0,b${SERIAL_BR},clocal TUN:${USTACK_TAP_ADDR},up
	
config_ip:
	ping ${USTACK_IP_ADDR} -s 113 -c 1

dump:
	tcpdump -l -n -S -XX -s 0 -vv -i sl0
	
sim:
	qemu-system-gnuarmeclipse -cpu cortex-m4 -machine STM32F4-Discovery -s --verbose --verbose -serial mon:stdio -kernel firmware.bin

clean:
	rm -rf .o *~ firmware.

veryclean: clean
	rm -f tuntap_if_host