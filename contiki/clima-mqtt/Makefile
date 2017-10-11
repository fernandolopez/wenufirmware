PROJECT = mqtt-sensores

-include Makefile.target

# Configuration

CONTIKI_WITH_RPL = 1
CONTIKI_WITH_IPV6 = 1
RF_CHANNEL = 26
WITH_TCP=1

APPS ?= mqtt-service
CONTIKI_SOURCEFILES ?= sht25.c z1-phidgets.c
PROJECT_SOURCEFILES ?= envioTiemposIndices.c

CFLAGS += -I ../../contiki/i2c-generico


all: $(PROJECT)

test:
	make APPS='mqtt-service unit-test'\
		 PROJECT_SOURCEFILES='unit-tests.c mqtt-sensores.c sht25.c z1-phidgets.c'\
		 prueba-unit-tests.z1

CONTIKI?=../../../..


CFLAGS += -DRF_CHANNEL=$(RF_CHANNEL) -DUSER_RF_CHANNEL
CFLAGS += -DIEEE802154_CONF_PANID=0xABCD

ifneq ($(WITH_TCP),1)
CFLAGS += -DUIP_CONF_TCP=0
endif


# Platform specific configuration

ifeq ($(TARGET),)
TARGET=native
endif

MSP430_20BITS=1

SMALL=1

ifeq ($(TARGET),sky)
MSP430_20BITS=0
endif

CFLAGS+=-DMSP430_20BITS=$(MSP430_20BITS)

ifeq ($(TARGET),z1)
CFLAGS+=-mmemory-model=medium
LDFLAGS+=-mmemory-model=medium
endif

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
ifneq  ($(PROJECT_CONF),)
CFLAGS += -DUSER_PROJECT_CONF_H=\"$(PROJECT_CONF)\"
endif


include $(CONTIKI)/Makefile.include

.PHONY: all
