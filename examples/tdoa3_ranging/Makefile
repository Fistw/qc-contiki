CONTIKI_PROJECT = tdoa3_ranging
all: $(CONTIKI_PROJECT)

# Add EVB1000 platform folder
# TARGETDIRS += ../../platform

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

# No network stack for the moment
CONTIKI_WITH_IPV4 = 0
CONTIKI_WITH_IPV6 = 0
CONTIKI_WITH_RIME = 1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
