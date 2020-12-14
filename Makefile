# verbosity: pass V=1 for verbose shell invocation
V           = 0

TOPDIR     := $(PWD)
DESTDIR    :=
PREFIX     := /usr/local
LIBDIR     := $(PREFIX)/lib

CROSS_COMPILE :=

CC         := $(CROSS_COMPILE)cc
OPT_CFLAGS := -O2
CPU_CFLAGS := -fomit-frame-pointer
DEB_CFLAGS := -Wall -g
DEF_CFLAGS :=
USR_CFLAGS :=
INC_CFLAGS :=
THR_CFLAGS :=
CFLAGS     := $(OPT_CFLAGS) $(CPU_CFLAGS) $(DEB_CFLAGS) $(DEF_CFLAGS) $(USR_CFLAGS) $(INC_CFLAGS) $(THR_CFLAGS)

LD         := $(CC)
DEB_LFLAGS := -g
USR_LFLAGS :=
LIB_LFLAGS :=
THR_LFLAGS :=
LDFLAGS    := $(DEB_LFLAGS) $(USR_LFLAGS) $(LIB_LFLAGS) $(THR_LFLAGS)

AR         := $(CROSS_COMPILE)ar
STRIP      := $(CROSS_COMPILE)strip
INSTALL    := install
BINS       := bin/bt
OBJS       :=
OBJS       += $(patsubst %.c,%.o,$(wildcard src/*.c))
OBJS       += $(patsubst %.S,%.o,$(wildcard src/*.S))

ifeq ($V,1)
Q=
cmd_AR      = $(AR)
cmd_CC      = $(CC)
cmd_LD      = $(LD)
cmd_STRIP   = $(STRIP)
cmd_INSTALL = $(INSTALL)
else
Q = @
cmd_AR      = $(Q)echo "  AR      $@";$(AR)
cmd_CC      = $(Q)echo "  CC      $@";$(CC)
cmd_LD      = $(Q)echo "  LD      $@";$(LD)
cmd_STRIP   = $(Q)echo "  STRIP   $@";$(STRIP)
cmd_INSTALL = $(Q)echo "  INSTALL $@";$(INSTALL)
endif

all: static shared bins

static: $(STATIC)

shared: $(SHARED)

bins: $(BINS)

bin/%: src/%.o
	$(Q)mkdir -p bin
	$(cmd_LD) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(cmd_CC) $(CFLAGS) -o $@ -c $<

install: install-bins

install-bins: bins
	$(cmd_STRIP) $(BINS)
	$(Q)[ -d "$(DESTDIR)$(PREFIX)/bin/." ] || mkdir -p -m 0755 $(DESTDIR)$(PREFIX)/bin
	$(cmd_INSTALL) -m 0755 $(BINS) $(DESTDIR)$(PREFIX)/bin

clean:
	$(Q)-rm -f $(BINS) $(OBJS) *.o *~
