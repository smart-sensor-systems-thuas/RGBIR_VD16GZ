CFLAGS ?= -O3 -mcpu=native -Wall -Wextra
LDLIBS = -lpthread -lm
PREFIX ?= /usr/local

vd16gz-bridge: vd16gz-bridge.c

install: vd16gz-bridge
	install -D -m 755 vd16gz-bridge $(DESTDIR)$(PREFIX)/bin/vd16gz-bridge

clean:
	rm -f vd16gz-bridge

.PHONY: install clean
