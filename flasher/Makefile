
# target (linux, windows)
TARGET = linux
LIBNAME = libanyio
LIBANYIO = $(LIBNAME).a

CC = gcc
CFLAGS = $(OPT) $(DEBUG) $(INCLUDE)


to_flasht: to_flasht.o
	$(CC) -o to_flasht to_flasht.o $(LIBANYIO) -lpci -lm

clean :
	$(RM) *.o $(LIBANYIO) $(BIN) to_flasht 

.PHONY: install
install: $(BIN)
	install --mode=0755 --owner root --group root --dir $(DESTDIR)/bin
	install --mode=0755 --owner root --group root $(BIN) $(DESTDIR)/bin

