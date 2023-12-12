KERNEL_SOURCE=/lib/modules/$(shell uname -r)/build
obj-m += virtether.o
MY_CFLAGS += -g -DDEBUG

STUB    := echoback

all: $(STUB)
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
	EXTRA_CLAGS="$(MY_CFLAGS)"

clean: 
	rm -f $(STUB) ./*.o
	make -C $(KERNEL_SOURCE) M=$(PWD) clean

$(STUB):
	gcc -g -O0 echoback.c -o $@
