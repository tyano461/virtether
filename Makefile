KERNEL_SOURCE=/lib/modules/$(shell uname -r)/build
obj-m += virtether.o
MY_CFLAGS += -g -DDEBUG

STUB   	    := echoback
SAMPLE_APP  := sample_app

all: $(STUB) $(SAMPLE_APP)
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
	EXTRA_CLAGS="$(MY_CFLAGS)"

clean: 
	rm -f $(STUB) $(SAMPLE_APP) ./*.o
	make -C $(KERNEL_SOURCE) M=$(PWD) clean

$(STUB):
	gcc -g -O0 echoback.c -o $@

$(SAMPLE_APP):
	gcc -g -O0 -lpthread sample_app.c -o $@ 
