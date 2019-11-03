CROSS_COMPILE ?= arm-none-eabi-

all: calculadora.s
	$(CROSS_COMPILE)as calculadora.s -o lab1.o
	$(CROSS_COMPILE)objcopy lab1.o lab1.bin -O binary
	$(CROSS_COMPILE)objdump -D -b binary -marm lab1.bin > lab1.lst
	cp *.bin /tftpboot


clean:
	rm *.o *.bin *.lst
