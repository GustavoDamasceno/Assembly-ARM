
/* Clock */
.equ CM_PER_BASE,   0x44e00000  /*clock base*/
.equ GPIO1_OFFSET,  0xAC        /*offset do clock*/         


/* Watch Dog Timer */
.equ WDT_BASE, 0x44E35000


/* GPIO */
.equ GPIO1_BASE,                0x4804C000
.equ GPIO1_OE_OFFSET,           0x134
.equ GPIO1_SETDATAOUT_OFFSET,   0x194
.equ GPIO1_CLEARDATAOUT_OFFSET, 0x190
.equ GPIO1_DATAIN,              0x138

/*CONTROL MODULE*/
.equ CNTMDL_BASE,               0x44E10854

_start:
    /* init */
    mrs r0, cpsr
    bic r0, r0, #0x1F @ clear mode bits
    orr r0, r0, #0x13 @ set SVC mode
    orr r0, r0, #0xC0 @ disable FIQ and IRQ
    msr cpsr, r0






.gpio_setup:
    /* set clock for GPIO1, TRM 8.1.12.1.31 */
    ldr r0, =CM_PER_BASE
    add r0, #GPIO1_OFFSET
    mov r2, #1
    lsl r1, r2, #1
    lsl r3, r2, #18
    orr r1, r1, r3
    str r1, [r0]

	/* set pin 21 for output, led USR0, TRM 25.3.4.3 */
    ldr r0, =GPIO1_BASE
    add r0, #GPIO1_OE_OFFSET
    ldr r1, [r0]
    bic r1, r1, #(1<<21)
    bic r1, r1, #(1<<22)
    bic r1, r1, #(1<<23)
    bic r1, r1, #(1<<24)
    str r1, [r0]
    
    /* setando o pino como entrada */
    ldr r0,=GPIO1_BASE
    add r0,#GPIO1_OE_OFFSET
    ldr r1,[r0]
    orr r1,r1,#(1<<15)
    str r1,[r0]

    /* configurar os MUX dos GPIO para configurar em entrada e saida */
    ldr r0,=CNTMDL_BASE 
    mov r1,#7
    str r1,[r0] 


/* UART */
.equ UART0_BASE, 0x44E09000

.syntax unified
uart_base0              = 0x44e09000
uart_dll                = 0x00
uart_dlh                = 0x04
uart_lcr                = 0x0c
uart_lsr                = 0x14
uart_lsr_txfifoe        = 1 << 5
uart_lsr_rxfifoe        = 1 << 0
uart_sysc               = 0x54
uart_sysc_softreset     = 1 << 1
uart_syss               = 0x58
uart_syss_resetdone     = 1 << 0
uart_mdr1               = 0x20
uart_thr                = 0x00
uart_rhr                = 0x00

/* Startup Code */
/* init */
mrs r0, cpsr
bic r0, r0, #0x1F @ clear mode bits
orr r0, r0, #0x13 @ set SVC mode
orr r0, r0, #0xC0 @ disable FIQ and IRQ
msr cpsr, r0


bl .uart_init
bl .disable_wdt

/* Printando o cabeçalho */
.main:

    adr r0, linha
    bl .print_string  
    adr r0, menu
    bl .print_string
    

mov r11,#0
mov r3,#0
.scanfdoassembly:

    bl .uart_getc
    mov r5,r0
    cmp r5,#32
    beq	.proximodoscanfdoassembly   
    mov r2,r0

    
    mov r0,r2
    bl .uart_putc
    
    cmp r2, #48   // SE FOR MENOR QUE 48 NÃO É UM NUMERO
    blt .outros_erros

    cmp r2, #57  // SE FOR MAIOR QUE 57 NÃO É UM NUMERO
    bgt .outros_erros

    sub r2,r2,#48  
    
    .multiplicador_10:
        mov r7,#0
        mov r4,#10
    .loop_mul:
        add r7,r7,r11
        subs r4,r4,#1
        bne .loop_mul
        mov r11,r7
    add	r11,r11,r2

    b .scanfdoassembly

	.proximodoscanfdoassembly:

        cmp r3,#1
        beq .fimdoscanfdoassembly
    	mov r9,r11
    	mov r11,#0
    	mov r3,#1

    b .scanfdoassembly


    .fimdoscanfdoassembly:

  
    bl .uart_getc
    bl .uart_putc

	cmp r0,'+'
	bleq .somaNormal

 	cmp r0,'='   
	bleq .SomaModular

	cmp r0,'/'
	bleq .divisao

	cmp r0,'*'
	bleq .multiplicacao

	cmp r0,'<'
	bleq .deslocaparaesquerda

.resultadoleds:

    ldr r0, =GPIO1_BASE	
    add r0, #GPIO1_SETDATAOUT_OFFSET
    lsl r11, r11, #21
    str r11, [r0]


.reset:

        .botao:
            ldr r0,=GPIO1_BASE
            add r0,#GPIO1_DATAIN 

            ldr r1, [r0]
            and r1, r1, #(1<<15) // limpa os bit para ficar somente o bit 15
            lsr r1,r1,#15
            cmp r1,#1
            bne .botao

            ldr r0, =GPIO1_BASE
    		add r0, #GPIO1_CLEARDATAOUT_OFFSET
 		    ldr r1, =(15<<21)
    		str r1, [r0]

            b .main 

	.divisao:
		cmp r11,#0
		beq	.erro_da_divisao

		bl .divi_semerro
        b .resultadoleds

		.erro_da_divisao:
			adr r0, linha
			bl 	.print_string
			adr r0, erro
			bl 	.print_string

		mov r10,#3
	 	.pisca3vezes:
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<21)
	        str r1, [r0]
	       
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<22)
	        str r1, [r0]


			bl .delay
		            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<21)
	        str r1, [r0]
	            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<22)
	        str r1, [r0]


	        bl .delay


		    sub r10,r10,#1
		    cmp r10,#0
		bne .pisca3vezes

		mov r11,#0
		mov r9,#0
		mov r3,#0
	b 	.main
	

	.somaNormal:
	bl .soma_normal
		cmp r11,#15
		bgt	.errosoma
		b .resultadoleds
	.errosoma:
		adr r0, linha
		bl 	.print_string
		adr r0, erro
		bl 	.print_string

		mov r10,#3
	 	.pisca3vezessoma:
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<23)
	        str r1, [r0]
	       
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<24)
	        str r1, [r0]

			bl .delay
		            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<23)
	        str r1, [r0]
	            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<24)
	        str r1, [r0]

	        bl .delay

		    sub r10,r10,#1
		    cmp r10,#0
		bne .pisca3vezessoma

		mov r11,#0
		mov r9,#0
		mov r3,#0
	b 	.scanfdoassembly

    .outros_erros:
        adr r0, linha
		bl 	.print_string
		adr r0, erro
		bl 	.print_string

        mov r10,#5
        .pisca5vezes:
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<21)
	        str r1, [r0]
	       
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<22)
	        str r1, [r0]

            ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<23)
	        str r1, [r0]
	       
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_SETDATAOUT_OFFSET
	        ldr r1, =(1<<24)
	        str r1, [r0]

			bl .delay
		            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<21)
	        str r1, [r0]
	            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<22)
	        str r1, [r0]
		            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<23)
	        str r1, [r0]
	            
	        ldr r0, =GPIO1_BASE
	        add r0, #GPIO1_CLEARDATAOUT_OFFSET
	        ldr r1, =(1<<24)
	        str r1, [r0]

	        bl .delay

		    sub r10,r10,#1
		    cmp r10,#0
		bne .pisca5vezes
        	mov r11,#0
		    mov r9,#0
		    mov r3,#0
	b 	.scanfdoassembly

/********************************************************
Imprime uma string até o '\0'
// R0 -> Endereço da string
/********************************************************/
.print_string:
    stmfd sp!,{r0-r2,lr}
    mov r1, r0
.print1:
    ldrb r0,[r1],#1
    and r0, r0, #0xff
    cmp r0, #0
    beq .end_print1
    bl .uart_putc
    b .print1
    
.end_print1:
    ldmfd sp!,{r0-r2,pc}
/********************************************************/

/********************************************************
UART0 PUTC (Default configuration)  
********************************************************/
.uart_putc:
    stmfd sp!,{r1-r2,lr}
    ldr     r1, =UART0_BASE

.wait_tx_fifo_empty:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<5)
    cmp r2, #0
    beq .wait_tx_fifo_empty

    strb    r0, [r1]
    ldmfd sp!,{r1-r2,pc}

/********************************************************
UART0 GETC (Default configuration)  
********************************************************/
.uart_getc_string:
    stmfd sp!,{r1-r2,lr}
    ldr     r1, =UART0_BASE

.wait_rx_fifo:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<0)
    cmp r2, #0
    beq .wait_rx_fifo

    ldrb    r0, [r1]
    ldmfd sp!,{r1-r2,pc}
/********************************************************/

menu : .asciz "CALCULADORA\n\r+.Soma\n\r=.SomaModular\n\r*.Multiplicacao\n\r<.rotacao\n\r/.divisao\n\r"

linha: .asciz  " \n\r"

/********************************************************
UART0 GETC (Default configuration)  
********************************************************/
.uart_getc:

    ldr r1, =uart_base0

    /* Wait for transmit hold register to fill (RXFIFOE == 1) */
1:
    ldr r2, [r1, uart_lsr]
    tst r2, uart_lsr_rxfifoe
    beq 1b

    /* Input character */
    ldr r0, [r1, uart_rhr]

    bx lr

/********************************************************/


/********************************************************/

.uart_init:

    ldr r0, =uart_base0

    /* Reset: set UARTi.UART_SYSC[1] SOFTRESET to 1 */
    mov r1, uart_sysc_softreset
    str r1, [r0, uart_sysc]
    
    /* Wait for reset: poll for UARTi.UART_SYSS[0] RESETDONE == 1 */
1:
    ldr r1, [r0, uart_syss]
    cmp r1, uart_syss_resetdone
    bne 1b

    mov r1, 0x83
    str r1, [r0, uart_lcr]

    /* Set Baud Rate to 115200: assume DLH == 0 (default), set DLL = 0x1A */
    mov r1, 0x1a
    str r1, [r0, uart_dll]

    /* Enable UART 16x mode: set UARTi.UART_MDR1[2:0] MODESELECT to 0 */
    mov r1, 0
    str r1, [r0, uart_mdr1]

    /* Switch to Operational mode: clear UARTi.UART_LCR[7] DIV_EN */
    ldr r1, [r0, uart_lcr]
    bic r1, r1, 0x80
    str r1, [r0, uart_lcr]

    bx lr

 /********************************************************/


/********************************************************
UART0 PUTC (Default configuration)  
********************************************************/
.uart_putc_char:

    ldr r1, =uart_base0

    /* Wait for transmit hold register to clear (TXFIFOE == 1) */
1:
    ldr r2, [r1, uart_lsr]
    tst r2, uart_lsr_txfifoe
    beq 1b

    /* Output character */
    str r0, [r1, uart_thr]

    bx lr



		
.soma_normal:
	add r11,r11,r9 
bx lr


.SomaModular:
	add r11,r11,r9 
		.loop2: 
			cmp     r11,#16
			blt .menor_soma
			sub	r11,r11,#16
		bge	.loop2
		.menor_soma:
bx lr


.divi_semerro:
	mov r3,#0
	.loop: 
		subs	r9,r9,r11 
		addge	r3,r3,#1
	bge		.loop
	mov r11,r3 
bx lr




.multiplicacao:
    mov r7,#0
    mov r4,r9
    .loop_multiplica:
    add r7,r7,r11
    subs r4,r4,#1
    bne .loop_multiplica
    mov r11,r7

    .loop3: //mod 16
		subs	r11,r11,#16
		addge	r3,r3,#1
	bge	.loop3
	add r4,r11,#16  
	mov r11,r4 
bx lr



.deslocaparaesquerda:	
	mov		r1,r9
	mov		r2,r11
	cmp 	r11,#0
	beq	.finaldodeslocamento
	.loop_desloca:
		mov		r3,#0
		mov		r1,r1,lsl #1
		mov		r3,r1
		mov		r3,r3,lsr #4
		orr		r1,r1,r3
		and		r1,r1,#15
		sub 	r2,r2,#1
		cmp 	r2,#0
		bne		.loop_desloca
	.finaldodeslocamento:
	mov 	r11,r1
bx lr



.delay:
	    mov r4, #0xfffffff
	    .delayleds:
            sub r4, r4, #1
            cmp r4, #0
	    bne .delayleds
bx lr 

/********************************************************/
.disable_wdt:
    /* TRM 20.4.3.8 */
    stmfd sp!,{r0-r1,lr}
    ldr r0, =WDT_BASE

    ldr r1, =0xAAAA
    str r1, [r0, #0x48]
    bl .poll_wdt_write

    ldr r1, =0x5555
    str r1, [r0, #0x48]
    bl .poll_wdt_write

    ldmfd sp!,{r0-r1,pc}

.poll_wdt_write:
    ldr r1, [r0, #0x34]
    and r1, r1, #(1<<4)
    cmp r1, #0
    bne .poll_wdt_write
    bx lr
/********************************************************/

erro: .asciz "ERRO\n\r"





