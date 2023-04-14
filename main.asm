;
; 804_iterface_FRAM.asm
;
; Created: 20/01/2023 13:09:12
; Author : Manama
; The MB85RC256V is an FRAM (Ferroelectric Random Access Memory) chip in a configuration of 32,768words × 8 bits,
; PB0  = I2C clock (SCK)
; PB1  = I2C data  (SDA)
; PA1  = UART TX
; PA2  = UART RX
.equ fclk = 10000000
.equ BAUD = 9600
.equ fBAUD = ((64 * fclk) /(16 * BAUD)+0.5)
.equ read_data_len = 6
.equ FRAM_addressW = 0xA0
.equ FRAM_addressR = 0xA1
.def temp = r16
.def SLAVE_REG = r17




.macro micros					; macro for delay in us
ldi temp,@0
rcall delayTx1uS
.endm

.macro millis					; macro for delay in ms
ldi YL,low(@0)
ldi YH,high(@0)
rcall delayYx1mS
.endm


.dseg

BUFFER:  .byte 14			; reserve 14 bytes in SRAM
PAD1:	 .byte 1			; reserve 1 byte




.cseg
reset:
	rcall PROT_WRITE		; call procedure to increase clock speed to 10mhz
	rcall TWI_INIT			; call procedure to initialize I2C
	rcall UART_setup		; call subroutine to initialize UART
	ldi SLAVE_REG,FRAM_addressW		; load r16 with slave address (write)
	rcall TWI_START			; send I2C start
	ldi SLAVE_REG,0x00		; load slave register r17 with FRAM start address 0x0000
	rcall TWI_WRITE			; transmit 0x00 high byte
	rcall TWI_WRITE			; transmit 0x00 low byte
	ldi SLAVE_REG,'s'		; load ASCII s
	rcall TWI_WRITE			; transmit
	ldi SLAVE_REG,'a'		; load ASCII a
	rcall TWI_WRITE			; transmit
	ldi SLAVE_REG,'j'		; load ASCII j
	rcall TWI_WRITE			; transmit
	ldi SLAVE_REG,'e'		; load ASCII e
	rcall TWI_WRITE			; transmit
	ldi SLAVE_REG,'e'		; load ASCII e
	rcall TWI_WRITE			; transmit
	ldi SLAVE_REG,'v'		; load ASCII v
	rcall TWI_WRITE			; transmit
	rcall TWI_STOP			; stop I2C transmission
	millis 2000				; 2sec dely
	ldi ZL,low(BUFFER)		; load Z pointer with lower portion of the address BUFFER
	ldi ZH,high(BUFFER)		; load Z pointer with higher portion of the address BUFFER
	ldi SLAVE_REG,FRAM_addressW		; load R17 with slave write address
	rcall TWI_READ			; call subroutine to read fron slave and store in SRAM address named BUFFER
	rcall TWI_STOP			; send I2C stop command
	ldi ZL,low(BUFFER)		
	ldi ZH,high(BUFFER)
	ldi r16,6				; load r16 with 6
	sts PAD1,r16			; store 6 in SRAM address PAD1, used as  counter
Uloop:
	ld SLAVE_REG,Z+			; load r17 with value in address pointed by Z pointer
	rcall sendbyte			; call UART subroutine that transmits 1 byte
	lds r16,PAD1			; load value stored in PAD1
	dec r16					; decrease r16 by 1
	sts PAD1,r16			; store back the new value
	cpi r16,0				; compare r16 with 0
	brne Uloop				; if not 0 jump back to label Uloop
	ldi SLAVE_REG,'\n'		; load ASCII new line character
	rcall sendbyte			; transmit on UART
	ldi SLAVE_REG,'\r'		; load ASCII carriage return character
	rcall sendbyte			; transmit on UART
here:
	millis 2000				; delay of 2 seconds
	ldi ZL,low(BUFFER)		; load Z pointer with lower portion of the address BUFFER 
	ldi ZH,high(BUFFER)		; load Z pointer with higher portion of the address BUFFER
	ldi r16,6				; load r16 with 6
	sts PAD1,r16			; store 6 in SRAM address PAD1, used as  counter
	rjmp Uloop				; jump to Uloop to transmit whats in the buffer
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROTECTED WRITE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PROT_WRITE:
		ldi r16,0Xd8
		out CPU_CCP,r16
		ldi r16,0x01						; clk prescaler of 2, 20Mhz/2 = 10Mhz
		STS CLKCTRL_MCLKCTRLB,R16
		RET
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TWI_INIT:
		ldi r16,40
		sts TWI0_MBAUD,R16
		LDI R16,0b00000011			;SMEN,ENABLE
		STS TWI0_MCTRLA,R16
		LDI R16,0b00001000			;FLUSH ADDR & DATA REGISTERS
		STS TWI0_MCTRLB,R16
		LDI R16,0b00000001			;FORCE IDLE
		STS TWI0_MSTATUS,R16
		ret
		


TWI_START:
		MOV r16,SLAVE_REG			;SLAVE_REG IS R17, READ OR WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF
		SBRC R16,4					;SKIP NEXT INSTRUCTION IF RXACK IS SET
		RCALL TWI_STOP
		RET

TWI_WRITE:

		MOV R16,SLAVE_REG
		STS TWI0_MDATA,R16
		RCALL WAIT_wIF
		SBRC R16,4
		RCALL TWI_STOP
		RET

TWI_READ:
		MOV R16,SLAVE_REG			; SLAVE_REG IS R17,  WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF

		ldi SLAVE_REG,0x00			; read address of FRAM 0x0000

		MOV R16,SLAVE_REG			; send hi byte of READ_ADDRESS to THE SLAVE FROM WHICH DATA IS READ
		STS TWI0_MDATA,R16
		RCALL WAIT_WIF

		ldi SLAVE_REG,0x00			; read address of FRAM 0x0000

		MOV R16,SLAVE_REG			; send low byte of READ_ADDRESS to THE SLAVE FROM WHICH DATA IS READ
		STS TWI0_MDATA,R16
		RCALL WAIT_WIF

		ldi r16,0x00				;loading 0 in ACKACT bit enables master to send ack after reading data register
		sts TWI0_MCTRLB,r16

		ldi SLAVE_REG,FRAM_addressR

		MOV R16,SLAVE_REG			; repeated start ; READ ADDRESS SHOULD BE LOADED HERE FOR READING DATA FROM SLAVE READ_ADDRESS GIVEN ABOVE
		STS TWI0_MADDR,R16			; THIS IS A REPEATED START
		RCALL WAIT_RIF				; once data arrives in the data register the read flag is set

		ldi r16,read_data_len		; load r16 with number of bytes to be read
		cpi r16,0x02				; is num of bytes less than or greater than 2
		brlo BYYTE					; if less than 2 branch to 1BYTE as NACK+STOP will be loaded prior to read
		dec r16						; decreace one count from the total count to get loop value,NACK should be sent before the last byte read
		mov r5,r16					; move the count -1 value to counter r5
loop_read:
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,DATA IS RECIVED INTO MDATA FROM SLAVE
		ST Z+,R16					;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RCALL WAIT_RIF				;wait for read flag
		dec r5						;decrease counter after each read
		brne loop_read				;go throug loop till {count - 1} is finished
BYYTE: 
		LDI R16,0b00000111			;CLEAR ACKACT BIT BEFORE READING LAST BYTE AND ISSUE A STOP = NACK+STOP
		STS TWI0_MCTRLB,R16
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,THIS THE LAST DATA IS RECEIVED  FROM SLAVE
		ST Z+ ,R16					;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RET


TWI_STOP:
		LDI R16,0b00000011                       ;STOP
		STS TWI0_MCTRLB,R16
		RET


WAIT_WIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,6				;CHECK WIF IS SET,IF SET SKIP NEXT INSTRUCTION
		RJMP WAIT_WIF
		RET


WAIT_RIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,7
		RJMP WAIT_RIF
		RET

; ============================== Time Delay Subroutines =====================
; Name:     delayYx1mS
; Purpose:  provide a delay of (YH:YL) x 1 mS
; Entry:    (YH:YL) = delay data
; Exit:     no parameters
; Notes:    the 16-bit register provides for a delay of up to 65.535 Seconds
;           requires delay1mS

delayYx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    sbiw    YH:YL, 1                        ; update the the delay counter
    brne    delayYx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret
; ---------------------------------------------------------------------------
; Name:     delayTx1mS
; Purpose:  provide a delay of (temp) x 1 mS
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 mS
;           requires delay1mS

delayTx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    dec     temp                            ; update the delay counter
    brne    delayTx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay1mS
; Purpose:  provide a delay of 1 mS
; Entry:    no parameters
; Exit:     no parameters
; Notes:    chews up fclk/1000 clock cycles (including the 'call')

delay1mS:
    push    YL                              ; [2] preserve registers
    push    YH                              ; [2]
    ldi     YL, low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH, high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    sbiw    YH:YL, 1                        ; [2] update the the delay counter
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
    ret                                     ; [4]

; ---------------------------------------------------------------------------
; Name:     delayTx1uS
; Purpose:  provide a delay of (temp) x 1 uS with a 16 MHz clock frequency
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 uS
;           requires delay1uS

delayTx1uS:
    rcall    delay10uS                        ; delay for 1 uS
    dec     temp                            ; decrement the delay counter
    brne    delayTx1uS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay10uS
; Purpose:  provide a delay of 1 uS with a 16 MHz clock frequency ;MODIFIED TO PROVIDE 10us with 1200000cs chip by Sajeev
; Entry:    no parameters
; Exit:     no parameters
; Notes:    add another push/pop for 20 MHz clock frequency

delay10uS:
    ;push    temp                            ; [2] these instructions do nothing except consume clock cycles
    ;pop     temp                            ; [2]
    ;push    temp                            ; [2]
    ;pop     temp                            ; [2]
    ;ret                                     ; [4]
     nop
     nop
     nop
     ret

; ============================== End of Time Delay Subroutines ==============

UART_setup:
	ldi r16,0x01
	sts PORTMUX_CTRLB,r16		; PA1 = TX , PA2 = RX alternate pins selected for UART0
	lds r16,portA_dir			; PA1 set as output for TX
	ori r16,0b00000010
	sts portA_dir,r16			; 
	ldi r16,0b00000010			; 
	sts PORTA_OUTSET,r16		; PA1 set high
	ldi r16,low(fBAUD)			;load low value of fBAUD as calculated in the formula provided above
	ldi r17,high(fBAUD)			;load high value of fBAUD as calculated in the formula provided above
	sts USART0_BAUD,r16			;store low fBAUD in BAUD register
	sts USART0_BAUD + 1,r17		;store low fBAUD in BAUD register
	ldi r16,(1<<6)|(1<<7)		;Enable receive(RX) & transmit (TX)
	sts USART0_CTRLB,r16		;store TXEN & RXEN in USART_CTRLB register
	ret

sendbyte:
	lds r16,USART0_STATUS		;copy USART status register to r16
	andi r16,USART_DREIF_bm     ;(0b00100000) AND with DATA REGISTER EMPTY FLAG bitmask (position5) to check flag status 0= not empty 1= empty 
	sbrs r16,5					;skip next instruction if bit 5 is 1 (means flag set for data transmit buffer ready to receive new data )
	rjmp sendbyte				;if DREIF = 0 ,bit 5 in r16 is 0 then loop back to sendbyte until DREIF = 1
	mov r16,r17					;copy data to be transmitted from r17 to r16
	sts USART0_TXDATAL,r16		;store r16 in TXDATAL transmit data low register 
	ret

USARTread:
		lds r16,USART0_STATUS
		andi r16,0x80
		sbrs r16,7
		rjmp USARTread
		lds r17,USART0_RXDATAH
		lds r17,USART0_RXDATAL
		ret