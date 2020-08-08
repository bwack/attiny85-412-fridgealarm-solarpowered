; Fridge alarm, attiny85

.nolist
.include "tn85def.inc"
.list

.equ IR_LED	= PB1
.equ SENS_PWR	= PB0
.equ SENS_IN	= PB3
.equ BUZZER	= PB4

.def tmp	= r16
.def tmp2	= r17
.def tmp3	= r18
.def tmpisr     = r19
.def loopCt	= r20		; delay loop count
.def loopCt2	= r21		; adc loop count
.def iLoopRl	= r24		; inner loop register low
.def iLoopRh	= r25		; inner loop register high

.dseg
.org	SRAM_START
sample:
.byte	2

.cseg
.org	0x0000		; reset vector
	rjmp setup
.org	WDTaddr		; Watchdog time-out vector
	rjmp WDT_isr

.org 0x000f

; morsetable:
; .db 	0b10100000,0b10110000		; 0-1
; .db 	0b10111000,0b10111100		; 2-3
; .db 	0b10111110,0b10111111		; 4-5
; .db 	0b10101111,0b10100111		; 6-7
; .db 	0b10100011,0b10100001		; 8-9
; .db 	0b01010000,0b10001110		; A-B
; .db 	0b10001010,0b01101100		; C-D
; .db 	0b00110000,0b10011010		; E-F

setup:
	ldi	tmp, low(RAMEND)	; init stack pointer
	out	SPL, tmp
	ldi	tmp, high(RAMEND)
	out	SPH, tmp
	sbi	DDRB, IR_LED		; SENSOR IR LED
	sbi	DDRB, SENS_PWR		; SENSOR power output
	cbi	DDRB, SENS_IN		; SENSOR read pin
	sbi	DDRB, BUZZER		; BUZZER output

stop:
	;ldi tmp3, 0x12
	;rcall checksensor
	;brcc _stop
	;lds tmp3, (sample)
	;rcall morse_play
	; lds tmp3, (sample)+1
	; rcall morse_play
	;lds tmp3, (sample)+1
	;rcall morse_play
; _stop:
; 	rcall sleep_2s
; 	rjmp stop
	rcall	WDT_off
	clr	tmp
	out	PORTB, tmp
	out	ADCSRA, tmp
	ldi	tmp, 0x0f
	out	PRR, tmp
	;rcall	alarm_sound

main:
	rcall	checksensor
	brbs	SREG_C, first_detected_wait

not_detected:
	rcall	sleep_8s
	rjmp	main

first_detected_wait:
	rcall	sleep_2s

detect_again:
	rcall	checksensor
	brbs	SREG_C, alarm_sound
	rjmp	not_detected

alarm_sound:
	ldi	iloopRl, LOW(15)
	ldi	iloopRh, HIGH(15)
	rcall	buzzz
 	nop
	rcall	sleep_2s
	rjmp	detect_again

; subroutines
; -----------------------------------------------------------------------------

sleep_2s:
	rcall	WDT_off
	; setup watchdog for interrupt and not reset, and 8s timeout
	ldi	tmp, (1<<WDIE)|(1<<WDE)
	out	WDTCR, tmp
	ldi	tmp, (0<<WDP3)|(1<<WDIE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0)
	out	WDTCR, tmp
	ldi	tmp, 0x0f
	out	PRR, tmp
	ldi	tmp, (1<<SE)|(1<<SM1)|(0<<SM1)
	out	MCUCR, tmp
	sleep
	clr	tmp
	out	PRR, tmp
	ret

sleep_8s:
	rcall	WDT_off
	; setup watchdog for interrupt and not reset, and 8s timeout
	ldi	tmp, (1<<WDIE)|(1<<WDE)
	out	WDTCR, tmp
	ldi	tmp, (1<<WDP3)|(1<<WDIE)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)
	out	WDTCR, tmp
	ldi	tmp, 0x0f
	out	PRR, tmp
	ldi	tmp, (1<<SE)|(1<<SM1)|(0<<SM1)
	out	MCUCR, tmp
	sleep
	clr	tmp
	out	PRR, tmp
	ret

buzzz:
	sbi	PORTB, BUZZER
	ldi	loopCt, 100
	rcall   delay
	cbi	PORTB, BUZZER
	ldi	loopCt, 100
	rcall   delay
 	sbiw	iloopRl,1
	brcc	buzzz
	ret

;	function: checksensor
;	- sets carry bit if light reflected
checksensor:
	sbi	PORTB, SENS_PWR
	sbi	PORTB, IR_LED
	ldi	loopCt, 250
	rcall	delay
	clc				; clear carry
	sbis	PINB, SENS_IN		; sensor high?, start ADC
	rjmp	checksensor_return_false
	;rjmp	checksensor_return_true

	sbi	DIDR0, ADC3D
	ldi	tmp, (1<<ADLAR)|(0<<REFS1)|(0<<REFS0)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(1<<MUX0)
	out	ADMUX, tmp
	ldi	tmp, (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADPS2)|(1<<ADPS1)|(0<<ADPS0)
	out	ADCSRA, tmp
	ldi	loopCt2, 10		; test ambient blockout n times
checksensor_adc:
	sbi	PORTB, IR_LED		; sample with IR LED first
	ldi	loopCt, 150
	rcall	delay
	sbi	ADCSRA, ADSC		; start ADC conversion
checksensor_wait1:
	sbic	ADCSRA, ADSC
	rjmp	checksensor_wait1
	in	tmp, ADCL
	in	tmp, ADCH
	sts	(sample), tmp

	cbi	PORTB, IR_LED		; sample without IR LED HIGH
	ldi	loopCt, 150
	rcall	delay
	sbi	ADCSRA, ADSC		; start ADC conversion
	sbi	PORTB, PB2
checksensor_wait2:
	sbic	ADCSRA, ADSC
	rjmp	checksensor_wait2
	cbi	PORTB, PB2
	in	tmp2, ADCL
	in	tmp2, ADCH
	sts	(sample)+1, tmp2

	ldi	tmp3, 60

	sub	tmp, tmp3
	cp	tmp2, tmp
	brsh	checksensor_return_false
	;rjmp	checksensor_return_false
	dec	loopCt2
	brne	checksensor_adc

checksensor_return_true:
	sbi PORTB,PB2
	clr	tmp
	out	ADCSRA, tmp
	cbi	DIDR0, ADC3D
	cbi	PORTB, SENS_PWR
	cbi	PORTB, IR_LED
	sec
	ret

checksensor_return_false:
	cbi PORTB,PB2
	clr	tmp
	out	ADCSRA, tmp
	cbi	DIDR0, ADC3D
	cbi	PORTB, SENS_PWR
	cbi	PORTB, IR_LED
	clc
	ret

delay:
	dec	loopCt
	brne	delay
	ret

delay_ms:
.equ delayconst = 160
.equ delayconst2 = 2
	push 	tmp
	push 	tmp2
	ldi	tmp, delayconst
	ldi	tmp2, delayconst2
_delay_ms:
	dec	tmp
	brne	_delay_ms
	ldi	tmp, delayconst
	dec	tmp2
	brne	_delay_ms
	ldi	tmp2, delayconst2

 	sbiw	iloopRl,1
	brne	_delay_ms
	pop	tmp2
	pop	tmp
	ret


; morse_play:
; .equ morsespeed_unit = 2	; morse code speed in ms
; 	push	tmp
; 	push	tmp2
; 	push	tmp3

; 	mov	tmp, tmp3	; copy input
; 	andi	tmp, 0xf0	; mask msb hex digit
; 	swap	tmp		; swap nibbles
; 	rcall	morse_play_hex_digit
; 	ldi	iloopRl, LOW(morsespeed_unit*3)	; pause letter
; 	ldi	iloopRh, HIGH(morsespeed_unit*3)
; 	rcall	delay_ms

; 	pop	tmp3		; bring back value in tmp3
; 	push	tmp3
; 	mov	tmp, tmp3	; copy input
; 	andi	tmp, 0x0f	; mask lsb second hex digit
; 	rcall	morse_play_hex_digit
; 	ldi	iloopRl, LOW(morsespeed_unit*6)	; the remaining word pause
; 	ldi	iloopRh, HIGH(morsespeed_unit*6)
; 	rcall	delay_ms
; 	pop	tmp3
; 	pop	tmp2
; 	pop	tmp
; 	ret

; morse_play_hex_digit:
; 	ldi	ZL, LOW(morsetable*2)
; 	ldi	ZH, HIGH(morsetable*2)
; 	clr	tmp2
; 	clc
; 	add	ZL, tmp		; table offset
; 	adc	ZH, tmp2
; 	lpm	tmp3, Z		; lookup morsecode

; 	mov	tmp2, tmp3	; get loop counter
; 	lsr	tmp2
; 	lsr	tmp2
; 	lsr	tmp2
; 	lsr	tmp2
; 	lsr	tmp2
; 	lsl	tmp3
; 	lsl	tmp3
; 	lsl	tmp3
; morse_loop:
; 	mov	tmp, tmp3
; 	andi	tmp, 0x80
; 	sbrc	tmp, 7
; 	rjmp	morse_short
; morse_long:
; 	ldi	iloopRl, LOW(morsespeed_unit*3)
; 	ldi	iloopRh, HIGH(morsespeed_unit*3)
; 	rjmp	morse_buzz
; morse_short:
; 	ldi	iloopRl, LOW(morsespeed_unit*1)
; 	ldi	iloopRh, HIGH(morsespeed_unit*1)
; morse_buzz:
; 	sbi	PORTB, PB2
; 	rcall	delay_ms
; 	cbi	PORTB, PB2
; 	ldi	iloopRl, LOW(morsespeed_unit*1)
; 	ldi	iloopRh, HIGH(morsespeed_unit*1)
; 	rcall	delay_ms
; 	lsl	tmp3
; 	dec	tmp2
; 	brne	morse_loop
; 	ret

; sample, high low high low

WDT_off:
	cli
	wdr
	; Clear WDRF in MCUSR
	ldi	tmp, (0<<WDRF)
	out	MCUSR, tmp
	; Write logical one to WDCE and WDE
	; Keep old prescaler setting to prevent unintentional Watchdog Reset
	in	tmp, WDTCR
	ori	tmp, (1<<WDCE)|(1<<WDE)
	out	WDTCR, tmp
	; Turn off WDT
	ldi	tmp, (0<<WDE)
	out	WDTCR, tmp
	sei
	ret

; Interrupt service routines
; -----------------------------------------------------------------------------

; watchdog
WDT_isr:
	rcall	WDT_off
	ldi	tmpisr, 0
	out	MCUSR, tmpisr
	reti
