; Fridge alarm, attiny85

.nolist
.include "tn85def.inc"
.list

.equ SENS_PWR	= PB0
.equ SENS_IN	= PB3
.equ BUZZER	= PB4
.equ LED_PIN	= PB2

.def tmp	= r16
.def tmpisr     = r17
.def loopCt	= r18		; delay loop count
.def iLoopRl	= r24		; inner loop register low
.def iLoopRh	= r25		; inner loop register high

.org	0x0000		; reset vector
	rjmp setup
.org	WDTaddr		; Watchdog time-out vector
	rjmp WDT_isr
;.cseg

.org 0x000f
setup:
	ldi	tmp, low(RAMEND)	; init stack pointer
	out	SPL, tmp
	ldi	tmp, high(RAMEND)
	out	SPH, tmp
	sbi	DDRB, SENS_PWR		; SENSOR power output
	sbi	DDRB, BUZZER		; BUZZER output
	sbi	DDRB, LED_PIN
	clr	tmp
	out	PORTB, tmp
	out	ADCSRA, tmp
	ldi	tmp, 0x0f
	out	PRR, tmp
	rcall	alarm_sound

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
	ldi	loopCt, 15
alarm_oloop:
	sbi	PORTB, BUZZER
	;nop
	cbi	PORTB, BUZZER
 	dec	loopCt
 	brne	alarm_oloop
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
	ldi	tmp, (0<<WDP3)|(1<<WDIE)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0)
	out	WDTCR, tmp
	ldi	tmp, 0x0f
	out	PRR, tmp
	ldi	tmp, (1<<SE)|(1<<SM1)|(0<<SM1)
	out	MCUCR, tmp
	sleep
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
	ret

checksensor:
;	sets carry bit if light detected
	sbi	PORTB,SENS_PWR
	ldi	loopCt, 6
checksensor_loop:
	dec	loopCt
	brne	checksensor_loop
	clc
	sbic	PINB,SENS_IN
	sec
	cbi	PORTB,SENS_PWR
	ret

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
