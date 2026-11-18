.device ATmega328P
.include "m328pdef.inc"

; Smart Traffic Junction Controller
; 16 MHz crystal. Pedestrian request: INT0/PD2 (active-low).
; Traffic LEDs: PD3-7 and pedestrian display: PB0-PB5.
; Ambient-light input: PC0/ADC0.

.equ NIGHT_ENTER_THRESHOLD, 100
.equ NIGHT_RECOVER_THRESHOLD, 350
.equ NIGHT_RECOVER_SECONDS, 5
.equ PEDESTRIAN_SECONDS, 5

.equ NS_GREEN, 0
.equ NS_YELLOW, 1
.equ ALL_RED_NS_TO_EW, 2
.equ EW_GREEN, 3
.equ EW_YELLOW, 4
.equ ALL_RED_EW_TO_NS, 5
.equ EW_GREEN_EXTRA, 6
.equ NS_GREEN_EXTRA, 7
.equ PEDESTRIAN_MODE, 8
.equ NIGHT_MODE, 9

.dseg
.org 0x100
traffic_state:      .byte 1
traffic_tick:       .byte 1
ped_request:        .byte 1
ped_countdown:      .byte 1
night_active:       .byte 1
recovery_count:     .byte 1
flash_phase:        .byte 1
adc_value_lo:       .byte 1
adc_value_hi:       .byte 1

.cseg
.org 0x0000
    rjmp reset
.org INT0addr
    rjmp int0_handler
.org TIM1_COMPAaddr
    rjmp timer1_handler

reset:
    ldi r16, low(RAMEND)
    out SPL, r16
    ldi r16, high(RAMEND)
    out SPH, r16

    ; Traffic and countdown are outputs.
    ldi r16, 0xFF
    out DDRD, r16
    ldi r16, 0x3F
    out DDRB, r16

    ldi r16, 0x00
    out PORTD, r16
    out PORTB, r16

    ; INT0 falling edge, pull-up enabled on PD2.
    ldi r16, (1<<ISC01)
    sts EICRA, r16
    ldi r16, (1<<INT0)
    out EIMSK, r16
    ldi r16, (1<<PD2)
    out PORTD, r16

    ldi r16, NS_GREEN
    sts traffic_state, r16
    sts traffic_tick, r16
    sts ped_request, r16
    sts ped_countdown, r16
    sts night_active, r16
    sts recovery_count, r16
    sts flash_phase, r16

    ; Timer1 CTC: 1 second at 16 MHz using /1024 prescaler.
    ldi r16, high(15624)
    sts OCR1AH, r16
    ldi r16, low(15624)
    sts OCR1AL, r16
    ldi r16, (1<<WGM12)
    sts TCCR1B, r16
    ldi r16, (1<<OCIE1A)
    sts TIMSK1, r16
    ldi r16, (1<<CS12)|(1<<CS10)
    ori r16, (1<<WGM12)
    sts TCCR1B, r16

    ; ADC0, AVCC reference, /128 prescaler.
    ldi r16, (1<<REFS0)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADMUX, r16
    ldi r16, (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADCSRA, r16

    sei

main:
    rcall set_traffic_outputs
    rcall sample_light
    rcall service_pedestrian
    rjmp main

int0_handler:
    push r16
    in r16, SREG
    push r16
    ldi r16, 1
    sts ped_request, r16
    pop r16
    out SREG, r16
    pop r16
    reti

timer1_handler:
    push r16
    push r17
    in r16, SREG
    push r16
    rcall update_night_mode
    lds r16, night_active
    tst r16
    breq timer_normal
    lds r16, flash_phase
    inc r16
    andi r16, 1
    sts flash_phase, r16
    rcall set_night_outputs
    pop r16
    out SREG, r16
    pop r17
    pop r16
    reti

timer_normal:
    lds r16, ped_countdown
    tst r16
    breq traffic_tick_handler
    dec r16
    sts ped_countdown, r16
    rcall display_countdown
    cpi r16, 0
    brne timer_normal_done
    ldi r16, NS_GREEN
    sts traffic_state, r16
    sts traffic_tick, r16

timer_normal_done:
    pop r16
    out SREG, r16
    pop r17
    pop r16
    reti

traffic_tick_handler:
    lds r16, traffic_tick
    inc r16
    cpi r16, 2
    brsh next_traffic_state
    sts traffic_tick, r16
    pop r16
    out SREG, r16
    pop r17
    pop r16
    reti

next_traffic_state:
    ldi r16, 0
    sts traffic_tick, r16
    lds r16, traffic_state
    inc r16
    cpi r16, 8
    brlo store_state
    ldi r16, NS_GREEN
store_state:
    sts traffic_state, r16
    pop r16
    out SREG, r16
    pop r17
    pop r16
    reti

sample_light:
    lds r16, ADCSRA
    ori r16, (1<<ADSC)
    sts ADCSRA, r16
wait_adc:
    lds r16, ADCSRA
    sbrc r16, ADSC
    rjmp wait_adc
    lds r16, ADCL
    sts adc_value_lo, r16
    lds r16, ADCH
    sts adc_value_hi, r16
    ret

update_night_mode:
    lds r16, adc_value_lo
    lds r17, adc_value_hi
    tst r17
    brne adc_bright
    cpi r16, NIGHT_ENTER_THRESHOLD
    brsh adc_bright
    ldi r16, 1
    sts night_active, r16
    ldi r16, 0
    sts recovery_count, r16
    ret

adc_bright:
    lds r16, adc_value_hi
    cpi r16, 1
    brlo below_recovery_low
    breq below_recovery_high
    rjmp sustained_bright

below_recovery_low:
    lds r16, adc_value_lo
    cpi r16, NIGHT_RECOVER_THRESHOLD
    brsh sustained_bright
    ldi r16, 0
    sts recovery_count, r16
    ret

below_recovery_high:
    lds r16, adc_value_lo
    cpi r16, 94
    brsh sustained_bright
    ldi r16, 0
    sts recovery_count, r16
    ret

sustained_bright:
    lds r16, night_active
    tst r16
    breq recovery_done
    lds r16, recovery_count
    inc r16
    cpi r16, NIGHT_RECOVER_SECONDS
    brsh exit_night
    sts recovery_count, r16
    ret

exit_night:
    ldi r16, 0
    sts night_active, r16
    sts recovery_count, r16
recovery_done:
    ret

service_pedestrian:
    lds r16, night_active
    tst r16
    brne pedestrian_done
    lds r16, ped_request
    tst r16
    breq pedestrian_done
    lds r16, traffic_state
    cpi r16, ALL_RED_NS_TO_EW
    breq start_pedestrian
    cpi r16, ALL_RED_EW_TO_NS
    breq start_pedestrian
    rjmp pedestrian_done

start_pedestrian:
    ldi r16, PEDESTRIAN_MODE
    sts traffic_state, r16
    ldi r16, PEDESTRIAN_SECONDS
    sts ped_countdown, r16
    ldi r16, 0
    sts ped_request, r16
    rcall all_red
    rcall display_countdown
pedestrian_done:
    ret

set_traffic_outputs:
    lds r16, night_active
    tst r16
    brne set_night_outputs
    lds r16, traffic_state
    cpi r16, NS_GREEN
    breq ns_green
    cpi r16, NS_YELLOW
    breq ns_yellow
    cpi r16, EW_GREEN
    breq ew_green
    cpi r16, EW_GREEN_EXTRA
    breq ew_green
    cpi r16, EW_YELLOW
    breq ew_yellow
    cpi r16, NS_GREEN_EXTRA
    breq ns_green
    cpi r16, PEDESTRIAN_MODE
    breq all_red
    rjmp all_red

ns_green:
    ldi r16, (1<<PD4)|(1<<PD5)
    out PORTD, r16
    ret
ns_yellow:
    ldi r16, (1<<PD3)|(1<<PD5)
    out PORTD, r16
    ret
ew_green:
    ldi r16, (1<<PD2)|(1<<PD7)
    out PORTD, r16
    ret
ew_yellow:
    ldi r16, (1<<PD2)|(1<<PD6)
    out PORTD, r16
    ret
all_red:
    ldi r16, (1<<PD2)|(1<<PD5)
    out PORTD, r16
    ret

set_night_outputs:
    lds r16, flash_phase
    tst r16
    breq night_off
    ldi r16, (1<<PD3)|(1<<PD6)
    out PORTD, r16
    ret
night_off:
    ldi r16, 0
    out PORTD, r16
    ret

display_countdown:
    ldi r16, 0
    out PORTB, r16
    lds r17, ped_countdown
    cpi r17, 5
    breq seg5
    cpi r17, 4
    breq seg4
    cpi r17, 3
    breq seg3
    cpi r17, 2
    breq seg2
    cpi r17, 1
    breq seg1
    rjmp seg0

seg0:
    ldi r16, 0x3F
    rjmp write_segments
seg1:
    ldi r16, 0x06
    rjmp write_segments
seg2:
    ldi r16, 0x5B
    rjmp write_segments
seg3:
    ldi r16, 0x4F
    rjmp write_segments
seg4:
    ldi r16, 0x66
    rjmp write_segments
seg5:
    ldi r16, 0x6D
write_segments:
    out PORTB, r16
    ret
