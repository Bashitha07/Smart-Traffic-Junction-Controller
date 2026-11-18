;==============================================================================
; IE3064 Embedded Systems Engineering — Smart Traffic Junction Controller
; Target        : ATmega328P (Arduino Uno / Standalone), 16 MHz Crystal
; Language      : Pure AVR Assembly (avrasm2)
; Toolchain     : Microchip Studio 7 / Atmel Studio
;
; Compliance summary:
;   - Two-phase (N-S / E-W) intersection.
;   - Every direction changes  GREEN -> YELLOW -> RED  when stopping and
;     RED -> YELLOW -> GREEN  when starting (pre-green yellow). Never
;     red -> green directly.
;   - Sequence: NS_GREEN 3s, NS_YELLOW 2s, ALL_RED 1s, EW_PREP(yellow) 1s,
;               EW_GREEN 3s, EW_YELLOW 2s, ALL_RED 1s, NS_PREP(yellow) 1s
;     Total normal cycle = 3+2+1+1+3+2+1+1 = 14s (< 15s).
;   - Pedestrian crossing: deferred request via INT0/PD2 hardware interrupt,
;     started only after the vehicle phase and its All-Red clearance have
;     completed. ALL FOUR directions stay red for the 5s countdown, then
;     the next direction shows yellow before green (red->yellow->green).
;   - 7-seg: COMMON_CATHODE switch (5161AS = cathode: COM->GND).
;   - Night mode: LDR on ADC0/PC0, flashes both Yellow signals at 1 Hz.
;   - Hysteresis: requires 5s (10 ticks) of bright light to exit Night mode.
;==============================================================================

.NOLIST
.INCLUDE "m328Pdef.inc"        ; Include ATmega328P register definition file (maps names like PORTB, SPL to physical addresses)
.LIST

;==============================================================================
; SECTION 1: CONSTANTS & PIN DEFINITIONS
;==============================================================================

; ---- FSM State Numbers (Encoded States for the Traffic Controller) ---------
.EQU STATE_NS_GREEN     = 0     ; North-South Green active phase
.EQU STATE_NS_YELLOW    = 1     ; North-South yellow transition (EW red)
.EQU STATE_ALL_RED_1    = 2     ; All-Red safety clearance after N-S phase
.EQU STATE_EW_PREP      = 3     ; East-West pre-green yellow warning
.EQU STATE_EW_GREEN     = 4     ; East-West Green active phase
.EQU STATE_EW_YELLOW    = 5     ; East-West yellow transition (NS red)
.EQU STATE_ALL_RED_2    = 6     ; All-Red safety clearance after E-W phase
.EQU STATE_NS_PREP      = 7     ; North-South pre-green yellow warning
.EQU STATE_PEDESTRIAN   = 8     ; Pedestrian crossing active state (All Red)
.EQU STATE_NIGHT        = 9     ; Night mode active (Flashing Yellow)

; ---- Timing Constants (Configured in seconds, Total Normal Cycle = 14s) ----
.EQU TIME_NS_GREEN   = 3     ; North-South Green duration (3s)
.EQU TIME_NS_YELLOW  = 2     ; North-South Yellow duration (2s)
.EQU TIME_ALL_RED    = 1     ; Intersection All-Red Clearance (1s)
.EQU TIME_PRE_GREEN  = 1     ; Red->Yellow pre-green safety warning (1s)
.EQU TIME_EW_GREEN   = 3     ; East-West Green duration (3s)
.EQU TIME_EW_YELLOW  = 2     ; East-West Yellow duration (2s)

; ---- Pedestrian & Night Mode Configuration Parameters ---------------------
.EQU PED_COUNT_START    = 5      ; Pedestrian crossing countdown window (5 seconds)
.EQU HYSTERESIS_TICKS   = 10     ; 10 ticks * 500ms = 5000ms (5 consecutive seconds of light required to exit Night mode)
.EQU LIGHT_THRESHOLD    = 100    ; ADCH reading below this threshold = Dark (triggers Night Mode)

; ---- PORTB Bit Mapping (Traffic Light LED Outputs) ------------------------
.EQU NS_RED   = 0    ; PB0: North-South Red LED
.EQU NS_YEL   = 1    ; PB1: North-South Yellow LED
.EQU NS_GRN   = 2    ; PB2: North-South Green LED
.EQU EW_RED   = 3    ; PB3: East-West Red LED
.EQU EW_YEL   = 4    ; PB4: East-West Yellow LED
.EQU EW_GRN   = 5    ; PB5: East-West Green LED

; ---- 7-Segment Display Configuration ----------------------------------------
; 1 = COMMON CATHODE (e.g., 5161AS): Common pin connected to GND, segments active HIGH
.EQU COMMON_CATHODE  = 1

; ---- PORTD Bit Mapping ----------------------------------------------------
.EQU PED_BTN  = 2    ; PD2: INT0 Hardware Interrupt Pedestrian Push-Button Input

;==============================================================================
; SECTION 2: SRAM DATA MEMORY (State Variables & Flags)
;==============================================================================
.DSEG
.ORG SRAM_START

current_state:        .BYTE 1     ; Stores active FSM state index (1 byte in SRAM)
seconds_in_state:     .BYTE 1     ; Elapsed full seconds within the current state (1 byte)
resume_state:         .BYTE 1     ; Stores vehicle state to resume after pedestrian crossing (1 byte)
pedestrian_flag:      .BYTE 1     ; Flag: 1 = Pedestrian crossing requested via button (1 byte)
pedestrian_countdown: .BYTE 1     ; Active countdown value (5 down to 0) (1 byte)
hysteresis_counter:   .BYTE 1     ; Tracks consecutive 500ms ticks of bright light (1 byte)
adc_result:           .BYTE 1     ; Stores the latest 8-bit ADCH reading from LDR (1 byte)
tick_flag:            .BYTE 1     ; Set by Timer1 ISR every 500ms to drive main loop (1 byte)
half_tick_counter:    .BYTE 1     ; Toggles every 500ms (2 ticks = exactly 1 second) (1 byte)
flash_state:          .BYTE 1     ; Toggle variable (0/1) for 1 Hz night mode flashing (1 byte)

;==============================================================================
; SECTION 3: INTERRUPT VECTOR TABLE
;==============================================================================
.CSEG
.ORG 0x0000
    jmp RESET              ; 0x0000: System Reset Vector (Jump to boot initialization)
    jmp ISR_INT0           ; 0x0002: External Interrupt 0 (Pedestrian Button on PD2)
    jmp DEFAULT_ISR        ; 0x0004: INT1 (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0006: PCINT0 (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0008: PCINT1 (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x000A: PCINT2 (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x000C: WDT (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x000E: TIMER2 COMPA (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0010: TIMER2 COMPB (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0012: TIMER2 OVF (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0014: TIMER1 CAPT (Unused, catch-all)
    jmp ISR_TIMER1_COMPA   ; 0x0016: Timer1 Compare Match A (500ms system tick timer interrupt)
    jmp DEFAULT_ISR        ; 0x0018: TIMER1 COMPB (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x001A: TIMER1 OVF (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x001C: TIMER0 COMPA (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x001E: TIMER0 COMPB (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0020: TIMER0 OVF (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0022: SPI STC (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0024: USART RX (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0026: USART UDRE (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0028: USART TX (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x002A: ADC Conversion Complete (Unused, polled mode used instead)
    jmp DEFAULT_ISR        ; 0x002C: EEPROM Ready (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x002E: Analog Comparator (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0030: TWI / I2C (Unused, catch-all)
    jmp DEFAULT_ISR        ; 0x0032: Store Program Memory Ready (Unused, catch-all)

DEFAULT_ISR:
    reti                   ; Catch-all dummy ISR: return from interrupt immediately for unhandled vectors

;==============================================================================
; SECTION 4: INITIALISATION & MAIN SUPERVISOR LOOP
;==============================================================================

RESET:
    ; Initialize Hardware Stack Pointer to top of RAM
    ldi   r16, low(RAMEND)        ; Load low byte of RAM top address into r16 (e.g., 0xFF for ATmega328P)
    out   SPL, r16                ; Write low byte into Stack Pointer Low register (SPL)
    ldi   r16, high(RAMEND)       ; Load high byte of RAM top address into r16 (e.g., 0x08 for ATmega328P)
    out   SPH, r16                ; Write high byte into Stack Pointer High register (SPH)

    ; Initialize Microcontroller Peripherals
    rcall init_ports              ; Call subroutine to set up I/O pin directions and pull-ups
    rcall init_timer1             ; Call subroutine to configure Timer1 for 500ms intervals
    rcall init_adc                ; Call subroutine to configure the LDR analog-to-digital converter
    rcall init_int0               ; Call subroutine to configure the pedestrian button external interrupt

    ; Clear and Initialize SRAM Variables
    clr   r16                     ; Clear register r16 (sets r16 = 0x00)
    sts   seconds_in_state, r16   ; Clear state timer counter in SRAM
    sts   pedestrian_flag, r16    ; Clear pedestrian call flag in SRAM (0 = no call pending)
    sts   pedestrian_countdown, r16; Clear pedestrian crossing countdown variable in SRAM
    sts   hysteresis_counter, r16 ; Clear LDR stability counter in SRAM
    sts   tick_flag, r16          ; Clear 500ms timer tick flag in SRAM
    sts   half_tick_counter, r16  ; Clear 500ms half-tick auxiliary counter in SRAM
    sts   flash_state, r16        ; Clear night mode flash toggle state in SRAM
    
    ; Set starting traffic state to North-South Green
    ldi   r16, STATE_NS_GREEN     ; Load constant value representing NS Green state (0) into r16
    sts   current_state, r16      ; Save starting state ID into SRAM variable 'current_state'

    ; Apply Initial Hardware Outputs
    rcall update_leds_for_state   ; Drive physical traffic light pins to match NS Green state
    ldi   r19, 10                 ; Load index value 10 into r19 (Index 10 makes 7-segment display blank)
    rcall update_countdown_display ; Output blank character to 7-segment display at boot

    ; Configure Sleep Mode (Idle Mode) to optimize simulator CPU efficiency
    ldi   r16, (1<<SE)            ; Set Sleep Enable (SE) bit in r16
    out   SMCR, r16               ; Write config byte to Sleep Mode Control Register (SMCR)

    sei                           ; Enable Global Interrupts (Sets I-bit in SREG)

main_loop:
    lds   r16, tick_flag          ; Load the 500ms tick flag from SRAM into r16
    tst   r16                     ; Test r16 contents (sets Zero flag if r16 == 0x00)
    brne  do_tick                 ; If tick_flag is non-zero (timer expired), jump to do_tick processing
    sleep                         ; Otherwise, put CPU in low-power idle sleep until next interrupt wakes it
    rjmp  main_loop               ; Unconditional jump back to re-check tick flag after waking up

do_tick:
    clr   r16                     ; Clear register r16 (r16 = 0x00)
    sts   tick_flag, r16          ; Reset tick flag in SRAM so it handles only once per tick interval
    rcall handle_tick             ; Execute core FSM state transitions, time updates, and LDR sensor checking
    rjmp  main_loop               ; Unconditional jump back to main loop

;==============================================================================
; SECTION 5: PERIPHERAL INITIALISATION SUBROUTINES
;==============================================================================

init_ports:
    ; PORTB Setup: PB0..PB5 configured as outputs for Traffic Light LEDs
    ldi   r16, 0b00111111         ; Set bits 0 to 5 as output drivers
    out   DDRB, r16               ; Write configuration to Data Direction Register B
    clr   r16                     ; Clear r16 (0x00)
    out   PORTB, r16              ; Ensure all PORTB output pins start LOW

    ; PORTD Setup: PD2 configured as input (Pedestrian Button), PD3..PD7 as outputs (7-seg segments A..E)
    ldi   r16, 0b11111000         ; Set bits 3-7 as outputs, bits 0-2 as inputs
    out   DDRD, r16               ; Write configuration to Data Direction Register D
    ldi   r16, (1<<PED_BTN)        ; Create bitmask for PD2 pull-up resistor activation
    out   PORTD, r16              ; Activate internal pull-up resistor on pedestrian push-button input pin (PD2)

    ; PORTC Setup: PC0 configured as input (LDR ADC), PC1/PC2 configured as outputs (7-seg segments F,G)
    ldi   r16, 0b00000110         ; Set PC1 and PC2 as outputs, PC0 (ADC0) as analog input
    out   DDRC, r16               ; Write configuration to Data Direction Register C
    clr   r16                     ; Clear r16
    out   PORTC, r16              ; Set PORTC outputs LOW
    ret                           ; Return from subroutine

init_timer1:
    ; Setup Timer1 in CTC Mode (Clear Timer on Compare Match, TOP = OCR1A)
    ; Prescaler = 256. For 16 MHz crystal: 16,000,000 / 256 = 62,500 Hz clock.
    ; Target tick interval = 500ms (0.5s). Counts = 62,500 * 0.5 = 31,250.
    ; OCR1A = 31250 - 1 = 31249 (0x7A11)
    clr   r16                     ; Clear r16
    sts   TCCR1A, r16             ; Clear Timer/Counter 1 Control Register A (normal port operation)
    ldi   r16, (1<<WGM12) | (1<<CS12) ; Enable CTC mode (WGM12=1) and set Prescaler to 256 (CS12=1)
    sts   TCCR1B, r16             ; Write configuration to TCCR1B
    ldi   r16, high(31249)        ; Load high byte of compare match target count (0x7A) into r16
    sts   OCR1AH, r16             ; Store high byte into Output Compare Register 1 A High
    ldi   r16, low(31249)         ; Load low byte of compare match target count (0x11) into r16
    sts   OCR1AL, r16             ; Store low byte into Output Compare Register 1 A Low
    ldi   r16, (1<<OCIE1A)        ; Create bitmask to enable Timer1 Compare Match A Interrupt
    sts   TIMSK1, r16             ; Write mask to Timer Interrupt Mask Register 1
    ret                           ; Return from subroutine

init_adc:
    ; Configure ADC: AVcc reference voltage, Left-adjusted result (8-bit resolution in ADCH), Channel ADC0 (PC0)
    ldi   r16, (1<<REFS0) | (1<<ADLAR) ; REFS0 = AVcc reference with external capacitor on AREF, ADLAR = Left adjust result for 8-bit read
    sts   ADMUX, r16              ; Write configuration to ADC Multiplexer Selection Register
    ; Enable ADC peripheral, set Prescaler to 128 (16 MHz / 128 = 125 kHz safe ADC operating clock)
    ldi   r16, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) ; ADEN = Enable ADC, ADPS2..0 = Prescaler division factor 128
    sts   ADCSRA, r16             ; Write configuration to ADC Control and Status Register A
    ret                           ; Return from subroutine

init_int0:
    ; Configure External Interrupt INT0 (PD2) to trigger on falling edge (button press to ground)
    ldi   r16, (1<<ISC01)         ; ISC01 = 1, ISC00 = 0 sets falling edge interrupt trigger mode for INT0
    sts   EICRA, r16              ; Write configuration to External Interrupt Control Register A
    ldi   r16, (1<<INT0)          ; Create bitmask to enable external interrupt request 0
    out   EIMSK, r16              ; Write mask to External Interrupt Mask Register
    ret                           ; Return from subroutine

;==============================================================================
; SECTION 6: INTERRUPT SERVICE ROUTINES (ISRs)
;==============================================================================

ISR_INT0:
    push  r16                     ; Save register r16 onto the hardware stack
    in    r16, SREG               ; Read Status Register (SREG) into r16 to save CPU flags
    push  r16                     ; Save SREG value onto the stack

    ; Safety Check: Ignore pedestrian button presses if currently in Night Mode
    lds   r16, current_state      ; Load current state index from SRAM into r16
    cpi   r16, STATE_NIGHT        ; Compare current state with Night Mode state ID (9)
    breq  int0_exit               ; If in night mode, skip setting pedestrian flag and exit

    ; Set deferred pedestrian crossing request flag
    ldi   r16, 1                  ; Load value 1 into r16 indicating a pending pedestrian request
    sts   pedestrian_flag, r16    ; Store flag in SRAM variable 'pedestrian_flag'

int0_exit:
    pop   r16                     ; Restore saved SREG value from stack into r16
    out   SREG, r16               ; Restore CPU status flags
    pop   r16                     ; Restore original r16 register contents from stack
    reti                          ; Return from interrupt (automatically re-enables global interrupts)

ISR_TIMER1_COMPA:
    push  r16                     ; Save r16 register onto stack
    in    r16, SREG               ; Read Status Register into r16
    push  r16                     ; Save SREG onto stack

    ldi   r16, 1                  ; Load value 1 into r16
    sts   tick_flag, r16          ; Set tick_flag in SRAM to signal main loop that 500ms has elapsed

    pop   r16                     ; Restore SREG from stack
    out   SREG, r16               ; Restore status flags
    pop   r16                     ; Restore r16 from stack
    reti                          ; Return from interrupt

;==============================================================================
; SECTION 7: CORE FSM LOGIC & TIMING CONTROLLER
;==============================================================================

handle_tick:
    rcall read_adc                 ; Call helper function to sample LDR light sensor value into adc_result variable

    lds   r16, current_state      ; Load current FSM state from SRAM into r16
    cpi   r16, STATE_NIGHT        ; Compare state with Night Mode ID
    brne  check_darkness          ; If not in night mode, proceed to check ambient light levels
    rjmp  process_night_mode       ; If already in night mode, jump directly to night mode management routine

check_darkness:
    ; --- NORMAL / PEDESTRIAN ENVIRONMENT MONITORING ---
    lds   r17, adc_result         ; Load latest 8-bit LDR ADC reading from SRAM into r17
    cpi   r17, LIGHT_THRESHOLD    ; Compare ADC reading against light threshold constant (100)
    brsh  tick_timer              ; If ADC value >= 100 (bright ambient light), continue normal timing cycle
    rjmp  enter_night_mode         ; If ADC value < 100 (dark ambient conditions), switch immediately to Night Mode

tick_timer:
    ; Convert 500ms ticks into 1-second increments (2 ticks = 1 second)
    lds   r16, half_tick_counter  ; Load current half-tick flip-flop counter state into r16
    ldi   r17, 1                  ; Load bitmask value 1 into r17
    eor   r16, r17                ; Exclusive-OR half_tick_counter with 1 (toggles 0 to 1, or 1 to 0)
    sts   half_tick_counter, r16  ; Save toggled state back to SRAM variable 'half_tick_counter'
    tst   r16                     ; Test r16 (sets Zero flag if toggle resulted in 0x00)
    breq  second_elapsed          ; If Zero flag is set (every 2nd tick = 1 full second), proceed to update state timers
    rjmp  exit_tick               ; Otherwise (first 500ms half-tick), exit tick handler and wait for next tick

second_elapsed:
    ; --- EXACTLY 1 SECOND HAS PASSED: UPDATE FSM ---
    lds   r16, current_state      ; Load current state index into r16
    cpi   r16, STATE_PEDESTRIAN   ; Check if system is currently executing a pedestrian crossing countdown
    brne  increment_timer         ; If not in pedestrian mode, proceed to increment standard vehicle state timer
    rjmp  process_pedestrian_countdown ; If in pedestrian mode, jump to handle pedestrian countdown decrementing

increment_timer:
    lds   r17, seconds_in_state   ; Load elapsed seconds count for the current state from SRAM into r17
    inc   r17                     ; Increment elapsed second counter by 1
    sts   seconds_in_state, r17   ; Store updated second count back into SRAM variable 'seconds_in_state'

    ; FSM State Dispatcher Trampoline table (Evaluates current state using sequential conditional jumps)
    cpi   r16, STATE_NS_GREEN     ; Compare current state with North-South Green ID (0)
    brne  skip_ns_green           ; If not NS Green, skip to next state check
    rjmp  check_ns_green          ; If NS Green, jump to evaluate its duration timeout

skip_ns_green:
    cpi   r16, STATE_NS_YELLOW    ; Compare state with North-South Yellow ID (1)
    brne  skip_ns_yellow          ; If not NS Yellow, skip
    rjmp  check_ns_yellow         ; Jump to evaluate NS Yellow duration timeout

skip_ns_yellow:
    cpi   r16, STATE_ALL_RED_1    ; Compare state with First All-Red Clearance ID (2)
    brne  skip_all_red_1          ; If not All-Red 1, skip
    rjmp  check_all_red_1         ; Jump to evaluate All-Red 1 duration timeout

skip_all_red_1:
    cpi   r16, STATE_EW_PREP      ; Compare state with East-West Pre-Green Yellow ID (3)
    brne  skip_ew_prep            ; If not EW Prep, skip
    rjmp  check_ew_prep           ; Jump to evaluate EW Prep duration timeout

skip_ew_prep:
    cpi   r16, STATE_EW_GREEN     ; Compare state with East-West Green ID (4)
    brne  skip_ew_green           ; If not EW Green, skip
    rjmp  check_ew_green          ; Jump to evaluate EW Green duration timeout

skip_ew_green:
    cpi   r16, STATE_EW_YELLOW    ; Compare state with East-West Yellow ID (5)
    brne  skip_ew_yellow          ; If not EW Yellow, skip
    rjmp  check_ew_yellow         ; Jump to evaluate EW Yellow duration timeout

skip_ew_yellow:
    cpi   r16, STATE_ALL_RED_2    ; Compare state with Second All-Red Clearance ID (6)
    brne  skip_all_red_2          ; If not All-Red 2, skip
    rjmp  check_all_red_2         ; Jump to evaluate All-Red 2 duration timeout

skip_all_red_2:
    cpi   r16, STATE_NS_PREP      ; Compare state with North-South Pre-Green Yellow ID (7)
    brne  skip_ns_prep            ; If not NS Prep, skip
    rjmp  check_ns_prep           ; Jump to evaluate NS Prep duration timeout

skip_ns_prep:
    rjmp  exit_tick               ; Fallback safety exit if state index is unhandled

; --- INDIVIDUAL FSM PHASE DURATION EVALUATION ---

check_ns_green:
    lds   r17, seconds_in_state   ; Load elapsed seconds in NS Green state
    cpi   r17, TIME_NS_GREEN       ; Compare elapsed time against duration constant (3 seconds)
    brsh  end_ns_green            ; If elapsed time >= 3s, branch to transition out of NS Green
    rjmp  exit_tick               ; Otherwise, duration not met yet; exit tick handler
end_ns_green:
    ldi   r16, STATE_NS_YELLOW    ; Load next state ID (STATE_NS_YELLOW = 1) into r16
    rjmp  change_state            ; Jump to state transition helper routine

check_ns_yellow:
    lds   r17, seconds_in_state   ; Load elapsed seconds in NS Yellow state
    cpi   r17, TIME_NS_YELLOW      ; Compare elapsed time against duration constant (2 seconds)
    brsh  end_ns_yellow           ; If elapsed time >= 2s, branch to transition out of NS Yellow
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_ns_yellow:
    ldi   r16, STATE_ALL_RED_1    ; Load next state ID (STATE_ALL_RED_1 = 2) into r16
    rjmp  change_state            ; Jump to state transition routine

check_all_red_1:
    lds   r17, seconds_in_state   ; Load elapsed seconds in All-Red 1 state
    cpi   r17, TIME_ALL_RED        ; Compare elapsed time against duration constant (1 second)
    brsh  end_all_red_1           ; If elapsed time >= 1s, branch to complete clearance
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_all_red_1:
    ; All-red clearance complete: check if pedestrian button was pressed during the cycle
    lds   r18, pedestrian_flag    ; Load pedestrian request flag from SRAM into r18
    tst   r18                     ; Test flag (sets Zero flag if pedestrian_flag == 0)
    brne  ped_from_ar1            ; If flag is non-zero (pedestrian button was pushed), branch to handle pedestrian crossing
    ldi   r16, STATE_EW_PREP      ; Normal flow: load EW pre-green yellow warning state ID (3) into r16
    rjmp  change_state            ; Jump to state transition routine
ped_from_ar1:
    ldi   r16, STATE_EW_PREP      ; Load resumption state ID (EW pre-green yellow warning = 3) into r16
    sts   resume_state, r16       ; Save this state in SRAM variable 'resume_state' to resume traffic after pedestrian phase
    ldi   r16, STATE_PEDESTRIAN   ; Load Pedestrian crossing state ID (8) into r16
    rjmp  start_pedestrian_phase  ; Jump to initialize pedestrian crossing mode

check_ew_prep:
    lds   r17, seconds_in_state   ; Load elapsed seconds in EW Prep state
    cpi   r17, TIME_PRE_GREEN      ; Compare elapsed time against duration constant (1 second)
    brsh  end_ew_prep             ; If elapsed time >= 1s, branch to transition out of EW Prep
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_ew_prep:
    ldi   r16, STATE_EW_GREEN     ; Load next state ID (STATE_EW_GREEN = 4) into r16
    rjmp  change_state            ; Jump to state transition routine

check_ew_green:
    lds   r17, seconds_in_state   ; Load elapsed seconds in EW Green state
    cpi   r17, TIME_EW_GREEN       ; Compare elapsed time against duration constant (3 seconds)
    brsh  end_ew_green            ; If elapsed time >= 3s, branch to transition out of EW Green
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_ew_green:
    ldi   r16, STATE_EW_YELLOW    ; Load next state ID (STATE_EW_YELLOW = 5) into r16
    rjmp  change_state            ; Jump to state transition routine

check_ew_yellow:
    lds   r17, seconds_in_state   ; Load elapsed seconds in EW Yellow state
    cpi   r17, TIME_EW_YELLOW      ; Compare elapsed time against duration constant (2 seconds)
    brsh  end_ew_yellow           ; If elapsed time >= 2s, branch to transition out of EW Yellow
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_ew_yellow:
    ldi   r16, STATE_ALL_RED_2    ; Load next state ID (STATE_ALL_RED_2 = 6) into r16
    rjmp  change_state            ; Jump to state transition routine

check_all_red_2:
    lds   r17, seconds_in_state   ; Load elapsed seconds in All-Red 2 state
    cpi   r17, TIME_ALL_RED        ; Compare elapsed time against duration constant (1 second)
    brsh  end_all_red_2           ; If elapsed time >= 1s, branch to complete clearance
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_all_red_2:
    ; All-red clearance complete: check if pedestrian button was pressed during the cycle
    lds   r18, pedestrian_flag    ; Load pedestrian request flag from SRAM into r18
    tst   r18                     ; Test flag
    brne  ped_from_ar2            ; If flag is non-zero, branch to handle pedestrian crossing
    ldi   r16, STATE_NS_PREP      ; Normal flow: load NS pre-green yellow warning state ID (7) into r16
    rjmp  change_state            ; Jump to state transition routine
ped_from_ar2:
    ldi   r16, STATE_NS_PREP      ; Load resumption state ID (NS pre-green yellow warning = 7) into r16
    sts   resume_state, r16       ; Save resumption state in SRAM variable 'resume_state'
    ldi   r16, STATE_PEDESTRIAN   ; Load Pedestrian crossing state ID (8) into r16
    rjmp  start_pedestrian_phase  ; Jump to initialize pedestrian crossing mode

check_ns_prep:
    lds   r17, seconds_in_state   ; Load elapsed seconds in NS Prep state
    cpi   r17, TIME_PRE_GREEN      ; Compare elapsed time against duration constant (1 second)
    brsh  end_ns_prep             ; If elapsed time >= 1s, branch to transition out of NS Prep
    rjmp  exit_tick               ; Otherwise, exit tick handler
end_ns_prep:
    ldi   r16, STATE_NS_GREEN     ; Load next state ID (STATE_NS_GREEN = 0) to loop back to start of normal cycle
    rjmp  change_state            ; Jump to state transition routine


start_pedestrian_phase:
    sts   current_state, r16      ; Store Pedestrian state ID (8) into 'current_state' in SRAM
    clr   r16                     ; Clear r16 (0x00)
    sts   seconds_in_state, r16   ; Reset elapsed seconds counter for the pedestrian state
    sts   pedestrian_flag, r16    ; Clear pedestrian request flag in SRAM (acknowledges request)
    ldi   r19, PED_COUNT_START    ; Load countdown start value constant (5 seconds) into r19
    sts   pedestrian_countdown, r19 ; Store countdown value into SRAM variable 'pedestrian_countdown'
    rcall update_leds_for_state   ; Update traffic light hardware outputs (all red during pedestrian phase)
    rcall update_countdown_display ; Output countdown number '5' to the 7-segment display
    rjmp  exit_tick               ; Exit tick handler

process_pedestrian_countdown:
    lds   r19, pedestrian_countdown ; Load remaining pedestrian countdown seconds from SRAM into r19
    tst   r19                     ; Test countdown value (sets Zero flag if r19 == 0)
    brne  countdown_not_zero      ; If countdown is not zero yet, branch to decrement it
    rjmp  end_pedestrian_phase    ; If countdown has reached zero, jump to terminate pedestrian phase
countdown_not_zero:
    dec   r19                     ; Decrement countdown value by 1 second
    sts   pedestrian_countdown, r19 ; Store updated countdown value back into SRAM
    rcall update_countdown_display ; Update 7-segment display with new countdown digit
    cpi   r19, 0                  ; Compare updated countdown with 0
    brne  countdown_still_going   ; If not zero yet, branch to exit tick
    rjmp  end_pedestrian_phase    ; If countdown just hit zero, terminate pedestrian phase
countdown_still_going:
    rjmp  exit_tick               ; Exit tick handler

end_pedestrian_phase:
    ldi   r19, 10                 ; Load index value 10 into r19 to blank the 7-segment display
    rcall update_countdown_display ; Output blank display pattern to hardware
    lds   r16, resume_state       ; Load saved vehicle resumption state from SRAM into r16
    rjmp  change_state            ; Jump to state transition routine to resume normal traffic sequence

change_state:
    sts   current_state, r16      ; Store new state ID into SRAM variable 'current_state'
    clr   r16                     ; Clear r16 (0x00)
    sts   seconds_in_state, r16   ; Reset phase elapsed seconds timer variable in SRAM to 0
    rcall update_leds_for_state   ; Update physical traffic light LED outputs for the new state
    rjmp  exit_tick               ; Exit tick handler

; --- NIGHT MODE & SENSOR HYSTERESIS LOGIC ---

enter_night_mode:
    ldi   r16, STATE_NIGHT        ; Load Night Mode state ID (9) into r16
    sts   current_state, r16      ; Store Night state ID into SRAM variable 'current_state'
    clr   r16                     ; Clear r16 (0x00)
    sts   hysteresis_counter, r16  ; Reset stabilization counter variable in SRAM to 0
    sts   flash_state, r16        ; Clear flash toggle variable in SRAM
    ldi   r19, 10                 ; Load index value 10 into r19 to blank 7-segment display in night mode
    rcall update_countdown_display ; Output blank display pattern
    rcall update_leds_for_state   ; Update hardware LED outputs for night mode
    rjmp  exit_tick               ; Exit tick handler

process_night_mode:
    ; Toggle 1 Hz flash state every 500ms tick
    lds   r16, flash_state        ; Load current flash toggle state (0 or 1) from SRAM into r16
    com   r16                     ; Perform logical one's complement bit inversion on r16 (flips 0 to 255, 1 to 254)
    andi  r16, 0x01               ; Mask out all bits except bit 0 (results in clean alternating 0 / 1 toggle)
    sts   flash_state, r16        ; Store updated flash state back into SRAM variable 'flash_state'
    rcall update_leds_for_state   ; Update yellow LED flashing output based on new flash state

    ; Hysteresis Filtering: Check if ambient light has returned
    lds   r17, adc_result         ; Load latest LDR ADC reading from SRAM into r17
    cpi   r17, LIGHT_THRESHOLD    ; Compare ADC reading against light threshold constant (100)
    brsh  bright_light_detected   ; If ADC value >= 100 (light returned), branch to increment hysteresis stability counter
    rjmp  reset_hysteresis         ; If ADC value < 100 (still dark), jump to clear/reset hysteresis counter
    
bright_light_detected:
    ; Bright light detected: increment consecutive stability counter
    lds   r18, hysteresis_counter ; Load current consecutive stability tick count from SRAM into r18
    inc   r18                     ; Increment stability tick count by 1
    sts   hysteresis_counter, r18 ; Store updated count back into SRAM variable 'hysteresis_counter'
    cpi   r18, HYSTERESIS_TICKS   ; Compare stability count against threshold constant (10 ticks = 5 seconds)
    brsh  hysteresis_complete     ; If stability count >= 10 ticks, branch to confirm return to normal mode
    rjmp  exit_tick               ; Otherwise, keep waiting in night mode; exit tick handler

hysteresis_complete:
    ; 5 consecutive seconds of bright light confirmed -> Safely exit Night Mode to NS Pre-Green
    ldi   r16, STATE_NS_PREP      ; Load North-South Pre-Green Yellow warning state ID (7) into r16
    rjmp  change_state            ; Jump to state transition routine to resume normal operation

reset_hysteresis:
    clr   r16                     ; Clear r16 (0x00)
    sts   hysteresis_counter, r16 ; Reset hysteresis stability counter in SRAM back to 0 because darkness re-occurred

exit_tick:
    ret                           ; Return from handle_tick subroutine

;==============================================================================
; SECTION 8: HARDWARE HELPER FUNCTIONS
;==============================================================================

read_adc:
    lds   r16, ADCSRA             ; Load ADC Control and Status Register A from SRAM into r16
    ori   r16, (1<<ADSC)          ; Set Start Conversion bit (ADSC) using bitwise OR (r16 |= 0x40)
    sts   ADCSRA, r16             ; Write updated control byte back to ADCSRA to trigger analog-to-digital conversion
wait_adc:
    lds   r16, ADCSRA             ; Load ADCSRA status register into r16
    sbrc  r16, ADSC               ; Skip next instruction if ADSC bit has cleared (hardware clears ADSC automatically when conversion finishes)
    rjmp  wait_adc                ; If ADSC is still set, loop back to wait for conversion completion
    lds   r16, ADCH               ; Read 8-bit left-adjusted conversion result byte from ADC Data Register High (ADCH)
    sts   adc_result, r16         ; Store 8-bit LDR sensor reading into SRAM variable 'adc_result'
    ret                           ; Return from subroutine

update_leds_for_state:
    lds   r16, current_state      ; Load current state index from SRAM into r16
    
    cpi   r16, STATE_NS_GREEN     ; Compare state with NS Green (0)
    brne  chk_ns_yel              ; If not match, branch to check next state
    rjmp  led_ns_green            ; If match, jump to NS Green LED pattern routine
chk_ns_yel:
    cpi   r16, STATE_NS_YELLOW    ; Compare state with NS Yellow (1)
    brne  chk_ar1                 ; If not match, branch
    rjmp  led_ns_yellow           ; Jump to NS Yellow LED pattern routine
chk_ar1:
    cpi   r16, STATE_ALL_RED_1    ; Compare state with All-Red 1 clearance (2)
    brne  chk_ew_prep             ; If not match, branch
    rjmp  led_all_red             ; Jump to All-Red LED pattern routine
chk_ew_prep:
    cpi   r16, STATE_EW_PREP      ; Compare state with EW Prep warning (3)
    brne  chk_ew_grn              ; If not match, branch
    rjmp  led_ew_yellow           ; Jump to EW Yellow LED pattern routine (EW Pre-Green warning)
chk_ew_grn:
    cpi   r16, STATE_EW_GREEN     ; Compare state with EW Green (4)
    brne  chk_ew_yel              ; If not match, branch
    rjmp  led_ew_green            ; Jump to EW Green LED pattern routine
chk_ew_yel:
    cpi   r16, STATE_EW_YELLOW    ; Compare state with EW Yellow (5)
    brne  chk_ar2                 ; If not match, branch
    rjmp  led_ew_yellow           ; Jump to EW Yellow LED pattern routine
chk_ar2:
    cpi   r16, STATE_ALL_RED_2    ; Compare state with All-Red 2 clearance (6)
    brne  chk_ns_prep             ; If not match, branch
    rjmp  led_all_red             ; Jump to All-Red LED pattern routine
chk_ns_prep:
    cpi   r16, STATE_NS_PREP      ; Compare state with NS Prep warning (7)
    brne  chk_ped                 ; If not match, branch
    rjmp  led_ns_yellow           ; Jump to NS Yellow LED pattern routine (NS Pre-Green warning)
chk_ped:
    cpi   r16, STATE_PEDESTRIAN   ; Compare state with Pedestrian crossing (8)
    brne  chk_night               ; If not match, branch
    rjmp  led_all_red             ; Jump to All-Red LED pattern routine (all red during pedestrian phase)
chk_night:
    cpi   r16, STATE_NIGHT        ; Compare state with Night Mode (9)
    brne  led_ret                 ; If not match, branch to return
    rjmp  led_night               ; Jump to Night Mode flashing LED pattern routine
led_ret:
    ret                           ; Return from subroutine

led_ns_green:
    ldi   r16, (1<<NS_GRN) | (1<<EW_RED) ; Create bitmask: turn ON North-South Green (PB2) and East-West Red (PB3)
    out   PORTB, r16              ; Write LED output bitmask directly to PORTB data register
    ret                           ; Return from subroutine

led_ns_yellow:
    ldi   r16, (1<<NS_YEL) | (1<<EW_RED) ; Create bitmask: turn ON North-South Yellow (PB1) and East-West Red (PB3)
    out   PORTB, r16              ; Write bitmask to PORTB
    ret                           ; Return from subroutine

led_all_red:
    ldi   r16, (1<<NS_RED) | (1<<EW_RED) ; Create bitmask: turn ON both North-South Red (PB0) and East-West Red (PB3)
    out   PORTB, r16              ; Write bitmask to PORTB
    ret                           ; Return from subroutine

led_ew_green:
    ldi   r16, (1<<NS_RED) | (1<<EW_GRN) ; Create bitmask: turn ON North-South Red (PB0) and East-West Green (PB5)
    out   PORTB, r16              ; Write bitmask to PORTB
    ret                           ; Return from subroutine

led_ew_yellow:
    ldi   r16, (1<<NS_RED) | (1<<EW_YEL) ; Create bitmask: turn ON North-South Red (PB0) and East-West Yellow (PB4)
    out   PORTB, r16              ; Write bitmask to PORTB
    ret                           ; Return from subroutine

led_night:
    lds   r17, flash_state        ; Load current night flash toggle state (0 or 1) from SRAM into r17
    tst   r17                     ; Test flash state value (sets Zero flag if r17 == 0)
    breq  led_night_off           ; If Zero flag is set (flash state is 0), branch to turn night flash yellows OFF
    ldi   r16, (1<<NS_YEL) | (1<<EW_YEL) ; If flash state is 1, create bitmask to turn ON both Yellow LEDs (PB1 and PB4)
    out   PORTB, r16              ; Write bitmask to PORTB
    ret                           ; Return from subroutine
led_night_off:
    clr   r16                     ; Clear r16 (0x00)
    out   PORTB, r16              ; Write 0x00 to PORTB to turn OFF both Yellow LEDs
    ret                           ; Return from subroutine

update_countdown_display:
    ; Input: r19 = countdown value (0..5), or 10 for blank display.
    push  r16                     ; Save register r16 onto stack
    push  r17                     ; Save register r17 onto stack
    push  r30                     ; Save Z-pointer low register (r30) onto stack
    push  r31                     ; Save Z-pointer high register (r31) onto stack

    cpi   r19, 10                 ; Compare input value in r19 against 10 (blank index)
    brlo  load_tables             ; If r19 < 10 (valid digit 0-9), proceed to load segment lookup tables
    ldi   r19, 10                 ; If r19 >= 10, force index to 10 (ensures safe fallback for blank display output)

load_tables:
    ; Fetch PORTD segment bit patterns (Segments A, B, C, D, E mapped to PD3..PD7)
    ldi   ZL, low(countdown_portd_table * 2) ; Load low byte of Flash program memory table base address into ZL (multiplied by 2 for byte addressing)
    ldi   ZH, high(countdown_portd_table * 2); Load high byte of Flash table address into ZH
    add   ZL, r19                 ; Add countdown digit index offset (0-10) to ZL pointer
    clr   r17                     ; Clear r17 (0x00)
    adc   ZH, r17                 ; Add carry bit to ZH high byte pointer if addition overflowed
    lpm   r16, Z                  ; Load Program Memory byte from Flash address pointed by Z into r16
.IF COMMON_CATHODE
    com   r16                     ; Invert active-low table values because hardware uses Common Cathode display (Active HIGH)
    andi  r16, 0b11111000         ; Apply bitmask to preserve only segment output pins PD3..PD7 (masking out PD0-PD2)
.ENDIF

    in    r17, PORTD              ; Read current PORTD register state into r17
    andi  r17, 0b00000111         ; Apply bitmask to preserve lower PORTD bits (PD0..PD2 button/UART lines)
    or    r16, r17                ; Combine calculated 7-segment output pattern with preserved lower PORTD bits
    out   PORTD, r16              ; Write final combined byte to PORTD output data register

    ; Fetch PORTC segment bit patterns (Segments F, G mapped to PC1, PC2)
    ldi   ZL, low(countdown_portc_table * 2) ; Load low byte of PORTC segment Flash table address into ZL
    ldi   ZH, high(countdown_portc_table * 2); Load high byte of PORTC table address into ZH
    add   ZL, r19                 ; Add digit index offset to ZL pointer
    clr   r17                     ; Clear r17
    adc   ZH, r17                 ; Add potential carry bit to ZH
    lpm   r16, Z                  ; Load program memory segment byte from Flash into r16
.IF COMMON_CATHODE
    com   r16                     ; Invert byte for Common Cathode active-HIGH display configuration
    andi  r16, 0b00000110         ; Apply bitmask to preserve only segment pins PC1 and PC2
.ENDIF

    in    r17, PORTC              ; Read current PORTC register state into r17
    andi  r17, 0b11111001         ; Apply bitmask to preserve PC0 (LDR ADC input) and higher PC pins
    or    r16, r17                ; Combine calculated segment bits with preserved PORTC pins
    out   PORTC, r16              ; Write final combined byte to PORTC output data register

    pop   r31                     ; Restore Z-pointer high register from stack
    pop   r30                     ; Restore Z-pointer low register from stack
    pop   r17                     ; Restore r17 register from stack
    pop   r16                     ; Restore r16 register from stack
    ret                           ; Return from subroutine

;==============================================================================
; SECTION 9: FLASH PROGRAM MEMORY LOOKUP TABLES (Digits 0-9)
;==============================================================================

; Tables stored Active-Low (0 = Segment ON). Inverted automatically when COMMON_CATHODE = 1.
; Indices 0..9 represent digits '0' through '9', Index 10 represents a blank display.

countdown_portd_table:
    ; Segments A, B, C, D, E mapped to PORTD pins PD3, PD4, PD5, PD6, PD7
    .DB 0b00000000, 0b11001000    ; Flash data bytes for Digits '0' and '1' segment patterns
    .DB 0b00100000, 0b10000000    ; Flash data bytes for Digits '2' and '3' segment patterns
    .DB 0b11001000, 0b10010000    ; Flash data bytes for Digits '4' and '5' segment patterns  
    .DB 0b00010000, 0b11000000    ; Flash data bytes for Digits '6' and '7' segment patterns
    .DB 0b00000000, 0b10000000    ; Flash data bytes for Digits '8' and '9' segment patterns
    .DB 0b11111000, 0b00000000    ; Flash data bytes for Index 10 (Blank/Off display output) and byte alignment padding

countdown_portc_table:
    ; Segments F, G mapped to PORTC pins PC1, PC2
    .DB 0b00000100, 0b00000110    ; Flash data bytes for Digits '0' and '1' segment patterns on PORTC
    .DB 0b00000010, 0b00000010    ; Flash data bytes for Digits '2' and '3' segment patterns on PORTC
    .DB 0b00000000, 0b00000000    ; Flash data bytes for Digits '4' and '5' segment patterns on PORTC
    .DB 0b00000000, 0b00000110    ; Flash data bytes for Digits '6' and '7' segment patterns on PORTC
    .DB 0b00000000, 0b00000000    ; Flash data bytes for Digits '8' and '9' segment patterns on PORTC
    .DB 0b00000110, 0b00000000    ; Flash data bytes for Index 10 (Blank/Off display output) and byte alignment padding