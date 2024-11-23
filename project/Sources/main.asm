; COE538 Final Project -- eebot Maze Robot
; By: Adrian Omoruyi, Rose Pagano
;*****************************************************************
;* This stationery serves as the framework for a                 *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                         *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                   *
;* Freescale CodeWarrior for the HC12 Program directory          *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup          ; export 'Entry' symbol
            ABSENTRY Entry                ; for absolute assembly: mark this as application entry point
            
; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc' 

;***************************************************************************************************
; equates section
;***************************************************************************************************

; State Machine Equates                                                                               
;-------------------------------

FWD_INT     EQU 69 ; 3 second delay (at 23Hz)
REV_INT     EQU 46 ; 3 second delay (at 23Hz)
LEFT_TRN_INT EQU 46 ; 2 second delay (at 23Hz)
RIGHT_TRN_INT EQU 46 ; 2 second delay (at 23Hz
REV_TRN_INT EQU 23 ; 2 second delay (at 23Hz)
START       EQU 0
FWD         EQU 1
REV         EQU 2
ALL_STP     EQU 3
LEFT_TRN    EQU 4
RIGHT_TRN   EQU 5
REV_TRN     EQU 6
LEFT_ALIGN  EQU 7
RIGHT_ALIGN EQU 8

; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
; -------------
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin

; Other codes
; -----------
NULL          EQU   00                    ; The string 'null terminator'
CR            EQU   $0D                   ; 'Carriage Return' character
SPACE         EQU   ' '                   ; The 'space' character

; variable/data section
; ---------------------
              ORG   $3800
             
DEFAULT_LINE   FCB   $9A                   ;NOTE!!! DEFAULT VALUE OVER WHITESPACE SHOULD BE ABOUT THE SAME
DEFAULT_BOW    FCB   $CB                   ;NOTE!!! DEFAULT VALUE OVER WHITESPACE IS ABOUT 40 (give or take) 
DEFAULT_PORT   FCB   $CB
DEFAULT_MID    FCB   $CB
DEFAULT_STBD   FCB   $CB

THRESHOLD_LINE FCB $20
THRESHOLD_BOW FCB $20
THRESHOLD_PORT FCB $20
THRESHOLD_MID FCB $20
THRESHOLD_STBD FCB $20             
             
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  3                       ; Current state register
T_TURN        ds.b  1                       ; time to stop turning
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit
NO_BLANK      ds.b  1                       ; Used in 'leading zero' blanking by BCD2ASC
BCD_SPARE     RMB   2        
T_FWD       ds.b 1 ; FWD time
T_REV       ds.b 1 ; REV time
T_LEFT_TRN   ds.b 1 ; LEFT_TURN time
T_RIGHT_TRN   ds.b 1 ; RIGHT_TURN time
T_REV_TRN   ds.b 1 ; REV_TURN time

    
             
; ------------------------------------------------------ 
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
; ------------------------------------------------------
SENSOR_LINE   FCB   $01                     ; Storage for guider sensor readings
SENSOR_BOW    FCB   $23                     ; Initialized to test values
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1 

TOP_LINE      RMB   20                      ; Top line of display
              FCB   NULL                    ; terminated by null
              
BOT_LINE      RMB   20                      ; Bottom line of display
              FCB   NULL                    ; terminated by null

CLEAR_LINE    FCC   '                  '    ; Clear the line of display
              FCB   NULL                    ; terminated by null

TEMP          RMB   1                       ; Temporary location



; code section
;***************************************************************************************************
              ORG   $4000                  ; Start of program text in memory
; Initialization
             
Entry:                                                                       
_Startup:

              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              JSR   INIT                   ; Initialize ports
              JSR   openADC                ; Initialize the ATD
              JSR   initLCD                ; Initialize the LCD
              JSR   CLR_LCD_BUF            ; Write 'space' characters to the LCD buffer 
              
              BSET DDRA,%00000011 ; STAR_DIR, PORT_DIR            
              BSET DDRT,%00110000 ; STAR_SPEED, PORT_SPEED 
              JSR initAD ; Initialize ATD converter
              JSR clrLCD ; Clear LCD & home cursor                
              
              LDX #msg1 ; Display msg1                            
              JSR putsLCD ; "                                                                             
              LDAA #$C0 ; Move LCD cursor to the 2nd row         
              JSR cmd2LCD                                       
              LDX #msg2 ; Display msg2                           
              JSR putsLCD ; "     
              JSR ENABLE_TOF ; Jump to TOF initialization --------
            
               
            
MAIN          
               JSR   G_LEDS_ON              ; Enable the guider LEDs
               
               
               
              JSR   UPDT_DISPL
              LDAA  CRNT_STATE                 
              JSR   DISPATCHER 
               
               
               
               JSR   READ_SENSORS           ; Read the 5 guider sensor
              
               JSR   G_LEDS_OFF             ; Disable the guider LEDs  
               
              ; JSR   DISPLAY_SENSORS        ; and write them to the LCD
               
              ; LDY   #6000                  ; 300 ms delay to avoid
              ; JSR   del_50us               ; display artifacts

                           
              BRA   MAIN                   ; Loop forever

; data section
*******************************************************************                        
msg1        dc.b "Battery volt ",0
msg2        dc.b "State   ",0
tab         dc.b "START   ",0
            dc.b "FWD     ",0
            dc.b "REV     ",0
            dc.b "ALL_STP ",0
            dc.b "LFT_TRN ",0
            dc.b "RGHT_TRN",0
            dc.b "REV_TRN ",0   
            
; dispatcher subroutines
    
;***************************************************************************************************

DISPATCHER  CMPA #START       ; If it’s the START state -----------------
            BNE NOT_START     ;                                          |
            JSR START_ST      ; then call START_ST routine               D
            BRA DISP_EXIT     ; and exit                                 I
                              ;                                          S
NOT_START   CMPA #FWD         ; 
            BNE NOT_FWD       ;                                          
            JSR FWD_ST        ; 
            BRA DISP_EXIT     ; 
            
NOT_FWD     CMPA #REV         ; 
            BNE NOT_REV       ;                                          
            JSR REV_ST        ; 
            BRA DISP_EXIT     ; 
            
NOT_REV     CMPA #ALL_STP     ; 
            BNE NOT_ALL_STP   ;                                          
            JSR ALL_STP_ST    ; 
            BRA DISP_EXIT     ; 
            
NOT_ALL_STP CMPA #LEFT_TRN     ; 
            BNE NOT_LEFT_TRN   ;                                          
            JSR LEFT_TRN_ST    ; 
            BRA DISP_EXIT     ; 
                              ;                                                                                      
NOT_LEFT_TRN CMPA #RIGHT_TRN     ; Else if it’s the REV_TRN state           C
            BNE NOT_RIGHT_TRN   ;                                          H
            JSR RIGHT_TRN_ST    ; then call REV_TRN_ST routine             E
            BRA DISP_EXIT     ; and exit                                 R
            
NOT_RIGHT_TRN CMPA #LEFT_ALIGN     ; Else if it’s the REV_TRN state           C
            BNE NOT_LEFT_ALIGN   ;                                          H
         ;   JSR LEFT_ALIGN_ST    ; then call REV_TRN_ST routine             E
            BRA DISP_EXIT     ; and exit                                 R
            
                        
NOT_LEFT_ALIGN CMPA #RIGHT_ALIGN     ; Else if it’s the REV_TRN state           C
            BNE NOT_RIGHT_ALIGN   ;                                          H
         ;   JSR RIGHT_ALIGN_ST    ; then call REV_TRN_ST routine             E
            BRA DISP_EXIT     ; and exit                                 R
            
                        
NOT_RIGHT_ALIGN CMPA #REV_TRN     ; Else if it’s the REV_TRN state           C
            BNE NOT_REV_TRN   ;                                          H
            JSR REV_TRN_ST    ; then call REV_TRN_ST routine             E
            BRA DISP_EXIT     ; and exit                                 R
                              ;                                          |
NOT_REV_TRN SWI               ; Else the CRNT_ST is not defined, so stop |
DISP_EXIT   RTS               ; Exit from the state dispatcher ----------

*******************************************************************















*******************************************************************
START_ST    BRCLR PORTAD0,$04,NO_FWD          ; If /FWD_BUMP
            JSR INIT_FWD                          ; Initialize the FORWARD state
            MOVB #FWD,CRNT_STATE           ; Go into the FORWARD state
            BRA START_EXIT
NO_FWD      NOP                                   ; Else
START_EXIT  RTS                                   ; return to the MAIN routine
                                 
*******************************************************************
FWD_ST      BRSET PORTAD0,$04,NO_FWD_BUMP ; If FWD_BUMP then

            MOVB #REV_TRN,CRNT_STATE ; set the state to REVERSE
            JSR UPDT_DISPL
           ; JSR INIT_REV ; initialize the REVERSE routine
            
           ; LDY #6000
           ; JSR del_50us
            JSR INIT_LEFT_TRN
            
            LDY #27000
            JSR del_50us
            
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE ;
            
            JMP FWD_EXIT ; and return
            
NO_FWD_BUMP BRSET PORTAD0,$08,NO_REAR_BUMP ; If REAR_BUMP, then we should stop
            MOVB #ALL_STP,CRNT_STATE ; and change state to ALL_STOP
            JSR INIT_ALL_STP ; so initialize the ALL_STOP state
            JMP FWD_EXIT ; and return
            
NO_REAR_BUMP
            
            LDAA SENSOR_LINE      ;Load the sensor reading into A
            SUBA DEFAULT_LINE     ;Subtract the default value for the sensor
            SUBA THRESHOLD_LINE   ;Subtract the threshold value
            BPL ADJUSTR           ;If the result is positive, that means the sensor reading increased (past
                                  ;the threshold) and the bot must adjust to the right 
                                  
            LDAA SENSOR_LINE      ;Load the sensor reading into A
            SUBA DEFAULT_LINE     ;Subtract the default value for the sensor
            ADDA THRESHOLD_LINE   ;Add the threshold value
            BMI ADJUSTL           ;If the result is negative, that means the sensor reading decreased (past
                                  ;the threshold) and the bot must adjust to the left
            
            
            ; LDAA SENSOR_BOW
            ; ADDA UNCERTAIN_BOW
            ; CMPA DEFAULT_BOW
            ; BPL JUNCTION1
             
            ; LDAA SENSOR_PORT
            ; ADDA UNCERTAIN_PORT
            ; CMPA DEFAULT_PORT
            ; BMI JUNCTION2
             
            ; LDAA SENSOR_STBD
            ; ADDA UNCERTAIN_STBD
            ; CMPA DEFAULT_STBD
            ; BMI JUNCTION3
             
            ; JMP NO_LEFT_TRN
             
             
;             JSR INIT_LEFT_TRN ; initialize the FORWARD_TURN state
;             MOVB #LEFT_TRN,CRNT_STATE ; and go to that state
;             JMP FWD_EXIT


NO_FWD_TRN  NOP                ; Else
FWD_EXIT    RTS                ; return to the MAIN routine
*******************************************************************
ADJUSTL     JSR INIT_LEFT_TRN
            LDY #500
            JSR del_50us
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE 
            RTS
            
ADJUSTR     JSR INIT_RIGHT_TRN
            LDY #500
            JSR del_50us
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE 
            RTS

JUNCTION1   JSR INIT_LEFT_TRN
            LDY #13500
            JSR del_50us
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE 
            RTS
            
JUNCTION2   JSR INIT_RIGHT_TRN
            LDY #13500
            JSR del_50us
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE 
            RTS 
            
JUNCTION3   JSR INIT_LEFT_TRN
            LDY #13500
            JSR del_50us
            JSR INIT_FWD
            MOVB #FWD,CRNT_STATE
            RTS          
*******************************************************************
REV_ST      LDAA TOF_COUNTER   ; If Tc>Trev then
            CMPA T_REV         ; the robot should make a FWD turn
            BNE NO_REV_TRN     ; so
            MOVB #REV_TRN,CRNT_STATE ; set state to REV_TRN
        ;    JSR INIT_REV_TRN   ; initialize the REV_TRN state
            MOVB #REV_TRN,CRNT_STATE ; set state to REV_TRN
            BRA REV_EXIT       ; and return
NO_REV_TRN  NOP                ; Else
REV_EXIT    RTS                ; return to the MAIN routine
*******************************************************************
ALL_STP_ST  BRSET PORTAD0,$04,NO_START    ; If FWD_BUMP
            BCLR PTT,%00110000            ; initialize the START state (both motors off)
            MOVB #START,CRNT_STATE        ; set the state to START
            BRA ALL_STP_EXIT              ; and return
NO_START    NOP                           ; Else
ALL_STP_EXIT RTS                          ; return to the MAIN routine
*******************************************************************
LEFT_TRN_ST  LDAA TOF_COUNTER              ; If Tc>Tfwdturn then
            CMPA T_LEFT_TRN                ; the robot should go FWD
            BNE NO_LEFT_TRN                 ; so
            JSR INIT_FWD                  ; initialize the FWD state
            MOVB #FWD,CRNT_STATE          ; set state to FWD
            BRA LEFT_TRN_EXIT              ; and return
NO_LEFT_TRN   NOP                             ; Else
LEFT_TRN_EXIT RTS                          ; return to the MAIN routine
*******************************************************************
RIGHT_TRN_ST  LDAA TOF_COUNTER              ; If Tc>Tfwdturn then
            CMPA T_RIGHT_TRN                ; the robot should go FWD
            BNE NO_RIGHT_TURN                 ; so
            JSR INIT_FWD                  ; initialize the FWD state
            MOVB #FWD,CRNT_STATE          ; set state to FWD
            BRA RIGHT_TRN_EXIT              ; and return
NO_RIGHT_TURN   NOP                             ; Else
RIGHT_TRN_EXIT RTS
*******************************************************************
REV_TRN_ST LDAA TOF_COUNTER               ; If Tc>Trevturn then
            CMPA T_REV_TRN                ; the robot should go FWD
            BNE NO_FWD_RT                 ; so
            MOVB #FWD,CRNT_STATE          ; set state to FWD
            JSR INIT_FWD                  ; initialize the FWD state
            MOVB #FWD,CRNT_STATE          ; set state to FWD
            BRA REV_TRN_EXIT              ; and return
NO_FWD_RT NOP                             ; Else
REV_TRN_EXIT RTS                          ; return to the MAIN routine














*******************************************************************
INIT_FWD    BCLR PORTA,%00000011          ; Set FWD direction for both motors
            BSET PTT,%00110000            ; Turn on the drive motors
            ;LDAA TOF_COUNTER              ; Mark the fwd time Tfwd
            ;ADDA #FWD_INT
            ;STAA T_FWD
            RTS
*******************************************************************
INIT_REV    BSET PORTA,%00000011          ; Set REV direction for both motors
            BSET PTT,%00110000            ; Turn on the drive motors
            LDAA TOF_COUNTER              ; Mark the fwd time Tfwd
            ADDA #REV_INT
            STAA T_REV
            RTS
*******************************************************************
INIT_ALL_STP BCLR PTT,%00110000           ; Turn off the drive motors
            RTS
*******************************************************************
INIT_LEFT_TRN BSET PORTA,%00000001         ; Set REV dir. for STARBOARD (right) motor
            LDAA TOF_COUNTER              ; Mark the fwd_turn time Tfwdturn
            ADDA #LEFT_TRN_INT
            STAA T_LEFT_TRN
            RTS
*******************************************************************
INIT_RIGHT_TRN BCLR PORTA,%00000010         ; Set FWD dir. for STARBOARD (right) motor
            LDAA TOF_COUNTER              ; Mark the fwd time Tfwd
            ADDA #REV_TRN_INT
            STAA T_REV_TRN
            RTS












; utility subroutines
*******************************************************************

;       Initialize Sensors
INIT              BCLR   DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                  BSET   DDRA,$FF  ; Make PORTA an output (DDRA @ $0002)
                  BSET   DDRB,$FF  ; Make PORTB an output (DDRB @ $0003)
                  BSET   DDRJ,$C0  ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS


;***************************************************************************************************
;        Initialize ADC              
openADC           MOVB   #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
                  LDY    #1           ; Wait for 50 us for ADC to be ready
                  JSR    del_50us     ; - " -
                  MOVB   #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                  MOVB   #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                  RTS
                  
;***************************************************************************************************
;                           Clear LCD Buffer
; This routine writes ?space? characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
; display routine should maintain the buffer properly.
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

;***************************************************************************************************     
; String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
; Y contains first address of destination
STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; and do it again

STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS  
 
;***************************************************************************************************    
;                                   Guider LEDs ON                                                 |
; This routine enables the guider LEDs so that readings of the sensor                              |
; correspond to the ?illuminated? situation.                                                       |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                 |
                  RTS                                                                             ;|

;***************************************************************************************************     
;                                   Guider LEDs OFF                                                |
; This routine disables the guider LEDs. Readings of the sensor                                    |
; correspond to the ?ambient lighting? situation.                                                  |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                               |
                  RTS                                                                             ;|  
                  
                  
;***************************************************************************************************
;                               Read Sensors

READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #400           ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS

;***************************************************************************************************   
;                               Select Sensor
;***************************************************************************************************    
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS
                  
DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9

DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           ; "
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS                                     
                  
;*******************************************************************
;* Initialization of the LCD
;*******************************************************************
initLCD     BSET DDRB,%11111111 ; configure pins PS7,PS6,PS5,PS4 for output
            BSET DDRJ,%11000000 ; configure pins PE7,PE4 for output
            LDY #2000 ; wait for LCD to be ready
            JSR del_50us ; -"-
            LDAA #$28 ; set 4-bit data, 2-line display
            JSR cmd2LCD ; -"-
            LDAA #$0C ; display on, cursor off, blinking off
            JSR cmd2LCD ; -"-
            LDAA #$06 ; move cursor right after entering a character
            JSR cmd2LCD ; -"-
            RTS
            
            
;*******************************************************************
;* Clear display and home cursor *
;*******************************************************************
clrLCD      LDAA #$01 ; clear cursor and return to home position
            JSR cmd2LCD ; -"-
            LDY #40 ; wait until "clear cursor" command is complete
            JSR del_50us ; -"-
            RTS


;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns. *
;*******************************************************************
del_50us:   PSHX ;2 E-clk
eloop:      LDX #30 ;2 E-clk -
iloop:      PSHA ;2 E-clk |
            PULA ;3 E-clk |
            PSHA ;2 E-clk | 50us
            PULA ;3 E-clk |
            PSHA ;2 E-clk |
            PULA ;3 E-clk |
            PSHA ;2 E-clk |
            PULA ;3 E-clk |
            PSHA ;2 E-clk |
            PULA ;3 E-clk |
            PSHA ;2 E-clk |
            PULA ;3 E-clk |
           
            NOP ;1 E-clk |
            NOP ;1 E-clk |
            DBNE X,iloop ;3 E-clk -
            DBNE Y,eloop ;3 E-clk
            PULX ;3 E-clk
            RTS ;5 E-clk
            
;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:    BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
            JSR dataMov ; send data to IR
            RTS
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *
;*******************************************************************
putsLCD     LDAA 1,X+ ; get one character from the string
            BEQ donePS ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
donePS      RTS

;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *
;*******************************************************************
putcLCD     BSET LCD_CNTR,LCD_RS ; select the LCD Data register (DR)
            JSR dataMov ; send data to DR
            RTS
            
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov     BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
            STAA LCD_DAT ; send the upper 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LSLA ; match the lower 4 bits with the LCD data pins
            LSLA ; -"-
            LSLA ; -"-
            LSLA ; -"-
            BSET LCD_CNTR,LCD_E ; pull the LCD E signal high
            STAA LCD_DAT ; send the lower 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LDY #1 ; adding this delay will complete the internal
            JSR del_50us ; operation for most instructions
            RTS

initAD      MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
            JSR del_50us ;wait for 50 us
            MOVB #$00,ATDCTL3 ;8 conversions in a sequence
            MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
            RTS
            
LCD_POS_CRSR ORAA #%10000000 ; Set the high bit of the control word
             JSR cmd2LCD ; and set the cursor address
             RTS            
            
;*****************************************************************
;             Integer to BCD Conversion Routine
;******************************************************************

int2BCD  XGDX            ;Save the binary number into .X
         LDAA #0         ;Clear the BCD_BUFFER
         STAA TEN_THOUS
         STAA THOUSANDS
         STAA HUNDREDS
         STAA TENS
         STAA UNITS
         STAA BCD_SPARE
         STAA BCD_SPARE+1
*
         CPX #0         ;Check for a zero input
         BEQ CON_EXIT   ;and if so, exit
*
         XGDX           ;Not zero, get the binary number back to .D as dividend
         LDX #10        ;Setup 10 (Decimal!) as the divisor
         IDIV           ;Divide: Quotient is now in .X, remainder in .D
         ANDB #$0F      ;Clear high nibble of remainder
         STAB UNITS     ;and store it.
         CPX #0         ;If quotient is zero,
         BEQ CON_EXIT   ;then exit
*
         XGDX           ;else swap first quotient back into .D
         LDX #10        ;and setup for another divide by 10
         IDIV
         ANDB #$0F
         STAB TENS
         CPX #0
         BEQ CON_EXIT
*
         XGDX           ;Swap quotient back into .D
         LDX #10        ;and setup for another divide by 10
         IDIV
         ANDB #$0F
         STAB HUNDREDS
         CPX #0
         BEQ CON_EXIT
*
         XGDX           ;Swap quotient back into .D
         LDX #10        ;and setup for another divide by 10
         IDIV
         ANDB #$0F
         STAB THOUSANDS
         CPX #0
         BEQ CON_EXIT
*
         XGDX           ;Swap quotient back into .D
         LDX #10        ;and setup for another divide by 10
         IDIV
         ANDB #$0F
         STAB TEN_THOUS
*
CON_EXIT RTS            ;We're done the conversion

;*****************************************************************
;             Binary to ASCII
;******************************************************************
HEX_TABLE             FCC '0123456789ABCDEF' ; Table for converting value

BIN2ASC               PSHA               ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                
                      LDAA 0,X            ; Get the LSnibble character
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the LSnibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ;  into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the MSnibble 
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                                                               
                      LDAA 0,X            ; Get the MSnibble character into ACCA
                      PULB                ; Retrieve the LSnibble character into ACCB
                      RTS
           
;****************************************************************
;                BCD to ASCII Conversion Routine
;****************************************************************

BCD2ASC  LDAA #0         ;Initialize the blanking flag
         STAA NO_BLANK
*
C_TTHOU  LDAA TEN_THOUS  ;Check the 'ten_thousands' digit
         ANDA #$0F       ;Clear the high nibble
         ORAA NO_BLANK
         BNE NOT_BLANK1
*
ISBLANK1 LDAA #' '        ;It's blank
         STAA TEN_THOUS   ;so store a space
         BRA C_THOU       ;and check the 'thousands' digit
*
NOT_BLANK1 LDAA TEN_THOUS ;Get the 'ten_thousands' digit
           ORAA #$30      ;Convert to ascii
           STAA TEN_THOUS
           LDAA #$1       ;Signal that we have seen a 'non-blank' digit
           STAA NO_BLANK
*
C_THOU     LDAA THOUSANDS ;Check the thousands digit for blankness
           ANDA #$0F      ;Clear the high nibble
           ORAA NO_BLANK  ;If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK2
*
ISBLANK2   LDAA #' '      ;Thousands digit is blank
           STAA THOUSANDS ;so store a space
           BRA C_HUNS     ;and check the hundreds digit
*
NOT_BLANK2 LDAA THOUSANDS ;(similar to 'ten_thousands case)
           ORAA #$30
           STAA THOUSANDS
           LDAA #$1
           STAA NO_BLANK
*
C_HUNS     LDAA HUNDREDS  ;Check the hundreds digit for blankness
           ANDA #$0F      ;Clear the high nibble
           ORAA NO_BLANK  ;If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK3
*
ISBLANK3   LDAA #' '      ;Hundreds digit is blank
           STAA HUNDREDS  ;so store a space
           BRA C_TENS     ;and check the tens digit
*
NOT_BLANK3 LDAA HUNDREDS  ;(similar to 'ten_thousands case)
           ORAA #$30
           STAA HUNDREDS
           LDAA #$1
           STAA NO_BLANK
*
C_TENS     LDAA TENS      ;Check the tens digit for blankness
           ANDA #$0F      ;Clear the high nibble
           ORAA NO_BLANK  ;If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK4
*
ISBLANK4   LDAA #' '      ;Tens digit is blank
           STAA TENS      ;so store a space
           BRA C_UNITS    ;and check the units digit
*
NOT_BLANK4 LDAA TENS      ;(similar to 'ten_thousands case)
           ORAA #$30
           STAA TENS
           LDAA #$1
           STAA NO_BLANK
*
C_UNITS    LDAA UNITS     ;No blank check necessary, convert to ascii.
           ANDA #$0F
           ORAA #$30
           STAA UNITS
*
           RTS            ;We're done
           
           
           
************************************************************
ENABLE_TOF  LDAA #%10000000
            STAA TSCR1 ; Enable TCNT
            STAA TFLG2 ; Clear TOF
            LDAA #%10000100 ; Enable TOI and select prescale factor equal to 16
            STAA TSCR2
            RTS
************************************************************
TOF_ISR     INC TOF_COUNTER
            LDAA #%10000000; Clear
            STAA TFLG2 ; TOF
            RTI
*******************************************************************
* Update Display (Battery Voltage + Current State) *
*******************************************************************
UPDT_DISPL      MOVB  #$90,ATDCTL5              ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$8D                      ; move LCD cursor to the 1st row, end of msg1
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                
;------------------------------------------------
                LDAA  #$C6                      ; Move LCD cursor to the 2nd row, end of msg2
                JSR   cmd2LCD                   ;
                
                LDAB  CRNT_STATE                ; Display current state
                LSLB                            ; "
                LSLB                            ; "
                LSLB                            ; "
                LDX   #tab                      ; "
                ABX                             ; "
                JSR   putsLCD                   ; "
                RTS
*******************************************************************
* Interrupt Vectors *
*******************************************************************
            ORG $FFFE
            DC.W Entry ; Reset Vector
            ORG $FFDE
            DC.W TOF_ISR ; Timer Overflow Interrupt Vector