// CONFIG1H
#pragma config OSC = INTIO67     // Internal oscillator block, RA6/RA7 as I/O
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor disabled
#pragma config IESO = ON         // Internal/External Oscillator Switchover enabled

// CONFIG2L
#pragma config PWRT = OFF        // Power-up Timer disabled
#pragma config BOREN = SBORDIS   // Brown-out Reset enabled in hardware only
#pragma config BORV = 3          // Brown Out Reset Voltage (minimum)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer disabled
#pragma config WDTPS = 1         // Watchdog Timer Postscale 1:1

// CONFIG3H
#pragma config CCP2MX = PORTC    // CCP2 input/output multiplexed with RC1
#pragma config PBADEN = ON       // PORTB A/D enabled on reset
#pragma config LPT1OSC = OFF     // Low-power Timer1 oscillator disabled
#pragma config MCLRE = ON        // MCLR pin enabled; RE3 disabled

// CONFIG4L
#pragma config STVREN = ON       // Stack full/underflow will cause reset
#pragma config LVP = OFF         // Single-Supply ICSP disabled
#pragma config XINST = OFF       // Extended instruction set disabled

// CONFIG5L
#pragma config CP0 = OFF         // Code Protection Block 0 disabled
#pragma config CP1 = OFF         // Code Protection Block 1 disabled
#pragma config CP2 = OFF         // Code Protection Block 2 disabled
#pragma config CP3 = OFF         // Code Protection Block 3 disabled

// CONFIG5H
#pragma config CPB = OFF         // Boot Block Code Protection disabled
#pragma config CPD = OFF         // Data EEPROM Code Protection disabled

// CONFIG6L
#pragma config WRT0 = OFF        // Write Protection Block 0 disabled
#pragma config WRT1 = OFF        // Write Protection Block 1 disabled
#pragma config WRT2 = OFF        // Write Protection Block 2 disabled
#pragma config WRT3 = OFF        // Write Protection Block 3 disabled

// CONFIG6H
#pragma config WRTC = OFF        // Configuration Register Write Protection disabled
#pragma config WRTB = OFF        // Boot Block Write Protection disabled
#pragma config WRTD = OFF        // Data EEPROM Write Protection disabled

// CONFIG7L
#pragma config EBTR0 = OFF       // Table Read Protection Block 0 disabled
#pragma config EBTR1 = OFF       // Table Read Protection Block 1 disabled
#pragma config EBTR2 = OFF       // Table Read Protection Block 2 disabled
#pragma config EBTR3 = OFF       // Table Read Protection Block 3 disabled

// CONFIG7H
#pragma config EBTRB = OFF       // Boot Block Table Read Protection disabled

#include <ctype.h>
#include <pic18f4520.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#define _XTAL_FREQ 4000000
#define STR_MAX 100
#define VR_MAX ((1 << 10) - 1)
#define SERVO_MIN 500
#define SERVO_MID 1450
#define SERVO_MAX 2400


// ================== GLOBAL VARIABLES ==================
char buffer[STR_MAX];
int buffer_size = 0;
bool btn_interr = false;
int current_servo_angle = 0;
int danger= 0;

// Movement state (continuous command sending for car control)
typedef enum {
    MOVE_NONE,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} MoveState;

// Buzzer state machine
typedef enum {
    BUZZ_IDLE,
    BUZZ_ON,
    BUZZ_OFF
} BuzzState;

volatile BuzzState buzz_state = BUZZ_IDLE;
volatile unsigned long buzz_next_time = 0;
volatile unsigned int buzz_interval_ms = 0; // buzzer interval in ms
volatile unsigned long next_beep_time = 0;
volatile bool beep_active = false;
volatile unsigned int beep_interval_ms = 0;
volatile bool buzzer_loop_active = false;

volatile MoveState current_move = MOVE_NONE;
volatile unsigned long last_move_time_ms = 0;
volatile unsigned long system_time_ms = 0;
volatile unsigned long last_ultra_update_ms = 0;
volatile float last_distance_cm = 0;

// Laser timing control
volatile unsigned long laser_on_until = 0;

// Timer-related global variables (buzzer / Timer1)
volatile unsigned int buzzer_reload = 0;
volatile bool buzzer_enable = false;

// Danger state check (distance < threshold for a duration)
volatile unsigned long danger_enter_time_ms = 0;
volatile bool danger_triggered = false;

// ================== MOTOR PIN DEFINE ==================
#define IN1 LATBbits.LATB0
#define IN2 LATBbits.LATB1
#define IN3 LATBbits.LATB2
#define IN4 LATBbits.LATB3

// ===== TM1637 4-digit 7-seg on RD4/RD5 =====
#define TM_CLK      LATDbits.LATD4
#define TM_DIO      LATDbits.LATD5
#define TM_CLK_TRIS TRISDbits.TRISD4
#define TM_DIO_TRIS TRISDbits.TRISD5

// ===== LASER on RC5 =====
#define LASER LATCbits.LATC5



//// ================== LED ==================
//int get_LED() { return (LATD >> 4) & 0x0F; }
//void set_LED(int v) { LATD = (v << 4); }

// ================== UART ==================
// UART putch for printf redirection
void putch(char data) {
    while (!TXSTAbits.TRMT);
    TXREG = data;

    // Append '\r' after '\n' for serial terminals
    if (data == '\n') {
        while (!TXSTAbits.TRMT);
        TXREG = '\r';
    }
}

// Clear UART input buffer
void ClearBuffer() {
    for (int i = 0; i < STR_MAX; i++) buffer[i] = '\0';
    buffer_size = 0;
}

// Read one UART byte into buffer and echo back
void MyusartRead() {
    char data = RCREG;
    if (!isprint(data) && data != '\r') return;

    buffer[buffer_size++] = data;
    putch(data);
}

// If a full line ends with '\r', copy it to str and return 1
int GetString(char *str) {
    if (buffer_size > 0 && buffer[buffer_size - 1] == '\r') {
        buffer[--buffer_size] = '\0';
        strcpy(str, buffer);
        ClearBuffer();
        return 1;
    }
    return 0;
}

// ================== INITIALIZE ==================
void Initialize() {
    OSCCONbits.IRCF = 0b110; // 4 MHz internal oscillator

//    TRISD &= 0x0F;
//    LATD &= 0x0F;

    TRISB = 0b00000000; // Motor pins output on PORTB
    LATB = 0;

    // PWM servo configuration (CCP1 + Timer2)
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS = 0b11;
    CCP1CONbits.CCP1M = 0b1100;
    PR2 = 0x9B;
    TRISCbits.TRISC2 = 0;

    // UART configuration (9600 baud @ 4MHz)
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    TXSTAbits.SYNC = 0;
    BAUDCONbits.BRG16 = 0;
    TXSTAbits.BRGH = 1;
    SPBRG = 25; // 9600 baud
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1;
    RCSTAbits.CREN = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 1;   // UART RX interrupt set as HIGH PRIORITY

    // ADC configuration (RA0/AN0)
    TRISAbits.RA0 = 1;
    ADCON1bits.PCFG = 0b1110;
    ADCON0bits.CHS = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON2bits.ADFM = 1;
    ADCON2bits.ADCS = 0;
    ADCON2bits.ACQT = 0b001;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO = 1;

    // Interrupts
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;

    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;

    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
    
    // HC-SR04 ultrasonic sensor
    TRISDbits.TRISD2 = 0; // TRIG output
    TRISDbits.TRISD3 = 1; // ECHO input
    LATDbits.LD2 = 0;     // TRIG low
    
    // Buzzer
    TRISDbits.TRISD1 = 0; // RD1 output (buzzer)
    LATDbits.LATD1 = 0;   // buzzer OFF
    
    // Laser
    TRISCbits.TRISC5 = 0; // RC5 output (laser)
    LATCbits.LATC5 = 0;   // laser OFF
    
    // TM1637 7-seg display
    TRISDbits.TRISD4 = 0;  // TM1637 CLK
    TRISDbits.TRISD5 = 0;  // TM1637 DIO
    LATDbits.LATD4 = 1;    // TM idle high
    LATDbits.LATD5 = 1;    // TM idle high
}

void Timer1_Init_us(unsigned int us) {

    T1CONbits.TMR1ON = 0;
    T1CONbits.TMR1CS = 0;   // Fosc/4
    T1CONbits.T1CKPS = 0;  // 1:1

    buzzer_reload = 65536 - us;
    TMR1H = buzzer_reload >> 8;
    TMR1L = buzzer_reload & 0xFF;

    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    IPR1bits.TMR1IP = 0;   // LOW priority

    T1CONbits.TMR1ON = 1;
}


// ================== SERVO CONTROL ==================
int set_servo_angle(int angle) {
    int current = (CCPR1L << 2) + CCP1CONbits.DC1B;
    int target = (int)((500 + (double)(angle + 90) / 180 * (2400 - 500)));

    btn_interr = false;

    while (current != target) {
        if (btn_interr) return -1;

        if (current < target) current++;
        else current--;

        CCPR1L = (current >> 2);
        CCP1CONbits.DC1B = (current & 0b11);
        __delay_ms(1);
    }

    current_servo_angle = angle;
    return 0;
}

void variable_register_changed(int value) {}
void button_pressed() {}
void keyboard_input(char *str);

// ================== INTERRUPT ==================
void __interrupt(high_priority) H_ISR() {

    // ===== ADC Interrupt =====
    if (PIR1bits.ADIF) {
        int value = (ADRESH << 8)+ ADRESL;
        variable_register_changed(value);
        PIR1bits.ADIF = 0;
    }

    // ===== Button Interrupt (INT0) =====
    if (INTCONbits.INT0IF) {
        button_pressed();
        btn_interr = true;
        __delay_ms(50);      // Debounce delay
        INTCONbits.INT0IF = 0;
    }

    // ===== UART Interrupt (HIGH PRIORITY) =====
    if (PIR1bits.RCIF) {
        if (RCSTAbits.OERR) {
            CREN = 0;
            CREN = 1;
        }
        MyusartRead();
    }
}

// Low priority ISR (Timer1 for buzzer toggle)
void __interrupt(low_priority) L_ISR() {

    if (PIR1bits.TMR1IF) {

        if (buzzer_enable) {
            LATDbits.LATD1 ^= 1;   // toggle buzzer pin
        } else {
            LATDbits.LATD1 = 0;
        }

        TMR1H = buzzer_reload >> 8;
        TMR1L = buzzer_reload & 0xFF;

        PIR1bits.TMR1IF = 0;
    }
}


int delay(double sec) {
    btn_interr = false;
    for (int i = 0; i < sec * 1000 / 100; i++) {
        if (btn_interr) return -1;
//        float d = ultrasonic_read_cm();
//        buzzer_by_distance(d);
        __delay_ms(100);
    }
    return 0;
}

volatile int turn=0; 
volatile int stop_front=0;
// ================== MOTOR CONTROL ==================
void motor_stop() {
    IN1 = IN2 = IN3 = IN4 = 0;
}

void motor_forward() {
    //IN1 = 1; IN2 = 0;
    //IN3 = 1; IN4 = 0;
    IN1 = 0; IN2 = 1;
    IN3 = 0; IN4 = 1;
}

void motor_backward() {
    //IN1 = 0; IN2 = 1;
    //IN3 = 0; IN4 = 1;
    IN1 = 1; IN2 = 0;
    IN3 = 1; IN4 = 0;
}

void motor_left() {
    IN1 = 0; IN2 = 1;
    IN3 = 1; IN4 = 0;
}

void motor_right() {
    IN1 = 1; IN2 = 0;
    IN3 = 0; IN4 = 1;
}

// ================== KEYBOARD COMMAND ==================
void keyboard_input(char *str) {
    if (strlen(str) == 0) return;

    char c = str[0];
    if(danger== 0){
        switch (c) {
            case 'F':
                if(stop_front== 0){
                    motor_forward();
                    turn=0;
                    current_move = MOVE_FORWARD;
                    last_move_time_ms = system_time_ms;
                    printf("[CMD] Forward\n");
               
                    break;
                }
                break;

            case 'B':
                motor_backward();
                turn=0;
                current_move = MOVE_BACKWARD;
                last_move_time_ms = system_time_ms;
                printf("[CMD] Backward\n");
                break;

            case 'L':
                motor_left();
                turn=1;
                current_move = MOVE_LEFT;
                last_move_time_ms = system_time_ms;
                printf("[CMD] Left\n");
                break;

            case 'R':
                motor_right();
                turn=1;
                current_move = MOVE_RIGHT;
                last_move_time_ms = system_time_ms;
                printf("[CMD] Right\n");
                break;

            case 'S':
                motor_stop();
                turn=0;
                current_move = MOVE_NONE;
                printf("[CMD] Stop\n");
                break;

            case 'H':
                LASER = 1;                         // turn on laser
                laser_on_until = system_time_ms + 500; // keep ON for 5s (based on 10ms tick)
                printf("[CMD] Laser ON 5s\n");
                set_servo_angle(90);
               
                set_servo_angle(-90);
               
                set_servo_angle(90);
                
                set_servo_angle(-90);
                
                set_servo_angle(90);
                
                set_servo_angle(-90);
                
                break;

            default:
                printf("[ERROR] Invalid command: %c\n", c);
                printf("Valid: F/B/L/R/S\n");
                motor_stop();
                break;
        }
    }
}

// void buzzer_beep(double s) {
//     LATDbits.LATD1 = 1;
//     delay(s);
//     LATDbits.LATD1 = 0;
// }

// Start buzzer at given frequency (Hz)
void buzzer_start(unsigned int freq) {
    unsigned int half_us = 1000000 / (freq * 2);
    Timer1_Init_us(half_us);
    buzzer_enable = true;
}

// Stop buzzer output
void buzzer_stop(void) {
    buzzer_enable = false;
    LATDbits.LATD1 = 0;
}


void to_safe(){
    motor_backward();
    delay(2);
    motor_stop();
}

unsigned int calc_interval(float d) {
    if (d < 20)   return 250;
    if (d < 40)  return 500;
    if (d < 60)  return 1000;
    return 0;    // >= 60cm
}


void buzzer_by_distance(float distance) {

    unsigned int new_interval = calc_interval(distance);

    /* ================= Danger check: <20cm for 5 seconds ================= */
    if (distance < 20) {

        // First time entering <20cm zone
        if (danger_enter_time_ms == 0) {
            danger_enter_time_ms = system_time_ms;
            danger_triggered = false;
        }

        // If staying in danger zone for 5 seconds, execute safe behavior
        if (!danger_triggered &&
            (system_time_ms - danger_enter_time_ms >= 5000)) {

            danger_triggered = true;
            printf("[DANGER] <20cm for 5s -> TO_SAFE\n");
            to_safe();
        }
    }
    else {
        // Reset danger timer/state when leaving danger zone
        danger_enter_time_ms = 0;
        danger_triggered = false;
    }

    /* ================= Buzzer loop control ================= */

    // If distance >= 60cm, stop buzzer loop
    if (new_interval == 0) {
        if (buzzer_loop_active) {
            buzzer_stop();
            buzz_state = BUZZ_IDLE;
            buzzer_loop_active = false;
        }
        return;
    }

    // Start buzzer loop if entering <60cm zone
    if (!buzzer_loop_active) {
        buzzer_loop_active = true;
        buzz_interval_ms = new_interval;
        buzz_state = BUZZ_IDLE;
        next_beep_time = system_time_ms;
        return;
    }

    // Update interval if distance zone changed
    if (new_interval != buzz_interval_ms) {
        buzz_interval_ms = new_interval;
    }

    // State machine for beep pattern
    switch (buzz_state) {

        case BUZZ_IDLE:
            if (system_time_ms >= next_beep_time) {
                buzzer_start(2000);
                buzz_state = BUZZ_ON;
                next_beep_time = system_time_ms + 100;
            }
            break;

        case BUZZ_ON:
            if (system_time_ms >= next_beep_time) {
                buzzer_stop();
                buzz_state = BUZZ_OFF;
                next_beep_time = system_time_ms + buzz_interval_ms;
            }
            break;

        case BUZZ_OFF:
            if (system_time_ms >= next_beep_time) {
                buzzer_start(2000);
                buzz_state = BUZZ_ON;
                next_beep_time = system_time_ms + 100;
            }
            break;
    }
}



// RD3 echo, RD2 trig
float ultrasonic_read_cm() {

    // Send TRIG pulse
    LATDbits.LD2 = 1;
    __delay_us(10);
    LATDbits.LD2 = 0;

    // Wait for ECHO to go high
    while (!PORTDbits.RD3);   

    unsigned int us_count = 0;

    // Measure ECHO high time in microseconds
    while (PORTDbits.RD3) {
        __delay_us(1);
        us_count++;

        if (us_count > 30000) break; // safety break to avoid infinite loop
    }

    // Convert echo time to distance (cm)
    return (float)us_count*20.0 / 58.0;
}

// Read the first character from UART buffer immediately (one-char command)
char UART_ReadCharImmediate() {
    if (buffer_size > 0) {
        char c = buffer[0];
        ClearBuffer();
        return c;
    }
    return 0;
}

// ================== TM1637 4-DIGIT 7-SEG ==================
static void tm_delay(void) {
    __delay_us(5);
}










static void tm_start(void) {
    TM_DIO_TRIS = 0;
    TM_CLK_TRIS = 0;
    TM_DIO = 1; TM_CLK = 1;
    tm_delay();
    TM_DIO = 0;
    tm_delay();
}

static void tm_stop(void) {
    TM_CLK = 0;
    tm_delay();
    TM_DIO = 0;
    tm_delay();
    TM_CLK = 1;
    tm_delay();
    TM_DIO = 1;
    tm_delay();
}

static void tm_write_byte(unsigned char b) {
    for (int i = 0; i < 8; i++) {
        TM_CLK = 0;
        tm_delay();
        TM_DIO = (b & 0x01) ? 1 : 0;
        tm_delay();
        TM_CLK = 1;
        tm_delay();
        b >>= 1;
    }

    // ACK (ignored, but perform the cycle)
    TM_CLK = 0;
    TM_DIO_TRIS = 1; // input
    tm_delay();
    TM_CLK = 1;
    tm_delay();
    TM_CLK = 0;
    TM_DIO_TRIS = 0; // output
}



static void tm_forceInit(void) {
    tm_start();
    tm_write_byte(0x40);
    tm_stop();

    tm_start();
    tm_write_byte(0x88 | 0x07);
    tm_stop();
}




static const unsigned char seg_map[10] = {
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8
    0x6f  // 9
};

// Low-level raw 4-digit writer
static void tm_displayRaw4(unsigned char s0, unsigned char s1, unsigned char s2, unsigned char s3) {
    tm_start();
    tm_write_byte(0x40); // auto increment
    tm_stop();

    tm_start();
    tm_write_byte(0xC0); // address 0
    tm_write_byte(s0);
    tm_write_byte(s1);
    tm_write_byte(s2);
    tm_write_byte(s3);
    tm_stop();

    tm_start();
    tm_write_byte(0x88 | 0x07); // display ON, brightness max
    tm_stop();
}

// Show "----"
static void tm_showDash(void) {
    tm_displayRaw4(0x40, 0x40, 0x40, 0x40);
}

// Clear display (show nothing)
static void tm_showBlank(void) {
    tm_displayRaw4(0x00, 0x00, 0x00, 0x00);
}

// Show distance in cm: dist10 = cm*10 (e.g. 123 -> "12.3")
static void tm_showDistance_cm_x10(unsigned int dist10) {
    if (dist10 > 9999) dist10 = 9999;

    unsigned int d0 = (dist10 / 1000) % 10;
    unsigned int d1 = (dist10 / 100) % 10;
    unsigned int d2 = (dist10 / 10) % 10;
    unsigned int d3 = dist10 % 10;

    // decimal point on the second digit from left (not enabled here)
    unsigned char s0 = (d0 == 0) ? 0x00 : seg_map[d0];
    unsigned char s1 = (d0 == 0 && d1 == 0) ? 0x00 : (seg_map[d1] ); // dot could be added here if needed
    unsigned char s2 = seg_map[d2];
    unsigned char s3 = seg_map[d3];

    tm_displayRaw4(s0, s1, s2, s3);
}

// Update display only when changed; briefly disable interrupts for stable bit-bang
static void tm_updateDistance_ifChanged(unsigned int dist10, bool valid) {
    tm_forceInit();
    static unsigned int last_dist10 = 0xFFFF;
    static unsigned int last_state  = 0xFFFF; // 0 invalid, 1 valid

    unsigned int state = valid ? 1 : 0;

    if (!valid) {
        if (last_state != 0) {
            unsigned char gh = INTCONbits.GIEH, gl = INTCONbits.GIEL;
            INTCONbits.GIEH = 0; INTCONbits.GIEL = 0;
            tm_showDash();
            INTCONbits.GIEH = gh; INTCONbits.GIEL = gl;
            last_state = 0;
        }
        return;
    }

    if (dist10 != last_dist10 || last_state != 1) {
        unsigned char gh = INTCONbits.GIEH, gl = INTCONbits.GIEL;
        INTCONbits.GIEH = 0; INTCONbits.GIEL = 0;
        tm_showDistance_cm_x10(dist10);
        INTCONbits.GIEH = gh; INTCONbits.GIEL = gl;

        last_dist10 = dist10;
        last_state = 1;
    }
}




// ================== MAIN ==================
void main() {
    __delay_ms(500);   // ? ???????
    Initialize();
    motor_stop();

    printf("\n=== UART Motor Control (High Priority UART) ===\n");
    printf("Commands: F B L R S\n\n");

    // Local variables for periodic ultrasonic update / display refresh
    unsigned long last_ultra_update_ms = 0;
    float last_distance_cm = 0;
    set_servo_angle(0);

    while (1) {

        /* ================= UART command processing ================= */
        char c = UART_ReadCharImmediate();
        if (c != 0) {
            char temp[2] = {c, '\0'};
            keyboard_input(temp);
        }

        /* ================= Movement timeout (auto stop) ================= */
        if (current_move != MOVE_NONE) {
            if(turn == 0){
                if ((system_time_ms - last_move_time_ms) >= 300) {
                motor_stop();
                current_move = MOVE_NONE;
                printf("[AUTO] Movement timeout -> STOP\n");
                }
            }
            else{
                if ((system_time_ms - last_move_time_ms) >= 100) {
                motor_stop();
                current_move = MOVE_NONE;
                printf("[AUTO] Movement timeout -> STOP\n");
                }
            }
        }

        /* ================= Laser auto-off ================= */
        if (laser_on_until != 0 && system_time_ms >= laser_on_until) {
            LASER = 0;
            laser_on_until = 0;
            printf("[AUTO] Laser OFF\n");
        }

        /* ================= Ultrasonic + buzzer + 7-seg update every 100ms ================= */
        if (system_time_ms - last_ultra_update_ms >= 100) {

            last_ultra_update_ms = system_time_ms;

            // Read distance
            last_distance_cm = ultrasonic_read_cm();
            
            // add,if distance too short
            
            if(last_distance_cm< 10){
                motor_stop();
                stop_front= 1;
            }
            else {
                stop_front= 0;
            }
            // Buzzer loop behavior based on distance
            buzzer_by_distance(last_distance_cm);
            
            // Update 7-seg display
            if (last_distance_cm < 0) {
                tm_updateDistance_ifChanged(0, false);
            }
            else if (last_distance_cm > 600.0f) {
                unsigned char gh = INTCONbits.GIEH, gl = INTCONbits.GIEL;
                INTCONbits.GIEH = 0; INTCONbits.GIEL = 0;
                tm_showBlank();
                INTCONbits.GIEH = gh; INTCONbits.GIEL = gl;
            }
            else {
                unsigned int dist = (unsigned int)last_distance_cm;
                if (dist > 9999) dist = 9999;
                tm_updateDistance_ifChanged(dist, true);
            }
        }

        /* ================= System tick (10ms) ================= */
        __delay_ms(10);
        system_time_ms += 10;
    }
}



