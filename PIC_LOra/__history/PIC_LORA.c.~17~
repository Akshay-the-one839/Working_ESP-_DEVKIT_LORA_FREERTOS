#include <18F25K22.h>
#device ADC=10
#INCLUDE <stdlib.h> 

#fuses HSM,PUT,NOPROTECT,NOLVP,NOWDT,NOBROWNOUT,NOPLLEN,NOPBADEN
#fuses IESO
#use delay(xtal=20MHz, clock=20MHz)
#byte clock_source=getenv("clock")

#use rs232(stream=COM_1,baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)
#include "internal_eeprom.c"

#define V1      PIN_B0
#define V2      PIN_B1
#define V3      PIN_B2
#define V4      PIN_B3
#define V5      PIN_B4
#define V6      PIN_B5
#define V7      PIN_B6
#define V8      PIN_B7
#define SIGLED  PIN_C3

// ============================================================
// SET THIS CHIP'S ID
// Slave A : #define MY_ID  'A'
// Slave B : #define MY_ID  'B'
// Slave C : #define MY_ID  'C'
// ============================================================
#define MY_ID  'A'

int i=0;
int j=0;
int k=0;
char ch;
char a[4];
char b[3];
char c[2];  
char state1='0'; 
char state2='0';
char state3='0';
char state4='0';
char state5='0'; 
char state6='0';
char state7='0';
char state8='0';

int1 flgg=0;
int1 flg=0;

#byte RCSTA = 0xFAB
#bit OERR = RCSTA.1
#bit FERR = RCSTA.2
#bit CREN = RCSTA.4

char bw[2];

int1 RX_Command_Ready = 0;     

char RxBuffer[50];
int8 Index = 0;

char TimeBuffer[9];

char vno1, vno2, vno3, vno4, vno5, vno6, vno7, vno8;

char star;
char timer;
int1 vanoflg1=0;
int1 vanoflg2=0;
int1 vanoflg3=0;
int1 vanoflg4=0;
int1 vanoflg5=0;
int1 vanoflg6=0;
int1 vanoflg7=0;
int1 vanoflg8=0;

int1 txflg=0;
int1 txflgg=0;

int8 overflow_count = 0;
int8 sec=0;
int8 min=0;
int8 hr=0;
int8 bwmin=0;
int8 bwminv1=0;
int8 bwminv2=0;

unsigned int16 eeprom_base_address = 0;

// -------------------------------------------------------
// Timer1 — unchanged
// -------------------------------------------------------
void setup_timer1_for_1s_interrupt() {
    setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
    set_timer1(34286);
    enable_interrupts(INT_TIMER1);
}

#INT_TIMER1
void timer1_isr() {
    overflow_count++;
    set_timer1(34286);

    if (overflow_count >= 20) {
        sec++;
        overflow_count = 0;

        if(sec >= 59) {
            sec = 0;
            min++;
            bwminv1++;
            if(bwminv1 >= 11) bwminv1 = 0;
            bwminv2++;
            if(bwminv2 >= 11) bwminv2 = 0;

            write_eeprom(eeprom_base_address + 40, hr);
            write_eeprom(eeprom_base_address + 41, min);

            if(min >= 59) { min = 0; hr++; }
            if(hr >= 24)    hr = 0;
        }
    }
}

void store_byte(unsigned int16 address, unsigned char value) {
    if (address < 256) write_eeprom(address, value);
}

unsigned char retrieve_byte(unsigned int16 address) {
    if (address < 256) return read_eeprom(address);
    return 0;
}

// -------------------------------------------------------
// ISR — filters MY_ID only
// -------------------------------------------------------
#int_RDA
void RDA_isr(void)
{
    if (oerr) { CREN = 0; CREN = 1; return; }
    if (ferr) { getc(COM_1); return; }

    char c = getc(COM_1);
    if (c != '$') return;

    char m = getc(COM_1);
    if (m != 'M') return;

    getc(COM_1);   // discard '*' after M

    char sid = getc(COM_1);
    if (sid != MY_ID) return;   // not my packet — ignore

    getc(COM_1);   // discard '*' after SlaveID

    int8 i = 0;
    char x;
    while (i < 49)
    {
        x = getc(COM_1);
        if (x == '#') break;
        RxBuffer[i] = x;
        i++;
    }
    RxBuffer[i] = '\0';

    RX_Command_Ready = 1;
}

// -------------------------------------------------------
// txvalve — sends acknowledgement back to app
// App waits for this reply to turn the button WHITE
// -------------------------------------------------------
void txvalve()
{
    for(int16 i2 = 0; i2 < 5; i2++)
    {
        if(txflg == 1)
        {
            output_high(SIGLED);
            printf("$");
            printf("%c", MY_ID);   // correct slave ID
            printf(",");
            printf("1"); printf(","); printf("%c", state1); printf(",");
            printf("2"); printf(","); printf("%c", state2); printf(",");
            printf("3"); printf(","); printf("%c", state3); printf(",");
            printf("4"); printf(","); printf("%c", state4); printf(",");
            printf("5"); printf(","); printf("%c", state5); printf(",");
            printf("6"); printf(","); printf("%c", state6); printf(",");
            printf("7"); printf(","); printf("%c", state7); printf(",");
            printf("8"); printf(","); printf("%c", state8); printf(",");
        }
        delay_ms(200);
    }
    txflg = 0;
    output_low(SIGLED);
}

// -------------------------------------------------------
// parse_packet
// -------------------------------------------------------
void parse_packet(void)
{
    vno1   = RxBuffer[0];
    state1 = RxBuffer[2];

    TimeBuffer[0] = RxBuffer[6];
    TimeBuffer[1] = RxBuffer[7];
    TimeBuffer[2] = RxBuffer[8];
    TimeBuffer[3] = RxBuffer[9];
    TimeBuffer[4] = RxBuffer[10];
    TimeBuffer[5] = RxBuffer[11];
    TimeBuffer[6] = RxBuffer[12];
    TimeBuffer[7] = RxBuffer[13];
    TimeBuffer[8] = '\0';

    vno2 = RxBuffer[15]; state2 = RxBuffer[17];
    vno3 = RxBuffer[19]; state3 = RxBuffer[21];
    vno4 = RxBuffer[23]; state4 = RxBuffer[25];
    vno5 = RxBuffer[27]; state5 = RxBuffer[29];
    vno6 = RxBuffer[31]; state6 = RxBuffer[33];
    vno7 = RxBuffer[35]; state7 = RxBuffer[37];
    vno8 = RxBuffer[39]; state8 = RxBuffer[41];

    printf("\r\n=== Parsed Packet ===\r\n");
    printf("SlaveID: %c\r\n", MY_ID);
    printf("Time   : %s\r\n", TimeBuffer);
    printf("V1: %c  State1: %c\r\n", vno1, state1);
    printf("V2: %c  State2: %c\r\n", vno2, state2);
    printf("V3: %c  State3: %c\r\n", vno3, state3);
    printf("V4: %c  State4: %c\r\n", vno4, state4);
    printf("V5: %c  State5: %c\r\n", vno5, state5);
    printf("V6: %c  State6: %c\r\n", vno6, state6);
    printf("V7: %c  State7: %c\r\n", vno7, state7);
    printf("V8: %c  State8: %c\r\n", vno8, state8);
    printf("=====================\r\n");
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
void main() {
    output_b(0x00);  
    set_tris_a(0xff); 
    SET_TRIS_b(0xcf);
    SET_TRIS_c(0xb8);  

    setup_timer1_for_1s_interrupt();
    setup_timer_1(T1_DISABLED); 

    enable_interrupts(INT_RDA);  
    enable_interrupts(GLOBAL);

    output_low(V1);
    output_low(V2);
    output_low(V3);
    output_low(V4);
    output_low(V5);
    output_low(V6);
    output_low(V7);
    output_low(V8);
    output_low(SIGLED);

    while(true)
    {
        if(RX_Command_Ready == 1)
        {
            RX_Command_Ready = 0;
            parse_packet();

            if(vno1 == '1' && vanoflg1 == 0)
                if(state1 == '1') { output_high(V1); vanoflg1 = 1; delay_ms(2000); }
            if(vno1 == '1' && vanoflg1 == 1)
                if(state1 == '0') { output_low(V1); vanoflg1 = 0; }

            if(vno2 == '2' && vanoflg2 == 0)
                if(state2 == '1') { output_high(V2); vanoflg2 = 1;delay_ms(2000);  }
            if(vno2 == '2' && vanoflg2 == 1)
                if(state2 == '0') { output_low(V2); vanoflg2 = 0; }

            if(vno3 == '3' && vanoflg3 == 0)
                if(state3 == '1') { output_high(V3); vanoflg3 = 1; delay_ms(2000); }
            if(vno3 == '3' && vanoflg3 == 1)
                if(state3 == '0') { output_low(V3); vanoflg3 = 0; }

            if(vno4 == '4' && vanoflg4 == 0)
                if(state4 == '1') { output_high(V4); vanoflg4 = 1; delay_ms(2000); }
            if(vno4 == '4' && vanoflg4 == 1)
                if(state4 == '0') { output_low(V4); vanoflg4 = 0; }

            if(vno5 == '5' && vanoflg5 == 0)
                if(state5 == '1') { output_high(V5); vanoflg5 = 1; delay_ms(2000); }
            if(vno5 == '5' && vanoflg5 == 1)
                if(state5 == '0') { output_low(V5); vanoflg5 = 0; }

            if(vno6 == '6' && vanoflg6 == 0)
                if(state6 == '1') { output_high(V6); vanoflg6 = 1; delay_ms(2000); }
            if(vno6 == '6' && vanoflg6 == 1)
                if(state6 == '0') { output_low(V6); vanoflg6 = 0; }

            if(vno7 == '7' && vanoflg7 == 0)
                if(state7 == '1') { output_high(V7); vanoflg7 = 1; delay_ms(2000); }
            if(vno7 == '7' && vanoflg7 == 1)
                if(state7 == '0') { output_low(V7); vanoflg7 = 0; }

            if(vno8 == '8' && vanoflg8 == 0)
                if(state8 == '1') { output_high(V8); vanoflg8 = 1; delay_ms(2000); }
            if(vno8 == '8' && vanoflg8 == 1)
                if(state8 == '0') { output_low(V8); vanoflg8 = 0; }

            // <<<< THIS IS THE FIX >>>>
            // After processing the packet and driving the GPIO,
            // set txflg=1 so txvalve() sends the reply back.
            // The app is waiting for this reply to confirm the
            // valve state and turn the button WHITE.
            // Without this line, txvalve() is NEVER called.
            txflg = 1;

        } // rx flag end

        output_low(SIGLED);

        if(txflg == 1)
        {
            txvalve();
        }

        delay_ms(1000);

    } // while
} // main
