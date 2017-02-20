//Panasonic TV volume down button spoof

#ifndef F_CPU
#define F_CPU 12000000UL // or whatever may be your frequency
#endif

#include <avr/io.h>

#include <string.h>


/*
===start===
on for 3600us
off for 1200 us

=== the data===
state 0 = (720 - 880)us, low for 480us, high for 240us (440)
state 1 = (1680-1760)us, low for 1200us, high for (480 to 560)us

0,0,1,0(x11),1, 0(x9), 1, 0(x8), 1, 0(x4), 1, 0 (x2), 1, 0(x4), 1, 0, 1, 0

=== end / between frames ===
off for 73ms
*/


// the codes for the remote buttons
const uint_fast64_t volDown  = 0x0001424201004004;
const uint_fast64_t volUp    = 0x0001404001004004;
const uint_fast64_t Power    = 0x00017A7A01004004;
const uint_fast64_t AV       = 0x00010A0A01004004;
#define CODE_LENGTH_BITS 49

// the GPIO pin connected to the LED
#define IR_PIN (1<<4)|(1<<5)

#define LED_PIN (1<<0)


// microsecond delay
void delay_us(unsigned int time)
{
    while (time--)
    {
        //_delay_us(1);

        /* avr's delay function is kind of fucked.
         so i'll make my own.
         at 12MHz clock, 1 clock happens every 83ns.
         we need 12 clocks to take 1 us.
         a few of the clocks are taken by the comparison of values (x2), int decrements (x2) and the while loop (a branch or jump)
         */

         __asm__ ("nop\n\t"
                  "nop\n\t"
                  "nop\n\t"
                  "nop\n\t"
                  "nop\n\t"
                  "nop\n\t"
                  "nop\n\t");
    }
}

// millisecond delay
void delay_ms(unsigned int time)
{
    while (time--)
    {
        delay_us(1000);
    }
}


// Enable the PWM
void enablePWM(void)
{
    // this only enables. Setup is in main.
    TCCR1B |= (1 << CS10);
    TCCR1A |= (1 << COM1A1);

    PORTB |= LED_PIN;
}

// Disable the PWM (when not a high bit)
void disablePWM(void)
{
    //disable
    TCCR1B &= ~(1 << CS10);
    TCCR1A &= ~(1 << COM1A1);

    //set the port to 0
    PORTD &= ~IR_PIN; //PD5 is connected to the timer1 out
    PORTB &= ~LED_PIN;
}

// UART send
void USART_Transmit(unsigned char data)
{
    /* Wait for empty transmit buffer */
    while ( !( UCSRA & (1<<UDRE)) );

    /* Put data into buffer, sends the data */
    UDR = data;
}

unsigned char USART_Receive(void)
{
    /* Wait for data to be received */
    while ( !(UCSRA & (1<<RXC)) );

    /* Get and return received data from buffer */
    return UDR;
}


// usart params
#define UART_BAUD 115200
// calculate the baud rate and split into high and low registers (8 + 4 bit)
#define BAUDRATE_LOW ((F_CPU)/(UART_BAUD*16UL)-1)
#define BAUDRATE_HIGH (BAUDRATE_LOW>>8)


// Panasonic Infra Red parameters

// frequency in Hz of the carrier, usually 36-40 kHz
#define IR_CARRIER_FREQ 37920

#define IR_DUTY_CYCLE 0.3f

// timing of pulses in microseconds
#define PREAMBLE_LONG 3600
#define PREAMBLE_SHORT 1200

#define BIT_LONG 1200
#define BIT_SHORT 440

// minimum time between frames in milliseconds
#define DELAY_BETWEEN_FRAMES 73

// encode and send the code
void IR_send(uint_fast64_t code)
{
    uint_fast8_t length = CODE_LENGTH_BITS + 1;
    // preamble
    enablePWM();
    delay_us(PREAMBLE_LONG);
    disablePWM();
    delay_us(PREAMBLE_SHORT);

    // encode the data
    uint_fast8_t bit = 0;

    // iterate through each value and modulate it onto the 38 kHz square wave
    for (uint_fast8_t i = 0; i < length; i++)
    {
        bit = (uint8_t)(code >> i);

        if (bit & 0x01)
        {
            // the bit is set, encode a symbol 1
            disablePWM();
            delay_us(BIT_LONG);
            enablePWM();
            delay_us(BIT_SHORT);
        }
        else
        {
            // bit is not set encode a symbol 0
            disablePWM();
            delay_us(BIT_SHORT);
            enablePWM();
            delay_us(BIT_SHORT);
        }
    }

    disablePWM();
    delay_ms(DELAY_BETWEEN_FRAMES);
}

void print(const char msg[])
{
    for (int i = 0; i < strlen(msg); i++)
    {
        USART_Transmit(msg[i]);
    }
}


int main(void)
{
    // enable pins
    DDRB = 0x0001; //initialize portB as output
    //DDRA = IR_PIN; //portA0, is the IR led, output

    DDRD = IR_PIN ;


    /*
    PWM initialization
    make a 38KHz PWM with defined duty cycle
    */

    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    ICR1 = ((F_CPU / IR_CARRIER_FREQ) - 1); // TOP
    OCR1A = (uint_fast16_t)(IR_DUTY_CYCLE*(F_CPU / IR_CARRIER_FREQ) - 1); //duty cycle

    disablePWM();

    //TOP
    /*Fpwm = F_clk / N(1+TOP)
    N = Prescaler
    (1+TOP) = 316 to make frequency 37974 Hz */

    //set up the usart
    /* Set frame format: 8data, 1 stop bit  */
    UCSRC = (1<<URSEL)|(3<<UCSZ0);

    //set the baud rate
    UBRRL = BAUDRATE_LOW;
    UBRRH = BAUDRATE_HIGH;

    //enable tx and rx
    UCSRB = (1<<RXEN)|(1<<TXEN);

    unsigned char command = 0;

    while (1)
    {
        command = USART_Receive();

        switch (command)
        {
        case 'U':
            // volume up
            IR_send(volUp);
            print("Ok\r\n");
            break;
        case 'D':
            // volume down
            IR_send(volDown);
            print("Ok\r\n");
            break;
        case 'P':
        // power button
            IR_send(Power);
            print("Ok\r\n");
            break;
        case 'A':
            //AV button
            IR_send(AV);
            print("Ok\r\n");
            break;
        default:
            print("No\r\n");
            break;
        }
        command = 0;
    }

}
