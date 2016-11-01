// this is the header file that tells the compiler what pins and ports, etc.
// are available on this chip.
#include <avr/io.h>
#include <util/delay.h>

#include <avr/interrupt.h>

// define what pins the LEDs are connected to.
// in reality, PD6 is really just '6'
#define LED PA0

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

ISR (TIMER1_COMPA_vect)
{
	// Toggle LED
	// Note: Writing to PINx toggles when the pin is an output
	PINA |= (1 << LED);
}

void timer1_init()
{
// REGISTERS //
  
  // OCR1A - Output Compare Register 1 A - Used to set TOP value for Counter 1
  // TIMSK - Timer Interrupt Mask Register - Used to enable interrupts for counter  
  // TCCR1A - Timer/Counter Control Register 1 A - Used to configure Timer/Counter 1
  // TCCR1B - Timer/Counter Control Register 1 B - Used to configure Timer/Counter 1

  // TCCR1A - COM1A0 - Compare Output Mode (Toggle on match)
  //				 NOTE: Depends on Waveform Generation Mode
  
  // TCCR1B - WGM13:0 = 4 (TOP = OCR1A) or 12 (TOP = ICR1) Ref Table 46
  // TCCR1B - CS12:0 Timer prescaling 
  //	      NOTE: Must set some bits otherwise the timer has no clock source
  
  TCCR1B |= (1 << CS12); //F_CPU/1024
  
  TCCR1B |= (1 << WGM12); //Enable CTC mode with OCR1A as TOP
  
  TCCR1A |= (1 << COM1A0); //Toggle OC1A on Compare Match.

  TCNT1 = 0;
  OCR1A = 4100; //Trigger every second
  
  TIMSK |= (1 << OCIE1A); //Enable Interrupt for Channel A of Timer/Counter 1
  sei(); //Enable Interrupts Globally
}

int main(void) {
  // initialize the direction of PORTD #6 to be an output
  set_output(DDRA, LED);
  set_output(DDRB, PB3);

  timer1_init();
  
  while (1) 
  {
  }
}