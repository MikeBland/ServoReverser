/****************************************************************************
*  Copyright (c) 2019 by Michael Blandford. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*
****************************************************************************/


// Input on PB0 = IO8
// Reversed output on PB1 = IO9
// Slow output on PB2 = IO10
// Switch output on PD2 = IO2

// This defines the time, in seconds, for the slow output
// to travel from one end to the other

#define SLOW_RATE						3		// End to end in seconds

// Uncomment the following line to reverse the slow output
//#define	REVERSE_SLOW_OUTPUT		1

// This is a default time used at startup
// The actual time measured between input pulses is then used
#define PULSE_PERIOD				20	// Time between servo pulses in mS

// Switch output setting

// Set this to 1 for the out ON state to be high
#define SWITCH_ON_LEVEL	1

// Set this to the pulse width (in uS) above which the switch output is ON
#define SWITCH_POINT		1600

// Set the gap to allow some hysteresis
// Output will switch at the switch point + GAP when pulse increasing
// Output will switch at the switch point - GAP when pulse decreasing
#define SWITCH_GAP			25


// Set some clock speed relative values
#if F_CPU == 20000000L   // 20MHz clock 
   #error Unsupported clock speed
#elif F_CPU == 16000000L  // 16MHz clock                                                  
#define CENTRE_PULSE_TIME		(1500 * 16)
#define SLOW_ADJUST_SCALE		16
#elif F_CPU == 8000000L   // 8MHz clock
#define CENTRE_PULSE_TIME		(1500 * 8)
#define SLOW_ADJUST_SCALE		8
#else
    #error Unsupported clock speed
#endif

#define SLOW_ADJUST					( (SLOW_ADJUST_SCALE * PULSE_PERIOD) / SLOW_RATE )

#if (SWITCH_GAP < 0)
#undef SWITCH_GAP
#define SWITCH_GAP			25
#endif

#define SWITCH_TRIGGER_UP	((SWITCH_POINT+SWITCH_GAP) * SLOW_ADJUST_SCALE)
#define SWITCH_TRIGGER_DOWN	((SWITCH_POINT-SWITCH_GAP) * SLOW_ADJUST_SCALE)


// Input capture on PB0 = IO8
// Output compare A on PB1 = IO9
// Output compare B on PB2 = IO10

#define ENABLE_TIMERA_INTERRUPT()       ( TIMSK1 |= ( 1<< OCIE1A ) )
#define DISABLE_TIMERA_INTERRUPT()      ( TIMSK1 &= ~( 1<< OCIE1A ) )
#define CLEAR_TIMERA_INTERRUPT()        ( TIFR1 = (1 << OCF1A) )
#define ENABLE_TIMERB_INTERRUPT()       ( TIMSK1 |= ( 1<< OCIE1B ) )
#define DISABLE_TIMERB_INTERRUPT()      ( TIMSK1 &= ~( 1<< OCIE1B ) )
#define CLEAR_TIMERB_INTERRUPT()        ( TIFR1 = (1 << OCF1B) )


uint16_t ServoPulse ;
uint16_t LastSlowServoPulse ;

uint16_t PulsePerioduS ;
uint32_t RisingTime ;

uint16_t SlowAdjust ;
uint8_t Counter ;

void setup()
{
}

void init()
{
	// Pin directions
	PORTB &= ~0x06 ;		// Outputs low
	DDRB |= 0x06 ; 			// Set pins as output
	DDRB &= ~0x01 ;			// Set pin as input

// Switch output default to low	
	PORTD &= ~0x04 ;
	DDRD |= 0x04 ;
// LED output a copy of Switch output
	PORTB &= ~0x20 ;
	DDRB|= 0x20 ;

  // Timer1
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1
	DISABLE_TIMERA_INTERRUPT() ;
	CLEAR_TIMERA_INTERRUPT() ;
	DISABLE_TIMERB_INTERRUPT() ;
	CLEAR_TIMERB_INTERRUPT() ;
	
	SlowAdjust = SLOW_ADJUST ;		// Default value
	LastSlowServoPulse = 1500 * SLOW_ADJUST_SCALE ;		// Default value
	
	sei() ;
}

// Interrupt for reversed output
ISR(TIMER1_COMPA_vect)
{
	OCR1A += CENTRE_PULSE_TIME * 2 - ServoPulse ;	// Set required pulse time
	TCCR1A &= ~0x40 ;								// Clear output on match
	DISABLE_TIMERA_INTERRUPT( ) ;		// We have finished the pulse
}

ISR(TIMER1_COMPB_vect)
{
	uint16_t pulseTime ;
	if ( Counter < 8 )			// At startup, ignore slow
	{
		pulseTime = ServoPulse ;			
	}
	else
	{
		if ( ServoPulse > LastSlowServoPulse )
		{
			pulseTime = LastSlowServoPulse + SlowAdjust ;		// new slowed value
			if ( pulseTime > ServoPulse )			// Check for passed required value
			{
				pulseTime = ServoPulse ;			
			}
		}
		else
		{
			pulseTime = LastSlowServoPulse - SlowAdjust ;		// new slowed value
			if ( pulseTime < ServoPulse )			// Check for passed required value
			{
				pulseTime = ServoPulse ;			
			}
		}
	}
#ifdef REVERSE_SLOW_OUTPUT
	pulseTime = CENTRE_PULSE_TIME * 2 - pulseTime ;	// Reverse if required
#endif
	OCR1B += pulseTime ;							// Set the timer output
	TCCR1A &= ~0x10 ;									// Clear output on match
	DISABLE_TIMERB_INTERRUPT( ) ;			// We have finished the pulse
	LastSlowServoPulse = pulseTime ;	// Remember where we are
}

//capture input in 16MHz
void capture()
{
  static uint16_t lastCapt ;
  uint8_t value ;
	  
	cli() ;								// Protect 16-bit register read
  uint16_t capture = ICR1 ;		// Note when edge occured
  sei() ;

	value = TCCR1B ^ 0x40 ;		// Change next edge
	TCCR1B = value ;
	if ( value & 0x40 )		// Next is rising edge
	{
  	ServoPulse = (capture - lastCapt) ;		// Get pulse width
		cli() ;             				// Protect 16-bit register access
		OCR1B = OCR1A = TCNT1 + 200 ;		// Start output pulses "soon"
		TCCR1A = 0xF0 ;									// Set outputs on match
		CLEAR_TIMERA_INTERRUPT() ;
		ENABLE_TIMERA_INTERRUPT() ;
		CLEAR_TIMERB_INTERRUPT() ;
		ENABLE_TIMERB_INTERRUPT() ;
  	sei() ;
		// Now set the switch output
		if ( ServoPulse > SWITCH_TRIGGER_UP )
		{
#if SWITCH_ON_LEVEL
			PORTD |= 0x04 ;
			PORTB |= 0x20 ;
#else
			PORTD &= ~0x04 ;
			PORTB &= ~0x20 ;
#endif
		}
		if ( ServoPulse < SWITCH_TRIGGER_DOWN )
		{
#if SWITCH_ON_LEVEL
			PORTD &= ~0x04 ;
			PORTB &= ~0x20 ;
#else
			PORTD |= 0x04 ;
			PORTB |= 0x20 ;
#endif
		}
	}
	else
	{
		// Found rising edge - note time
		uint32_t now = micros() ;
		PulsePerioduS = ( now - RisingTime ) ;
		RisingTime = now ;		
	}
  lastCapt = capture ;		// Remember when last edge occured
}


// replacement millis() and micros()
// These work polled, no interrupts
// micros() MUST be called at least once every 4 milliseconds at 16MHZ, 8mS at 8MHz
uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;

uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	
#if F_CPU == 16000000L  // 16MHz clock                                                  
	elapsed >>= 4 ;
#elif F_CPU == 8000000L   // 8MHz clock
	elapsed >>= 3 ;
#endif

	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd = 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd = 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd = 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}

void loop()
{
	for(;;)
	{
		if ( TIFR1 & (1 << ICF1) )		// Capture edge occured
		{
			capture() ;									// Process the capture
			TIFR1 = (1 << ICF1) ;				// Clear flag by writing a 1 to it
			if ( ++Counter > 20 )
			{
				Counter = 8 ;
				// Use the /8 below to avoid 16-bit overflow
				SlowAdjust = SLOW_ADJUST_SCALE * (PulsePerioduS / 8) / SLOW_RATE / (1000/8) ;
			}
		}
		millis() ;										// Keep calling this
	}
}

