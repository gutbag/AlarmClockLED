#include <avr/sleep.h>
#include <avr/interrupt.h>

static const int ALARM_PIN = 2;
static const int BUTTON_PIN = 3;
static const int LED_PIN = 4;
static const int DBG_P1 = 1;

enum StateType
{
	SLEEPING,
	ALARM,
	LED_TEST,
	SLEEP_PENDING
};

enum EventType // 'Event' won't compile
{
	NONE,
	ALARM_ON,
	ALARM_OFF,
	BUTTON_PRESSED
};

class DebouncedInput
{
public:
	DebouncedInput(const int aPin, const unsigned long aStableTimeMs)
	: pin(aPin), stableTimeMs(aStableTimeMs), stableSinceMs(~0), lastRawPinState(0), state(HIGH)
	{
	}
	
	void setup()
	{
		pinMode(pin, INPUT_PULLUP);
		reset();
	}
	
	void loop(unsigned long msNow)
	{
		int newPinState = digitalRead(pin);
		
		if (newPinState != lastRawPinState)
		{
			stableSinceMs = msNow;
		}
		else if (msNow - stableSinceMs >= stableTimeMs)
		{
			state = newPinState;
		}
		
		lastRawPinState = newPinState;
	}
	
	void reset()
	{
		lastRawPinState = digitalRead(pin);
		state = lastRawPinState;
		stableSinceMs = millis();
	}
	
	int getState() const
	{
		return state;
	}
	
private:
	int pin;
	unsigned long stableTimeMs;
	unsigned long stableSinceMs;
	int lastRawPinState;
	int state;
};

class Button : public DebouncedInput
{
public:
	Button(int aPin) : DebouncedInput(aPin, 50), pressedState(false)
	{
	}
	
	void loop(unsigned long msNow)
	{
		int oldState = getState();
		
		DebouncedInput::loop(msNow);
		
		int newState = getState();
		
		if (oldState != newState)
		{
			pressedState = newState == HIGH;
		}
	}
	
	void reset()
	{
		DebouncedInput::reset();
		pressedState = false;
	}
	
	boolean pressed()
	{
		boolean currentPressedState = pressedState;
		pressedState = false;
		return currentPressedState;
	}
	
private:
	boolean pressedState;
};

class Alarm : public DebouncedInput
{
public:
	Alarm(int aPin) : DebouncedInput(aPin, 2000), offState(false)
	{
	}
	
	void loop(unsigned long msNow)
	{
		int oldState = getState();
		
		DebouncedInput::loop(msNow);
		
		int newState = getState();
		
		if (oldState != newState)
		{
			offState = newState == HIGH;
		}
	}
	
	void reset()
	{
		DebouncedInput::reset();
		offState = false;
	}
	
	boolean off()
	{
		boolean currentOffState = offState;
		offState = false;
		return currentOffState;
	}
	
private:
	boolean offState;
};


StateType state = SLEEPING;
EventType event = NONE;
unsigned long ledTestOnTime = 0;
Button button(BUTTON_PIN);
Alarm alarm(ALARM_PIN);

void setup()
{
	//  pinMode(ALARM_PIN, INPUT_PULLUP);
	//  digitalWrite(ALARM_PIN, HIGH);
	pinMode(LED_PIN, OUTPUT);
	
	pinMode(DBG_P1, OUTPUT);
	digitalWrite(DBG_P1, LOW);
	
	button.setup();
	alarm.setup();
	
	// Flash quick sequence so we know setup has started
	for (int k = 0; k < 10; k++)
	{
		if (k % 2 == 0)
			digitalWrite(LED_PIN, HIGH);
		else
			digitalWrite(LED_PIN, LOW);
		
		delay(100);
	}
	
	state = SLEEPING;
	sleep();
}

void sleep()
{
	pinMode(LED_PIN, INPUT);
	
	GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
	GIMSK |= _BV(INT0);                     // Enable INT0 Interrupts
	PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
	//PCMSK |= _BV(PCINT0);                   // Use PB0 as interrupt pin
	//ADCSRA &= ~_BV(ADEN);                   // ADC off
	MCUCR &= ~_BV(ISC00);
	MCUCR &= ~_BV(ISC01);
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
	sei();                                  // Enable interrupts
	sleep_cpu();                            // sleep
	
	// ------- now we're awake -------
	
	cli();                                  // Disable interrupts
	PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
	GIMSK &= ~_BV(PCIE);                     // Disable Pin Change Interrupts
	GIMSK &= ~_BV(INT0);                     // Disable INT0 Interrupts
	//PCMSK &= ~_BV(PCINT0);                  // Turn off PB0 as interrupt pin
	sleep_disable();                        // Clear SE bit
	//ADCSRA |= _BV(ADEN);                    // ADC on
	
	sei();                                  // Enable interrupts
	
	pinMode(LED_PIN, OUTPUT);
	button.reset();
	alarm.reset();
}

ISR(PCINT0_vect)
{
	event = BUTTON_PRESSED;
}

ISR(INT0_vect)
{
	event = ALARM_ON;
}

boolean firstLoop = true;
boolean dbgPinHigh = false;

void dbgToggle()
{
	dbgPinHigh = dbgPinHigh ? false : true;
	digitalWrite(DBG_P1, dbgPinHigh ? HIGH : LOW);
}

void loop()
{
	unsigned long msNow = millis();
	boolean goToSleep = false;
	
	button.loop(msNow);
	alarm.loop(msNow);
	
	if ( ! firstLoop && button.pressed())
	{
		event = BUTTON_PRESSED;
		//dbgToggle();
	}
	
	if (alarm.off())
	{
		event = ALARM_OFF;
		//dbgToggle();
	}
	
	firstLoop = false;
	
	switch (state)
	{
		case SLEEPING:
			switch (event)
		{
			case ALARM_ON:
				state = ALARM;
				digitalWrite(LED_PIN, HIGH);
				digitalWrite(DBG_P1, LOW);
				break;
			case ALARM_OFF:
				// nothing
				break;
			case BUTTON_PRESSED:
				state = LED_TEST;
				ledTestOnTime = msNow;
				digitalWrite(LED_PIN, HIGH);
				break;
		}
			break;
		case ALARM:
			switch (event)
		{
			case ALARM_ON:
				// nothing
				break;
			case ALARM_OFF:
				// nothing - wait for button
				state = SLEEP_PENDING;
				//dbgToggle();
				break;
			case BUTTON_PRESSED: // turn LED off and wait for alarm to stop before sleeping
				digitalWrite(LED_PIN, LOW);
				state = SLEEP_PENDING;
				//dbgToggle();
				break;
		}
			break;
		case LED_TEST:
			switch (event)
		{
			case ALARM_ON:
				state = ALARM;
				digitalWrite(LED_PIN, HIGH); // redundant
				break;
			case ALARM_OFF:
				break;
			case BUTTON_PRESSED:
				// ignore
				break;
			default:
				if (msNow - ledTestOnTime > 1000)
				{
					digitalWrite(LED_PIN, LOW);
					state = SLEEPING;
					goToSleep = true;
				}
				break;
		}
			break;
		case SLEEP_PENDING:
			switch (event)
		{
			case ALARM_ON:
				break;
			case ALARM_OFF:
				digitalWrite(LED_PIN, LOW);
				state = SLEEPING;
				goToSleep = true;
				break;
			case BUTTON_PRESSED:
				digitalWrite(LED_PIN, LOW);
				state = SLEEPING;
				goToSleep = true;
				break;
		}
			break;
	}
	
	//if (stateDbg)
	//  dbg();
	
	event = NONE;
	
	if (goToSleep)
	{
		//dbgToggle();
		sleep();
		button.reset();
		alarm.reset();
		//dbgToggle();
	}
}

