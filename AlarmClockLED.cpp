#include <avr/sleep.h>
#include <avr/interrupt.h>

static const int ALARM_PIN = 2;
static const int BUTTON_PIN = 3;
static const int LED_PIN = 4;
static const int DBG_P1 = 1;

static const unsigned long CONFIG_CHANGE_PERIOD_MS = 3000;
static const unsigned long ALARM_ON_TIME_MS = 10000;

enum StateType
{
	SLEEPING,
	ALARM,
	LED_TEST,
	CHANGE_CONFIG,
	CONFIRM_CONFIG,
	SLEEP_PENDING
};

enum EventType // 'Event' won't compile
{
	NONE,
	ALARM_ON,
	ALARM_OFF,
	BUTTON_PRESSED,
	FLASH_FINISHED
};

class LED
{
public:
	LED(const int aPin)
	: pin(aPin)
	{
	}
	
	void setup()
	{
		pinMode(pin, OUTPUT);
	}
	
	void loop(const unsigned long msNow)
	{
	}
	
	void sleep()
	{
		pinMode(pin, INPUT);
	}
	
	void off()
	{
		digitalWrite(pin, LOW);
	}
	
	void on()
	{
		digitalWrite(pin, HIGH);
	}
	
private:
	int pin;
};

class Flasher
{
public:
	struct Phase
	{
		bool onState;
		unsigned long msDuration;
	};
	
	Flasher(LED& aLed) : led(aLed), lastChange(~0), phase(NULL), hasFinished(false)
	{
	}
	
	void setup()
	{
	}
	
	void loop(const unsigned long msNow)
	{
		if (phase != NULL)
		{
			if (msNow - lastChange > phase->msDuration)
			{
				phase++;
				
				if (phase->onState)
					led.on();
				else
					led.off();
				
				lastChange = msNow;
				
				if (phase->msDuration == 0) // finished
				{
					phase = NULL;
					hasFinished = true;
				}
			}
			
		}
	}
	
	void flash(const unsigned long msNow, const Phase* somePhases)
	{
		phase = somePhases;
		
		if (phase != NULL)
		{
			if (phase->onState)
				led.on();
			else
				led.off();
			
			lastChange = msNow;
		}
	}
	
	bool finished()
	{
		bool returnVal = hasFinished;
		hasFinished = false;
		return returnVal;
	}
	
private:
	LED led;
	unsigned long lastChange;
	const Phase* phase;
	bool hasFinished;
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
unsigned long changeConfigStartTime = 0;
unsigned long alarmStartTime = 0;
Button button(BUTTON_PIN);
Alarm alarm(ALARM_PIN);
LED led(LED_PIN);
Flasher flasher(led);
bool ignoreAlarm = false;

static const Flasher::Phase oneFlash[] = {
	{true, 400},
	{false, 0}
};

static const Flasher::Phase twoFlashes[] = {
	{true, 400},
	{false, 200},
	{true, 400},
	{false, 0},
};

void setup()
{
	pinMode(DBG_P1, OUTPUT);
	
	button.setup();
	alarm.setup();
	led.setup();
	flasher.setup();
	
	ignoreAlarm = false;
	
	// Flash quick sequence so we know setup has started
	for (int k = 0; k < 10; k++)
	{
		if (k % 2 == 0)
		{
			led.on();
		}
		else
		{
			led.off();
		}
		
		delay(100);
	}
	
	state = SLEEPING;
	sleep();
}

void sleep()
{
	led.sleep();
	
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
	
	led.setup();
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
	led.loop(msNow);
	flasher.loop(msNow);
	
	if ( ! firstLoop && button.pressed())
	{
		event = BUTTON_PRESSED;
	}
	
	if (alarm.off())
	{
		event = ALARM_OFF;
	}
	
	if (flasher.finished())
	{
		event = FLASH_FINISHED;
	}
	
	firstLoop = false;
	
	switch (state)
	{
		case SLEEPING:
			switch (event)
		{
			case ALARM_ON:
				if ( ! ignoreAlarm)
				{
					state = ALARM;
					alarmStartTime = msNow;
					led.on();
				}
				else
				{
					state = SLEEP_PENDING;
				}
				
				ignoreAlarm = !ignoreAlarm;
				
				break;
			case ALARM_OFF:
				// nothing
				break;
			case BUTTON_PRESSED:
				state = LED_TEST;
				flasher.flash(msNow, ignoreAlarm ? twoFlashes : oneFlash);
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
				break;
			case BUTTON_PRESSED: // turn LED off and wait for alarm to stop before sleeping
				led.off();
				state = SLEEP_PENDING;
				break;
			default:
				if (msNow - alarmStartTime >= ALARM_ON_TIME_MS)
				{
					led.off();
					state = SLEEP_PENDING;
				}
				break;
		}
			break;
		case LED_TEST:
			switch (event)
		{
			case ALARM_ON:
				break;
			case ALARM_OFF:
				break;
			case BUTTON_PRESSED:
				// ignore
				break;
			case FLASH_FINISHED:
				changeConfigStartTime = msNow;
				state = CHANGE_CONFIG;
				break;
		}
			break;
			
		case CHANGE_CONFIG:
			switch (event)
		{
			case BUTTON_PRESSED:
				ignoreAlarm = !ignoreAlarm;
				break;
			default:
				if (msNow - changeConfigStartTime >= CONFIG_CHANGE_PERIOD_MS)
				{
					flasher.flash(msNow, ignoreAlarm ? twoFlashes : oneFlash);
					state = CONFIRM_CONFIG;
				}
				break;
		}
			break;
		case CONFIRM_CONFIG:
			switch (event)
		{
			case FLASH_FINISHED:
				state = SLEEPING;
				goToSleep = true;
				break;
		}
			break;
		case SLEEP_PENDING:
			switch (event)
		{
			case ALARM_ON:
				break;
			case ALARM_OFF:
				led.off();
				state = SLEEPING;
				goToSleep = true;
				break;
			case BUTTON_PRESSED:
				led.off();
				state = SLEEPING;
				goToSleep = true;
				break;
		}
			break;
	}
	
	event = NONE;
	
	if (goToSleep)
	{
		sleep();
		button.reset();
		alarm.reset();
	}
}

