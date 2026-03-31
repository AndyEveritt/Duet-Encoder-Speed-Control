// CNC pendant interface to Duet
// D Crocker, started 2020-05-04

/* This Arduino sketch can be run on either Arduino Nano or Arduino Pro Micro.
 * It should also work on an Arduino Uno (using the same wiring scheme as for the Nano) or Arduino Leonardo (using the
same wiring scheme as for the Pro Micro).
 * The recommended board is the Arduino Pro Micro because the passthrough works without any modifications to the
Arduino.

*** Pendant to Arduino Pro Micro connections ***

Pro Micro Pendant   Wire colours
VCC       +5V       red
GND       0V,       black
		  COM,      orange/black
		  CN,       blue/black
		  LED-      white/black

D2        A         green
D3        B         white
D4        X         yellow
D5        Y         yellow/black
D6        Z         brown
D7        4         brown/black
D8        5         powder (if present)
D9        6         powder/black (if present)
D10       LED+      green/black
A0        STOP      blue
A1        X1        grey
A2        X10       grey/black
A3        X100      orange

NC        /A,       violet
		  /B        violet/black

*** Arduino Pro Micro to Duet PanelDue connector connections ***

Pro Micro Duet
VCC       +5V
GND       GND
TX1/D0    Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

To connect a PanelDue as well:

PanelDue +5V to +5V/VCC
PanelDue GND to GND
PanelDue DIN to Duet UTXD or IO_0_OUT
PanelDue DOUT to /Pro Micro RX1/D0.

*** Pendant to Arduino Nano connections ***

Nano    Pendant   Wire colours
+5V     +5V       red
GND     0V,       black
		COM,      orange/black
		CN,       blue/black
		LED-      white/black

D2      A         green
D3      B         white
D4      X         yellow
D5      Y         yellow/black
D6      Z         brown
D7      4         brown/black
D8      5         powder (if present)
D9      6         powder/black (if present)
D10     X1        grey
D11     X10       grey/black
D12     X100      orange
D13     LED+      green/black
A0      STOP      blue

NC      /A,       violet
		/B        violet/black

*** Arduino Nano to Duet PanelDue connector connections ***

Nano    Duet
+5V     +5V
GND     GND
TX1/D0  Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

To connect a PanelDue as well:

PanelDue +5V to +5V
PanelDue GND to GND
PanelDue DIN to Duet UTXD or IO_0_OUT
PanelDue DOUT to Nano/Pro Micro RX1/D0.

On the Arduino Nano is necessary to replace the 1K resistor between the USB interface chip by a 10K resistor so that
PanelDiue can override the USB chip. On Arduino Nano clones with CH340G chip, it is also necessary to remove the RxD LED
or its series resistor.

*/

#include <Arduino.h>

// Configuration constants
const int PinA = 2;
const int PinB = 3;
const int PinA2 = 4;
const int PinB2 = 5;

#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
const int PinLed = 10;
#endif

#if defined(__AVR_ATmega328P__) // Arduino Nano or Uno
const int PinLed = 13;
#endif

const unsigned long BaudRate = 57600;
const int PulsesPerClick = 4;
const unsigned long MinCommandInterval = 20;

#include "GCodeSerial.h"
#include "RotaryEncoder.h"

RotaryEncoder encoder(PinA, PinB, PulsesPerClick);
RotaryEncoder encoder2(PinA2, PinB2, PulsesPerClick);
RotaryEncoder* lastEncoder = nullptr;

int serialBufferSize;
int distanceMultiplier;
int speed = 100;
int speed2 = 100;
int pendingSpeedDelta0 = 0;
int pendingSpeedDelta1 = 0;
uint32_t whenLastCommandSent = 0;

struct PendingSpeedCommand
{
	bool active = false;
	uint32_t dueAt = 0;
	int value = 100;
};

PendingSpeedCommand pendingSpeed0;
PendingSpeedCommand pendingSpeed1;

#if defined(__AVR_ATmega32U4__) // Arduino Leonardo or Pro Micro
#  define UartSerial Serial1
#elif defined(__AVR_ATmega328P__) // Arduino Uno or Nano
#  define UartSerial Serial
#endif

GCodeSerial output(UartSerial);

void setup()
{
	encoder.init();
	encoder2.init();
	pinMode(PinLed, OUTPUT);

	output.begin(BaudRate);

	serialBufferSize = output.availableForWrite();

#if defined(__AVR_ATmega32U4__) // Arduino Leonardo or Pro Micro
	TX_RX_LED_INIT;
#endif
}

template <typename T>
static T clamp(T v, T min, T max)
{
	if (v < min)
		return min;
	if (v > max)
		return max;
	return v;
}

void loop()
{
	// 0. Poll both encoders and accumulate motion deltas. Ideally we would do this in the tick ISR, but after all these
	// years the Arduino core STILL doesn't let us hook it. We could possibly use interrupts instead, but if the encoder
	// suffers from contact bounce then that isn't a good idea. In practice this loop executes fast enough that polling
	// it here works well enough
	encoder.poll();
	encoder2.poll();
	const uint32_t now = millis();
	pendingSpeedDelta0 += encoder.getChange();
	pendingSpeedDelta1 += encoder2.getChange();

	digitalWrite(PinLed, HIGH);

	// 1. Send delayed M220 for P0 when due. This is non-blocking, so encoder polling continues while waiting.
	if (pendingSpeed0.active && now >= pendingSpeed0.dueAt && output.availableForWrite() == serialBufferSize)
	{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
		TXLED0;					// turn on transmit LED
#endif
		output.write("M220 S");
		output.print(pendingSpeed0.value);
		output.write('\n');
		pendingSpeed0.active = false;
		whenLastCommandSent = now;
	}

	// 2. Send delayed M220 for P1 when due. Only one M596->M220 pair can be in flight at a time.
	if (pendingSpeed1.active && now >= pendingSpeed1.dueAt && output.availableForWrite() == serialBufferSize)
	{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
		TXLED0;					// turn on transmit LED
#endif
		output.write("M220 S");
		output.print(pendingSpeed1.value);
		output.write('\n');
		pendingSpeed1.active = false;
		whenLastCommandSent = now;
	}

	// 3. If no delayed M220 is pending and the serial buffer is empty, start the next M596->M220 sequence.
	//    M596 is sent first and the corresponding M220 is queued for 100ms later.
	if (output.availableForWrite() == serialBufferSize && !pendingSpeed0.active && !pendingSpeed1.active)
	{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
		TXLED1;					// turn off transmit LED
#endif
		if (now - whenLastCommandSent >= MinCommandInterval)
		{
			constexpr uint32_t delayMs = 100;
			const bool canSendP0 = (pendingSpeedDelta0 != 0);
			const bool canSendP1 = (pendingSpeedDelta1 != 0);
			const bool sendP0 = canSendP0 && (!canSendP1 || lastEncoder == &encoder2);

			if (sendP0)
			{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
				TXLED0;			// turn on transmit LED
#endif
				speed = clamp(speed + pendingSpeedDelta0, 1, 1000);
				pendingSpeedDelta0 = 0;
				lastEncoder = &encoder;

				whenLastCommandSent = now;
				output.write("M596 P0\n");
				pendingSpeed0.active = true;
				pendingSpeed0.dueAt = now + delayMs;
				pendingSpeed0.value = speed;
			}
			else if (canSendP1)
			{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
				TXLED0;			// turn on transmit LED
#endif
				speed2 = clamp(speed2 + pendingSpeedDelta1, 1, 1000);
				pendingSpeedDelta1 = 0;
				lastEncoder = &encoder2;

				whenLastCommandSent = now;
				output.write("M596 P1\n");
				pendingSpeed1.active = true;
				pendingSpeed1.dueAt = now + delayMs;
				pendingSpeed1.value = speed2;
			}
		}
	}
}

// End
