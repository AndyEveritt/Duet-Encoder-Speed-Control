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

int serialBufferSize;
int distanceMultiplier;
int speed;
int speed2;
uint32_t whenLastCommandSent = 0;

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
	// 0. Poll the encoder. Ideally we would do this in the tick ISR, but after all these years the Arduino core STILL
	// doesn't let us hook it. We could possibly use interrupts instead, but if the encoder suffers from contact bounce
	// then that isn't a good idea. In practice this loop executes fast enough that polling it here works well enough
	encoder.poll();
	encoder2.poll();

	digitalWrite(PinLed, HIGH);

	// 5. If the serial output buffer is empty, send a G0 command for the accumulated encoder motion.
	if (output.availableForWrite() == serialBufferSize)
	{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
		TXLED1;					// turn off transmit LED
#endif
		const uint32_t now = millis();
		if (now - whenLastCommandSent >= MinCommandInterval)
		{
			int speedDiff = encoder.getChange();
			int speedDiff2 = encoder2.getChange();
			if (speedDiff != 0)
			{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
				TXLED0;			// turn on transmit LED
#endif
				speed = clamp(speed + speedDiff, 1, 1000);

				whenLastCommandSent = now;
				output.write("M596 P0 M220 S");
				output.print(speed);
				output.write('\n');
			}
			if (speedDiff2 != 0)
			{
#if defined(__AVR_ATmega32U4__) // Arduino Micro, Pro Micro or Leonardo
				TXLED0;			// turn on transmit LED
#endif
				speed2 = clamp(speed2 + speedDiff2, 1, 1000);

				whenLastCommandSent = now;
				output.write("M596 P1 M220 S");
				output.print(speed2);
				output.write('\n');
			}
		}
	}
}

// End
