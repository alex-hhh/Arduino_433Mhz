/*
 * Copyright (C) 2018 Alex Harsanyi <alexharsanyi@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hcs200.h"

/** Output pins for the leds on my board: one that lights up when a preamble
 * is received, and another one when a data transmission is in progress. 
 */
#define PREAMBLE_LED 7
#define DATA_LED 6

/** Possible states of the code word receiver. */
enum RxState {
    RS_NOSYNC = 0,                      // Receiver is inactive
    RS_PREAMBLE = 1,                    // 50% duty cycle
    RS_DATA = 2,                        // DATA is being received
    RS_COMPLETED = 3                    // Receive complete
};

/** The state of the code word received, this is 'volatile' since it is
 * accessed from both the main loop and the interrupt service routine.
 */
volatile char rx_state = RS_NOSYNC;

/** Pulse width lengths used by the transmitter.  A pulse width is the time
 * between a transition from 0 to 1 or from 1 to 0 of the input pin.
 */
enum RbType {
    RB_SHORT = 0,                       // pulse_width
    RB_LONG = 1,                        // 2 * pulse_width
    RB_ERROR = 2
};

/** Number of bits we expect to receive.  The HCS200 data sheet specifies that
 * 66 bits are transmited (plus an extra 16 reserved bits), but my keyfobs
 * only transmit 64 bits.  The missin bits are the "low voltage" bit, and a
 * bit that is always supposed to be "1".
 */
#define MAX_BITS 64

/** Time stamp when the previous interrupt was triggered (when the input pin
 * transitioned either from 0 to 1 or from 1 to 0.  This is the value of
 * micros() function.
 */
unsigned long last_timestamp = 0;

/** The duration of the last pulse width (the time between a transition of the
 * input pin)
 */
unsigned last_pulse_width = 0;

/** Number of bits received already. 
 */
char rx_bit_count = 0;

/** Buffer where we store the received bits.  We use 3 32 bit integers, so we
 * have space for up to 96 received bits. HCS sends only 66
 */
uint32_t rx_buf[3];

/** Duration of the transmitter clock -- this is determined when the preamble
 * is received, as the preamble has all the pulse widths the same length.
 *
 * The remotes I tested transmit at 2400 bauds, so the tx_clock is always
 * about 420 micro seconds, however, this code attempts to determine it
 * dynamically and should adapt to transmitters of different speeds.
 */
unsigned tx_clock = 0;

/** Determine if 'pulse' is a short pulse, RB_SHORT or a long one, RB_LONG.
 *  Normally, a short pulse is the same as the 'tx_clock' and a long pulse is
 *  twice as long.  However, timing problems with servicing interrupts means
 *  that we need to add some fudge factors.
 */
int Classify(unsigned pulse)
{
    int d = pulse - tx_clock;
    if (d < -100) {
        return RB_ERROR;
    }
    else if (d < 100) {
        return RB_SHORT;
    }
    else {
        d -= tx_clock;
        if (d < -100) {
            return RB_ERROR;
        }
        else if (d < 100) {
            return RB_LONG;
        }
        else {
            return RB_ERROR;
        }
    }
}

/** Interrupt service routine for PIN2.  This function is called when the
 * input pin 2 changes from 0 to 1 or from 1 to 0 -- this indicates that the
 * 433MHz carrier has been turned ON or OFF.  From this information we attempt
 * to decode the transmitted bits.
 */
void pin2ISR()
{
  unsigned long timestamp = micros();
  unsigned long pulse_width = timestamp - last_timestamp;
  int pin = digitalRead(2);

  switch (rx_state) {
  case RS_NOSYNC:
      // "Sync" is a high pulse, folowed by a long low
      if (pin == 1 && pulse_width > 10000 && pulse_width < 50000) {
          rx_state = RS_PREAMBLE;
          tx_clock = last_pulse_width;
      }
      break;
  case RS_PREAMBLE:
      if (pulse_width < 2 * tx_clock) {
           tx_clock = (tx_clock + pulse_width) >> 1;
      } else if (pin == 1 && pulse_width > 1000) {
          // pulse_width was for a long low, switch to receiving data.
          rx_state = RS_DATA;
          rx_bit_count = 0;
          memset(rx_buf, 0, sizeof(rx_buf));
      } else {
          rx_state = RS_NOSYNC;         // Transmission error
      }
      break;
  case RS_DATA:
      if (pin == 1) {
          int first = Classify(last_pulse_width);
          int second = Classify(pulse_width);
          if (first == RB_LONG && second == RB_SHORT) { // Received a 1 bit
              int idx = rx_bit_count / 32;
              rx_buf[idx] >>= 1;
              rx_buf[idx] |= 0x80000000;
              rx_bit_count++;
          }
          else if (first == RB_SHORT && second == RB_LONG) { // Received a 0 bit
              int idx = rx_bit_count / 32;
              rx_buf[idx] >>= 1;
              rx_bit_count++;
          }
          else {                        // invalid pulse combination
              rx_state = RS_NOSYNC;
          }
      }
      if (rx_bit_count >= MAX_BITS) {
          rx_state = RS_COMPLETED;
      }
      break;
  }

  last_timestamp = timestamp;
  last_pulse_width = pulse_width;
}

/** Decode the received data from 'rx_buf' into a Hcs200_keycode structure,
 * which holds the serial number of the keyfob and the buttons that were
 * pressed. */
void Hcs200Decode(uint32_t *rx_buf, Hcs200_keycode *out)
{
    out->encrypted = rx_buf[0];
    out->serial = rx_buf[1] & 0x0FFFFFFF;
    // NOTE: buttons are sent negated
    out->buttons = (~(rx_buf[1] >> 28)) & 0xF;
}

/** Print the contents of the received key code to the serial output -- this
 * can be viewed with the "Serial Monitor" in the Arduino IDE.
 */
void Hcs200Print(const Hcs200_keycode *keycode)
{
    Serial.print("Keyfob# ");
    Serial.print(keycode->serial, HEX);
    Serial.print(", buttons ");
    if (keycode->buttons & BM_S0) {
        Serial.print(" 1 ");
    }
    if (keycode->buttons & BM_S1) {
        Serial.print(" 2 ");
    }
    if (keycode->buttons & BM_S2) {
        Serial.print(" 3 ");
    }
    if (keycode->buttons & BM_S3) {
        Serial.print(" 4 ");
    }
    Serial.print(", code ");
    Serial.print(keycode->encrypted, HEX);
    Serial.print("\n");
}


// ....................................................... main program ....

void setup()
{
  Serial.begin(38400);
  // Board has the data bit wired to pin 8, and we bridge it to pin 2 to be
  // able to attach an interrupt.
  pinMode(8, INPUT);
  pinMode(2, INPUT);
  pinMode(PREAMBLE_LED, OUTPUT);
  pinMode(DATA_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), pin2ISR, CHANGE);
  Serial.print("Started listening\n");
}

void loop()
{
    digitalWrite(DATA_LED, rx_state == RS_DATA);
    digitalWrite(PREAMBLE_LED, rx_state == RS_PREAMBLE);

    if (rx_state == RS_COMPLETED) {
        if (rx_bit_count >= MAX_BITS) {
            Hcs200_keycode keycode;
            Hcs200Decode(rx_buf, &keycode);
            Hcs200Print(&keycode);
        }
        rx_state = RS_NOSYNC;
    }
}

/*
  Local Variables:
  mode: c++
  End:
*/
