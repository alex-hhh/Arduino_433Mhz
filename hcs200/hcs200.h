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

// Structure declarations for HCS200, this file exists because Arduino messes
// up the .ino files.  See http://arduino.cc/en/Hacking/BuildProcess and
// http://stackoverflow.com/questions/24252631/typedef-not-working-as-parameter-or-return-in-arduino-sketch-function

#ifndef HCS200_H
#define HCS200_H

typedef unsigned long uint32_t;

/** Bit masks to determine which button was pressed on the keyfob.
 * 
 * NOTE: the bit position does not match the button number
 */
enum Hcs200_button_mask
{
    BM_S3 = 0x1,
    BM_S0 = 0x2,
    BM_S1 = 0x4,
    BM_S2 = 0x8
};

/** Hold data received from the keyfob.  The 'encrypted' field contains the
 * keycode information which would identify the key sequence and authenticate
 * the keyfob.  This cannot be decoded without the secret key.
 */
struct Hcs200_keycode
{
    uint32_t encrypted;
    uint32_t serial;
    unsigned char buttons;
};

#endif /* HCS200_H */
