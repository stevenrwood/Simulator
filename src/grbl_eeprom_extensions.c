/*
  grbl_eeprom_extensions.c - 
  Grbl adds 2 functions to the orignal avr eeprom library. 
  They need to be reproduced here because we need to completely override the
  original eeprom interface for simulation

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Extensions added as part of Grbl 
// KEEP IN SYNC WITH ../eeprom.c

#include "eeprom.h"

#include "grbl/hal.h"
#include "grbl/crc.h"

bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint32_t dest = destination;
    // Compute the checksum over the source data up front: the write loop below consumes 'size' and
    // 'source', so the original code's trailing "if(size > 0 && with_checksum)" was always false
    // (size == 0 here) and the checksum was never written. Settings then failed the read-back check
    // and were wiped to defaults on every boot, so nothing ever persisted. Mirror nvs_buffer.c's
    // memcpy_to_ram: checksum first, then data, then the trailing CRC byte(s) at 'dest'.
    uint16_t checksum = with_checksum ? calc_checksum(source, size) : 0;

    for(; size > 0; size--)
        eeprom_put_char(dest++, *(source++));

    if(with_checksum) {
        eeprom_put_char(dest++, checksum & 0xFF);
#if NVS_CRC_BYTES > 1
        eeprom_put_char(dest, checksum >> 8);
#endif
    }

    return true;
}

bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint8_t *dest = destination;   // original start of the read buffer (destination is advanced below)
    uint32_t sz = size;

    for(; sz > 0; sz--)
        *(destination++) = eeprom_get_char(source++);

    // 'source' now points at the stored CRC (just past the data); checksum is over the data we read
    // into 'dest' (the original code used the post-loop 'destination', i.e. past the end - a bug).
#if NVS_CRC_BYTES == 1
    return !with_checksum || calc_checksum(dest, size) == eeprom_get_char(source);
#else
    return !with_checksum || calc_checksum(dest, size) == (eeprom_get_char(source) | (eeprom_get_char(source + 1) << 8));
#endif
}

// end of file
