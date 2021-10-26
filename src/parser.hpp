/*
   Parser for serial comms

   MIT License
 */

#pragma once

void parse(
          bool avail
        , uint8_t byte
        , bool & sending
        , bool & receiving
        , uint8_t & pindex
        , uint8_t & msgtype
        )
{
    static uint8_t _state;
    static uint8_t _size;
    static uint8_t _index;
    static uint8_t _type;
    static uint8_t _crc;
    static uint8_t _receiving;

    bool incoming = _size > 0;

    _size = _state == 3 ? byte : _size;

    _type = _state == 4 ? byte : _type;

    _index = _state == 5 ? _index + 1 : 0;

    _receiving = incoming && _state == 5 && _index <= _size;

    _crc = _state == 4 ? byte
        : _state == 5  ?  _crc
        : receiving ?  _crc ^ byte
        : 0;

    _state
        = _state == 0 && byte == '$' ? 1
        : _state == 1 && byte == 'M' ? 2
        : _state == 2 && (byte == '<' || byte == '>') ? 3
        : _state == 3 ? 4
        : _state == 4 ? 5
        : _state == 5 && _receiving ? 5
        : _state == 5 ? 0
        : _state;

    msgtype = _type;

    receiving = _receiving;

    sending = avail && _state == 0 && _crc == byte && !incoming;

    pindex = receiving ? _index - 1 : 0;
}
