/*
   Parser for serial comms

   MIT License
 */

#pragma once

void parse(bool avail, uint8_t byte,
        bool & sending, bool & receiving, uint8_t & pindex, uint8_t & msgtype)
{
    static uint8_t _parser_state;
    static uint8_t _input_size;
    static uint8_t _payload_index;
    static uint8_t _msgtype;
    static uint8_t _crc_in;

    bool incoming = _msgtype >= 200;

    // Payload functions
    _input_size = _parser_state == 3 ? byte : _input_size;
    _payload_index = _parser_state == 5 ? _payload_index + 1 : 0;
    receiving = incoming && _parser_state == 5 && _payload_index <= _input_size;

    // Command acquisition function
    _msgtype = _parser_state == 4 ? byte : _msgtype;

    // Checksum transition function
    _crc_in = _parser_state == 3 ? byte
        : _parser_state == 4  ?  _crc_in ^ byte 
        : receiving ?  _crc_in ^ byte
        : _parser_state == 5  ?  _crc_in
        : 0;

    // Parser state transition function
    _parser_state
        = _parser_state == 0 && byte == '$' ? 1
        : _parser_state == 1 && byte == 'M' ? 2
        : _parser_state == 2 && (byte == '<' || byte == '>') ? 3
        : _parser_state == 3 ? 4
        : _parser_state == 4 ? 5
        : _parser_state == 5 && receiving ? 5
        : _parser_state == 5 ? 0
        : _parser_state;

    msgtype = _msgtype;

    sending = avail && _parser_state == 0 && _crc_in == byte && !incoming;

    pindex = receiving ? _payload_index - 1 : 0;
}
