/*
   Timer task for serial comms

   Gnu Public License
 */

#pragma once

#include <RFT_board.hpp>
#include <RFT_debugger.hpp>
#include <RFT_actuator.hpp>
#include <RFT_parser.hpp>
#include <RFT_serialtask.hpp>


class SerialTask : public rft::SerialTask {

    friend class /* XXX */;

    private:

    uint8_t _payload[128] = {};

    void handle_PAA3905_Request(int16_t & x, int16_t & y)
    {
        // XXX
    }

    protected:

    virtual void collectPayload(uint8_t index, uint8_t value) override
    {
        _payload[index] = value;
    }

    virtual void dispatchMessage(uint8_t command) override
    {
        switch (command) {

            case 121:
                {
                    int16_t x = 0;
                    int16_t y = 0;
                    handle_PAA3905_Request(x, y);
                    prepareToSendShorts(command, 2);
                    sendShort(x);
                    sendShort(y);
                    completeSend();
                } break;

        } // switch (_command)

    } // dispatchMessage 

}; // class SerialTask
