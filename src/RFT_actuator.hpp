/*
   Actuator class for mixers, etc.

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

namespace rft {

    class Actuator {

        friend class RFTPure;
        friend class ClosedLoopTask;
        friend class SerialTask;

        protected:

            virtual void begin(void) { }

            virtual void runDisarmed(void) { }

            virtual void cut(void) { }

        public:

            virtual uint8_t getType(void) { return 0; }

            virtual void run(float * demands, bool olcInactive) = 0;

            virtual void setMotorDisarmed(uint8_t index, float value)
            {
                (void)index;
                (void)value;
            }

    }; // class Actuator

} // namespace rft
