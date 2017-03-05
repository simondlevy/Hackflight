#pragma once

#include <cstdlib>

namespace hf {


class TimedTask {
private:
    uint64_t elapsedMicro;
    uint32_t periodMicro;

public:
    void init(uint32_t _periodMicro) {

        this->periodMicro = _periodMicro;
        this->elapsedMicro = 0;
    }

    bool checkAndUpdate(uint64_t currentTimeMicro) {

        bool result = (int32_t)(currentTimeMicro - this->elapsedMicro) >= 0;

        if (result)
            this->update(currentTimeMicro);

        return result;
    }

    void update(uint64_t currentTime) {

        this->elapsedMicro = currentTime + this->periodMicro;
    }

    bool check(uint64_t currentTime) {

        return (int32_t)(currentTime - this->elapsedMicro) >= 0;
    }
};

}
