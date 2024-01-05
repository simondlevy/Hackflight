#pragma once

#include <errno.h>

#include <free_rtos.h>
#include <queue.h>
#include <task.h>

class Worker {

    public:

        void init()
        {
            if (_queue) {
                return;
            }

            _queue = xQueueCreateStatic(
                    QUEUE_LENGTH, ITEM_SIZE, _queueStorage, &_queueBuffer);
        }

        bool test()
        {
            return (_queue != NULL);
        }

        void loop()
        {
            if (!_queue) {
                return;
            }

            while (true)
            {
                work_t work = {};

                xQueueReceive(_queue, &work, portMAX_DELAY);

                if (work.function)
                    work.function(work.arg);
            }
        }

        int schedule(void (*function)(void*), void *arg)
        {
            if (!function) {
                return ENOEXEC;
            }

            work_t work = {};

            work.function = function;
            work.arg = arg;

            return xQueueSend(_queue, &work, 0) == pdFALSE ? ENOMEM : 0;
        }

    private:

        typedef struct {
            void (*function)(void*);
            void* arg;
        } work_t;

        static const auto ITEM_SIZE = sizeof(work_t);

        static const size_t QUEUE_LENGTH  = 5;

        uint8_t _queueStorage[QUEUE_LENGTH * ITEM_SIZE];

        StaticQueue_t _queueBuffer;

        xQueueHandle _queue;
};
