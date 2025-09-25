#include <free_rtos_include.h>

class Task {

    private:

        static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

        typedef void (*taskfun_t)(void * obj);

        StackType_t _taskStackBuffer[STACKSIZE];

        StaticTask_t _taskBuffer;

    public:

        void init(
                const taskfun_t fun,
                const char * name,
                void * obj,
                const uint8_t priority
                )
        {
            xTaskCreateStatic(
                    fun,      
                    name,              
                    STACKSIZE,    
                    obj,
                    priority,
                    _taskStackBuffer,         
                    &_taskBuffer);        
        }


};
