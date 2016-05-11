extern "C" {

    class PID {

        private:

            int16_t lastGyro[3];
            int32_t delta1[3]; 
            int32_t delta2[3];

        public:

        void init(void);

        void compute(
                int16_t rcCommand[4], 
                int16_t angle[3], 
                int16_t gyroADC[3],
                int16_t axisPID[3], 
                int32_t errorGyroI[3], 
                int32_t errorAngleI[3]);
    }; 

} // extern "C"
