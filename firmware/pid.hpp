extern "C" {

void pidCompute(
        int16_t rcCommand[4], 
        int16_t angle[3], 
        int16_t gyroADC[3],
        int16_t axisPID[3], 
        int32_t errorGyroI[3], 
        int32_t errorAngleI[3]);

} // extern "C"
