#include <arm_math.h>

void setup() 
{
    Serial.begin(115200);
}

void loop() 
{
    float32_t A_f32[4] = {1.0, 2.0, 3.0, 4.0};
    float32_t B_f32[4] = {5.0, 6.0, 7.0, 8.0};

    const uint16_t numRowsA = 2;
    const uint16_t numColsA = 2;
    const uint16_t numRowsB = 2;
    const uint16_t numColsB = 2;

    arm_matrix_instance_f32 A, B, C;

    arm_mat_init_f32(&A, numRowsA, numColsA, A_f32);
    arm_mat_init_f32(&B, numRowsB, numColsB, B_f32);
    arm_mat_init_f32(&C, numRowsA, numColsB, NULL); // C will be the result matrix

    float32_t C_f32[numRowsA * numColsB];
    C.pData = C_f32;

    arm_status status = arm_mat_mult_f32(&A, &B, &C);

    if (status == ARM_MATH_SUCCESS) {

        Serial.println("Matrix C (A * B):");
        for (int i = 0; i < numRowsA; i++) {
            for (int j = 0; j < numColsB; j++) {
                Serial.print(C.pData[i * numColsB + j]);
                Serial.print(" ");
            }
            Serial.println();
        }
        Serial.println();
    } else {
        Serial.println("Matrix multiplication failed!");
    }

    delay(1000);
}
