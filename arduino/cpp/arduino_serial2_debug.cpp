/*
   Arduino Serial2 debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void serial2Start(void)
{
    Serial2.begin(115200);
}

void serial2Debug(float v1, float v2, float v3, float v4)
{
    Serial2.print(v1);
    Serial2.print("    ");
    Serial2.print(v2);
    Serial2.print("    ");
    Serial2.print(v3);
    Serial2.print("    ");
    Serial2.println(v4);
}
