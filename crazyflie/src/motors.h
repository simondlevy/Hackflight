#pragma once

enum {
    MOTOR_M1,
    MOTOR_M2,
    MOTOR_M3,
    MOTOR_M4,
};

typedef enum {
    BRUSHED,
    BRUSHLESS,
} motorsDrvType;


#ifdef __cplusplus
extern "C" {
#endif

    void  motorsCheckDshot();
    int   motorsGetRatio(uint32_t id);
    void  motorsInit(void);
    bool  motorsTest(void);
    void  motorsSetRatios(const uint16_t ratios[]);
    void  motorsStop();

#ifdef __cplusplus
}
#endif
