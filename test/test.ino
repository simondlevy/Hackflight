#include <arm_math.h>

static const size_t STATE_DIM = 7;

static void dumpmat(const float A[STATE_DIM][STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        for (size_t j=0; j<STATE_DIM; ++j) {
            printf("%+6.4f ", A[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

static void dumpvec(const float x[STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        printf("%+6.4f ", x[i]);
    }
    printf("\n");
}

//////////////////////////////////////////////////////////////////////////////

static void arm_mat_trans(
        const arm_matrix_instance_f32 * pSrc,
        arm_matrix_instance_f32 * pDst)
{
    arm_mat_trans_f32((arm_matrix_instance_f32 *)pSrc,
            (arm_matrix_instance_f32 *)pDst);
}

static void arm_mat_mult(
        const arm_matrix_instance_f32 * pSrcA,
        const arm_matrix_instance_f32 * pSrcB,
        arm_matrix_instance_f32 * pDst) 
{
    arm_mat_mult_f32(
            (arm_matrix_instance_f32 *)pSrcA,
            (arm_matrix_instance_f32 *)pSrcB,
            (arm_matrix_instance_f32 *)pDst);
}

static void arm_update_with_scalar(
        const float P[STATE_DIM][STATE_DIM],
        const float h[STATE_DIM],
        const float error,
        const float R,
        float G[STATE_DIM])
{
    arm_matrix_instance_f32 Hm = {1, STATE_DIM, (float *)h};

    static float Ht[STATE_DIM * 1];
    static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, Ht};

    static float PHt[STATE_DIM * 1];
    static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHt};

    arm_matrix_instance_f32 Pm = {};
    Pm.numRows = STATE_DIM;
    Pm.numCols = STATE_DIM;
    Pm.pData = (float*)P;

    arm_mat_trans(&Hm, &HTm);
    arm_mat_mult(&Pm, &HTm, &PHTm); // PH'

    float HPHR = R; // HPH' + R
    for (size_t i=0; i<STATE_DIM; i++) { 
        HPHR += Hm.pData[i]*PHt[i]; 
    }

    for (size_t i=0; i<STATE_DIM; i++) {
        G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    }

    float GH[STATE_DIM][STATE_DIM] = {};
    static float GH_I[STATE_DIM][STATE_DIM];
    static float GH_I_P[STATE_DIM][STATE_DIM];

    // Temporary matrices for the covariance updates
    static arm_matrix_instance_f32 GHm ={ STATE_DIM, STATE_DIM, (float *)GH };
    static arm_matrix_instance_f32 GH_Im = { STATE_DIM, STATE_DIM, (float *)GH_I };
    static arm_matrix_instance_f32 GH_I_Pm = { STATE_DIM, STATE_DIM, (float *)GH_I_P };

    // The Kalman gain as a column vector
    static arm_matrix_instance_f32 Gm = {STATE_DIM, 1, (float *)G};

    // GH
    arm_mat_mult(&Gm, &Hm, &GHm);

     // GH - I
    for (size_t i=0; i<STATE_DIM; i++) { 
        GH[i][i] -= 1; 
    }

    // (GH - I)'
    arm_mat_trans(&GHm, &GH_Im); 

    // (GH - I)*P
    arm_mat_mult(&GHm, &Pm, &GH_I_Pm); 

    // (GH - I)*P*(GH - I)'
    arm_mat_mult(&GH_I_Pm, &GH_Im, &Pm);
}

//////////////////////////////////////////////////////////////////////////////

// C = A * B
static void dot(
        const float A[STATE_DIM][STATE_DIM],
        const float B[STATE_DIM][STATE_DIM],
        float C[STATE_DIM][STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; i++) {
        for (size_t j=0; j<STATE_DIM; j++) {
            C[i][j] = 0;
            for (size_t k=0; k<STATE_DIM; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// y = A * x
static void dot(
        const float A[STATE_DIM][STATE_DIM],
        const float x[STATE_DIM],
        float y[STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; i++) {
        y[i] = 0; 
        for (size_t j=0; j<STATE_DIM; j++) {
            y[i] += A[i][j] * x[j];
        }
    }
}

static void outer(
        const float x[STATE_DIM],
        const float y[STATE_DIM],
        float C[STATE_DIM][STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; i++) {
        for (size_t j=0; j<STATE_DIM; j++) {
            C[i][j] = x[i] * y[j];
        }
    }
}

static void trans(
        const float A[STATE_DIM][STATE_DIM],
        float At[STATE_DIM][STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; i++) {
        for (size_t j=0; j<STATE_DIM; j++) {
            At[i][j] = A[j][i];
        }
    }
}

static void simple_update_with_scalar(
        float P[STATE_DIM][STATE_DIM],
        const float h[STATE_DIM],
        const float error,
        const float R,
        float G[STATE_DIM])
{
    float PHt[STATE_DIM] = {};
    dot(P, h, PHt); // PH'

    float HPHR = R; // HPH' + R
    for (size_t i=0; i<STATE_DIM; i++) { 
        HPHR += h[i] * PHt[i]; 
    }

    for (size_t i=0; i<STATE_DIM; i++) {
        G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    }

    float GH[STATE_DIM][STATE_DIM] = {};
    float GH_I[STATE_DIM][STATE_DIM] = {};
    float GH_I_P[STATE_DIM][STATE_DIM] = {};

    outer(G, h, GH);

    // GH - I
    for (size_t i=0; i<STATE_DIM; i++) { 
        GH[i][i] -= 1; 
    }

    // (GH - I)'
    trans(GH, GH_I);

    // (GH - I)*P
    dot(GH, P, GH_I_P); 

    // (GH - I)*P*(GH - I)'
    dot(GH_I_P, GH_I, P);
}

void setup()
{
    (void)dumpmat;
    (void)dumpvec;
}

void loop()
{
    const float h[STATE_DIM] = {1, 2, 3, 4, 5, 6, 7};

    const float Parm[STATE_DIM][STATE_DIM] = {
        { 8,  9, 10, 11, 12, 13, 14},
        {15, 16, 17, 18, 19, 20, 21},
        {22, 23, 24, 25, 26, 27, 28},
        {29, 30, 31, 32, 33, 34, 35},
        {36, 37, 38, 39, 40, 41, 42},
        {43, 44, 45, 46, 47, 48, 49},
        {50, 51, 52, 53, 54, 55, 56},
    };

    const float R = 57;

    const float error = 58;

    float Garm[STATE_DIM] = {};

    arm_update_with_scalar(Parm, h, error, R, Garm);

    dumpmat(Parm);

    // ----------------------------------------------------

    float Psimple[STATE_DIM][STATE_DIM] = {
        { 8,  9, 10, 11, 12, 13, 14},
        {15, 16, 17, 18, 19, 20, 21},
        {22, 23, 24, 25, 26, 27, 28},
        {29, 30, 31, 32, 33, 34, 35},
        {36, 37, 38, 39, 40, 41, 42},
        {43, 44, 45, 46, 47, 48, 49},
        {50, 51, 52, 53, 54, 55, 56},
    };

    float Gsimple[STATE_DIM] = {};

    simple_update_with_scalar(Psimple, h, error, R, Gsimple);

    dumpmat(Psimple);

    printf("\n ------------------------------------- \n");
    delay(1000);
}
