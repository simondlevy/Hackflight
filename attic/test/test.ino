#include <ArduinoEigenDense.h>

#include <iostream>

static const size_t STATE_DIM = 7;

//////////////////////////////////////////////////////////////////////////////

typedef Eigen::MatrixXd Matrix;

typedef Eigen::VectorXd Vector;

static void dumpmat(const Matrix A)
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        for (size_t j=0; j<STATE_DIM; ++j) {
            printf("%+6.4f ", A(i,j));
        }
        printf("\n");
    }
    printf("\n");
}

/*
static void dumpvec(const Vector x)
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        printf("%+6.4f ", x(i));
    }
    printf("\n");
}*/

static auto eigen_update_with_scalar(
        const Matrix P,
        const Vector h,
        const float error,
        const float R) -> Matrix
{
    const auto PHt = P * h;

    // Division here corresponds to matrix inversion in a full implementation
    const auto G = PHt / (R + h.dot(PHt));

    const auto GH_I =
        G * h.transpose() - Matrix::Identity(STATE_DIM, STATE_DIM);

    return GH_I * P * GH_I.transpose();
}


//////////////////////////////////////////////////////////////////////////////

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

/*
static void dumpvec(const float x[STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        printf("%+6.4f ", x[i]);
    }
    printf("\n");
}*/


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
        const float R)
{
    float G[STATE_DIM] = {};

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

//////////////////////////////////////////////////////////////////////////////

void setup()
{
}

void loop()
{
    const float R = 57;

    const float error = 58;

    // ----------------------------------------------------

    const float h_simple[STATE_DIM] = {1, 2, 3, 4, 5, 6, 7};

    float P_simple[STATE_DIM][STATE_DIM] = {
        { 8,  9, 10, 11, 12, 13, 14},
        {15, 16, 17, 18, 19, 20, 21},
        {22, 23, 24, 25, 26, 27, 28},
        {29, 30, 31, 32, 33, 34, 35},
        {36, 37, 38, 39, 40, 41, 42},
        {43, 44, 45, 46, 47, 48, 49},
        {50, 51, 52, 53, 54, 55, 56},
    };

    simple_update_with_scalar(P_simple, h_simple, error, R);

    dumpmat(P_simple);

    // ----------------------------------------------------

    auto h_eigen = Vector(STATE_DIM);
    h_eigen << 1, 2, 3, 4, 5, 6, 7;

    auto P_eigen = Matrix(STATE_DIM, STATE_DIM);

    P_eigen <<
         8,  9, 10, 11, 12, 13, 14,
        15, 16, 17, 18, 19, 20, 21,
        22, 23, 24, 25, 26, 27, 28,
        29, 30, 31, 32, 33, 34, 35,
        36, 37, 38, 39, 40, 41, 42,
        43, 44, 45, 46, 47, 48, 49,
        50, 51, 52, 53, 54, 55, 56;
 
    dumpmat(eigen_update_with_scalar(P_eigen, h_eigen, error, R));

    printf("\n ------------------------------------- \n");

    delay(1000);
}
