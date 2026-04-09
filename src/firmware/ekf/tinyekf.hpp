// AUTO-GENERATED CODE; DO NOT EDIT

#pragma once

#include <math.h>

namespace tinyekf {

    class Vector {

        public:

            float _0, _1, _2, _3, _4, _5, _6;

            Vector()
               : _0(0), _1(0), _2(0), _3(0), _4(0), _5(0), _6(6) {}

            Vector(const float _0, const float _1, const float _2, const float _3, const float _4, const float _5, const float _6)
               : _0(_0), _1(_1), _2(_2), _3(_3), _4(_4), _5(_5), _6(_6) {}
    };

    class Matrix {

        public:

            float
               _00,  _01,  _02,  _03,  _04,  _05,  _06, 
               _10,  _11,  _12,  _13,  _14,  _15,  _16, 
               _20,  _21,  _22,  _23,  _24,  _25,  _26, 
               _30,  _31,  _32,  _33,  _34,  _35,  _36, 
               _40,  _41,  _42,  _43,  _44,  _45,  _46, 
               _50,  _51,  _52,  _53,  _54,  _55,  _56, 
               _60,  _61,  _62,  _63,  _64,  _65,  _66;

            Matrix() 
                :
                    _00(0),  _01(0),  _02(0),  _03(0),  _04(0),  _05(0),  _06(0), 
                    _10(0),  _11(0),  _12(0),  _13(0),  _14(0),  _15(0),  _16(0), 
                    _20(0),  _21(0),  _22(0),  _23(0),  _24(0),  _25(0),  _26(0), 
                    _30(0),  _31(0),  _32(0),  _33(0),  _34(0),  _35(0),  _36(0), 
                    _40(0),  _41(0),  _42(0),  _43(0),  _44(0),  _45(0),  _46(0), 
                    _50(0),  _51(0),  _52(0),  _53(0),  _54(0),  _55(0),  _56(0), 
                    _60(0),  _61(0),  _62(0),  _63(0),  _64(0),  _65(0),  _66(0) {}


            Matrix(
                    const float _00, const float _01, const float _02, const float _03, const float _04, const float _05, const float _06, 
                    const float _10, const float _11, const float _12, const float _13, const float _14, const float _15, const float _16, 
                    const float _20, const float _21, const float _22, const float _23, const float _24, const float _25, const float _26, 
                    const float _30, const float _31, const float _32, const float _33, const float _34, const float _35, const float _36, 
                    const float _40, const float _41, const float _42, const float _43, const float _44, const float _45, const float _46, 
                    const float _50, const float _51, const float _52, const float _53, const float _54, const float _55, const float _56, 
                    const float _60, const float _61, const float _62, const float _63, const float _64, const float _65, const float _66)
                :
                    _00(_00), _01(_01), _02(_02), _03(_03), _04(_04), _05(_05), _06(_06), 
                    _10(_10), _11(_11), _12(_12), _13(_13), _14(_14), _15(_15), _16(_16), 
                    _20(_20), _21(_21), _22(_22), _23(_23), _24(_24), _25(_25), _26(_26), 
                    _30(_30), _31(_31), _32(_32), _33(_33), _34(_34), _35(_35), _36(_36), 
                    _40(_40), _41(_41), _42(_42), _43(_43), _44(_44), _45(_45), _46(_46), 
                    _50(_50), _51(_51), _52(_52), _53(_53), _54(_54), _55(_55), _56(_56), 
                    _60(_60), _61(_61), _62(_62), _63(_63), _64(_64), _65(_65), _66(_66) {}
    };

    class Core {

        public:

            Vector x;
            Matrix P;

            Core() = default;

            Core(Vector & x, Matrix & P) : x(x), P(P) {}

            // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
            void predict(const Matrix &F)
            {
                Matrix FP;
                dot(F, P, FP);
                Matrix Ft;
                trans(F, Ft);
                dot(FP, Ft, P);
            }

            void updateWithScalar(const Vector & h,
                    const float error,
                    const float stdMeasNoise,
                    const float minCovariance,
                    const float maxCovariance)
            {
                const auto R = stdMeasNoise * stdMeasNoise;

                Vector PHt;
                dot(P, h, PHt); // PH

                const auto HPHR = R + h._0*PHt._0 + h._1*PHt._1 + h._2*PHt._2 + h._3*PHt._3 + h._4*PHt._4 + h._5*PHt._5 + h._6*PHt._6;

                // kalman gain = (PH' (HPH' + R )^-1)
                const auto G = Vector(PHt._0/HPHR, PHt._1/HPHR, PHt._2/HPHR, PHt._3/HPHR, PHt._4/HPHR, PHt._5/HPHR, PHt._6/HPHR);

                Matrix GH;
                outer(G, h, GH);

                // GH - I
                GH._00-=1; GH._11-=1; GH._22-=1; GH._33-=1; GH._44-=1; GH._55-=1; GH._66-=1; 

                // (GH - I)
                Matrix GH_I;
                trans(GH, GH_I);


                // (GH - I)*P
                Matrix GH_I_P;
                dot(GH, P, GH_I_P);

                // State updated
                x._0+=G._0 *error; x._1+=G._1 *error; x._2+=G._2 *error; x._3+=G._3 *error; x._4+=G._4 *error; x._5+=G._5 *error; x._6+=G._6 *error; 

                // Add covariance noise and ensure boundedness and symmetry
                P._00 = get_pval(0, 0, 0.5*P._00 + 0.5*P._00 + G._0 * R * G._0, minCovariance, maxCovariance);
                P._01 = P._10 = get_pval(0, 1, 0.5*P._01 + 0.5*P._10 + G._0 * R * G._1, minCovariance, maxCovariance);
                P._02 = P._20 = get_pval(0, 2, 0.5*P._02 + 0.5*P._20 + G._0 * R * G._2, minCovariance, maxCovariance);
                P._03 = P._30 = get_pval(0, 3, 0.5*P._03 + 0.5*P._30 + G._0 * R * G._3, minCovariance, maxCovariance);
                P._04 = P._40 = get_pval(0, 4, 0.5*P._04 + 0.5*P._40 + G._0 * R * G._4, minCovariance, maxCovariance);
                P._05 = P._50 = get_pval(0, 5, 0.5*P._05 + 0.5*P._50 + G._0 * R * G._5, minCovariance, maxCovariance);
                P._06 = P._60 = get_pval(0, 6, 0.5*P._06 + 0.5*P._60 + G._0 * R * G._6, minCovariance, maxCovariance);
                P._10 = P._01 = get_pval(1, 0, 0.5*P._10 + 0.5*P._01 + G._1 * R * G._0, minCovariance, maxCovariance);
                P._11 = get_pval(1, 1, 0.5*P._11 + 0.5*P._11 + G._1 * R * G._1, minCovariance, maxCovariance);
                P._12 = P._21 = get_pval(1, 2, 0.5*P._12 + 0.5*P._21 + G._1 * R * G._2, minCovariance, maxCovariance);
                P._13 = P._31 = get_pval(1, 3, 0.5*P._13 + 0.5*P._31 + G._1 * R * G._3, minCovariance, maxCovariance);
                P._14 = P._41 = get_pval(1, 4, 0.5*P._14 + 0.5*P._41 + G._1 * R * G._4, minCovariance, maxCovariance);
                P._15 = P._51 = get_pval(1, 5, 0.5*P._15 + 0.5*P._51 + G._1 * R * G._5, minCovariance, maxCovariance);
                P._16 = P._61 = get_pval(1, 6, 0.5*P._16 + 0.5*P._61 + G._1 * R * G._6, minCovariance, maxCovariance);
                P._20 = P._02 = get_pval(2, 0, 0.5*P._20 + 0.5*P._02 + G._2 * R * G._0, minCovariance, maxCovariance);
                P._21 = P._12 = get_pval(2, 1, 0.5*P._21 + 0.5*P._12 + G._2 * R * G._1, minCovariance, maxCovariance);
                P._22 = get_pval(2, 2, 0.5*P._22 + 0.5*P._22 + G._2 * R * G._2, minCovariance, maxCovariance);
                P._23 = P._32 = get_pval(2, 3, 0.5*P._23 + 0.5*P._32 + G._2 * R * G._3, minCovariance, maxCovariance);
                P._24 = P._42 = get_pval(2, 4, 0.5*P._24 + 0.5*P._42 + G._2 * R * G._4, minCovariance, maxCovariance);
                P._25 = P._52 = get_pval(2, 5, 0.5*P._25 + 0.5*P._52 + G._2 * R * G._5, minCovariance, maxCovariance);
                P._26 = P._62 = get_pval(2, 6, 0.5*P._26 + 0.5*P._62 + G._2 * R * G._6, minCovariance, maxCovariance);
                P._30 = P._03 = get_pval(3, 0, 0.5*P._30 + 0.5*P._03 + G._3 * R * G._0, minCovariance, maxCovariance);
                P._31 = P._13 = get_pval(3, 1, 0.5*P._31 + 0.5*P._13 + G._3 * R * G._1, minCovariance, maxCovariance);
                P._32 = P._23 = get_pval(3, 2, 0.5*P._32 + 0.5*P._23 + G._3 * R * G._2, minCovariance, maxCovariance);
                P._33 = get_pval(3, 3, 0.5*P._33 + 0.5*P._33 + G._3 * R * G._3, minCovariance, maxCovariance);
                P._34 = P._43 = get_pval(3, 4, 0.5*P._34 + 0.5*P._43 + G._3 * R * G._4, minCovariance, maxCovariance);
                P._35 = P._53 = get_pval(3, 5, 0.5*P._35 + 0.5*P._53 + G._3 * R * G._5, minCovariance, maxCovariance);
                P._36 = P._63 = get_pval(3, 6, 0.5*P._36 + 0.5*P._63 + G._3 * R * G._6, minCovariance, maxCovariance);
                P._40 = P._04 = get_pval(4, 0, 0.5*P._40 + 0.5*P._04 + G._4 * R * G._0, minCovariance, maxCovariance);
                P._41 = P._14 = get_pval(4, 1, 0.5*P._41 + 0.5*P._14 + G._4 * R * G._1, minCovariance, maxCovariance);
                P._42 = P._24 = get_pval(4, 2, 0.5*P._42 + 0.5*P._24 + G._4 * R * G._2, minCovariance, maxCovariance);
                P._43 = P._34 = get_pval(4, 3, 0.5*P._43 + 0.5*P._34 + G._4 * R * G._3, minCovariance, maxCovariance);
                P._44 = get_pval(4, 4, 0.5*P._44 + 0.5*P._44 + G._4 * R * G._4, minCovariance, maxCovariance);
                P._45 = P._54 = get_pval(4, 5, 0.5*P._45 + 0.5*P._54 + G._4 * R * G._5, minCovariance, maxCovariance);
                P._46 = P._64 = get_pval(4, 6, 0.5*P._46 + 0.5*P._64 + G._4 * R * G._6, minCovariance, maxCovariance);
                P._50 = P._05 = get_pval(5, 0, 0.5*P._50 + 0.5*P._05 + G._5 * R * G._0, minCovariance, maxCovariance);
                P._51 = P._15 = get_pval(5, 1, 0.5*P._51 + 0.5*P._15 + G._5 * R * G._1, minCovariance, maxCovariance);
                P._52 = P._25 = get_pval(5, 2, 0.5*P._52 + 0.5*P._25 + G._5 * R * G._2, minCovariance, maxCovariance);
                P._53 = P._35 = get_pval(5, 3, 0.5*P._53 + 0.5*P._35 + G._5 * R * G._3, minCovariance, maxCovariance);
                P._54 = P._45 = get_pval(5, 4, 0.5*P._54 + 0.5*P._45 + G._5 * R * G._4, minCovariance, maxCovariance);
                P._55 = get_pval(5, 5, 0.5*P._55 + 0.5*P._55 + G._5 * R * G._5, minCovariance, maxCovariance);
                P._56 = P._65 = get_pval(5, 6, 0.5*P._56 + 0.5*P._65 + G._5 * R * G._6, minCovariance, maxCovariance);
                P._60 = P._06 = get_pval(6, 0, 0.5*P._60 + 0.5*P._06 + G._6 * R * G._0, minCovariance, maxCovariance);
                P._61 = P._16 = get_pval(6, 1, 0.5*P._61 + 0.5*P._16 + G._6 * R * G._1, minCovariance, maxCovariance);
                P._62 = P._26 = get_pval(6, 2, 0.5*P._62 + 0.5*P._26 + G._6 * R * G._2, minCovariance, maxCovariance);
                P._63 = P._36 = get_pval(6, 3, 0.5*P._63 + 0.5*P._36 + G._6 * R * G._3, minCovariance, maxCovariance);
                P._64 = P._46 = get_pval(6, 4, 0.5*P._64 + 0.5*P._46 + G._6 * R * G._4, minCovariance, maxCovariance);
                P._65 = P._56 = get_pval(6, 5, 0.5*P._65 + 0.5*P._56 + G._6 * R * G._5, minCovariance, maxCovariance);
                P._66 = get_pval(6, 6, 0.5*P._66 + 0.5*P._66 + G._6 * R * G._6, minCovariance, maxCovariance);
            }

            void addCovarianceNoise(const Vector & noise)
            {
                P._00 += noise._0;
                P._11 += noise._1;
                P._22 += noise._2;
                P._33 += noise._3;
                P._44 += noise._4;
                P._55 += noise._5;
                P._66 += noise._6;
            }

            void enforceSymmetry(const float minval, const float maxval)
            {
                P._00 = get_pval(0, 0, 0.5*P._00 + 0.5*P._00, minval, maxval);
                P._01 = P._10 = get_pval(0, 1, 0.5*P._01 + 0.5*P._10, minval, maxval);
                P._02 = P._20 = get_pval(0, 2, 0.5*P._02 + 0.5*P._20, minval, maxval);
                P._03 = P._30 = get_pval(0, 3, 0.5*P._03 + 0.5*P._30, minval, maxval);
                P._04 = P._40 = get_pval(0, 4, 0.5*P._04 + 0.5*P._40, minval, maxval);
                P._05 = P._50 = get_pval(0, 5, 0.5*P._05 + 0.5*P._50, minval, maxval);
                P._06 = P._60 = get_pval(0, 6, 0.5*P._06 + 0.5*P._60, minval, maxval);
                P._10 = P._01 = get_pval(1, 0, 0.5*P._10 + 0.5*P._01, minval, maxval);
                P._11 = get_pval(1, 1, 0.5*P._11 + 0.5*P._11, minval, maxval);
                P._12 = P._21 = get_pval(1, 2, 0.5*P._12 + 0.5*P._21, minval, maxval);
                P._13 = P._31 = get_pval(1, 3, 0.5*P._13 + 0.5*P._31, minval, maxval);
                P._14 = P._41 = get_pval(1, 4, 0.5*P._14 + 0.5*P._41, minval, maxval);
                P._15 = P._51 = get_pval(1, 5, 0.5*P._15 + 0.5*P._51, minval, maxval);
                P._16 = P._61 = get_pval(1, 6, 0.5*P._16 + 0.5*P._61, minval, maxval);
                P._20 = P._02 = get_pval(2, 0, 0.5*P._20 + 0.5*P._02, minval, maxval);
                P._21 = P._12 = get_pval(2, 1, 0.5*P._21 + 0.5*P._12, minval, maxval);
                P._22 = get_pval(2, 2, 0.5*P._22 + 0.5*P._22, minval, maxval);
                P._23 = P._32 = get_pval(2, 3, 0.5*P._23 + 0.5*P._32, minval, maxval);
                P._24 = P._42 = get_pval(2, 4, 0.5*P._24 + 0.5*P._42, minval, maxval);
                P._25 = P._52 = get_pval(2, 5, 0.5*P._25 + 0.5*P._52, minval, maxval);
                P._26 = P._62 = get_pval(2, 6, 0.5*P._26 + 0.5*P._62, minval, maxval);
                P._30 = P._03 = get_pval(3, 0, 0.5*P._30 + 0.5*P._03, minval, maxval);
                P._31 = P._13 = get_pval(3, 1, 0.5*P._31 + 0.5*P._13, minval, maxval);
                P._32 = P._23 = get_pval(3, 2, 0.5*P._32 + 0.5*P._23, minval, maxval);
                P._33 = get_pval(3, 3, 0.5*P._33 + 0.5*P._33, minval, maxval);
                P._34 = P._43 = get_pval(3, 4, 0.5*P._34 + 0.5*P._43, minval, maxval);
                P._35 = P._53 = get_pval(3, 5, 0.5*P._35 + 0.5*P._53, minval, maxval);
                P._36 = P._63 = get_pval(3, 6, 0.5*P._36 + 0.5*P._63, minval, maxval);
                P._40 = P._04 = get_pval(4, 0, 0.5*P._40 + 0.5*P._04, minval, maxval);
                P._41 = P._14 = get_pval(4, 1, 0.5*P._41 + 0.5*P._14, minval, maxval);
                P._42 = P._24 = get_pval(4, 2, 0.5*P._42 + 0.5*P._24, minval, maxval);
                P._43 = P._34 = get_pval(4, 3, 0.5*P._43 + 0.5*P._34, minval, maxval);
                P._44 = get_pval(4, 4, 0.5*P._44 + 0.5*P._44, minval, maxval);
                P._45 = P._54 = get_pval(4, 5, 0.5*P._45 + 0.5*P._54, minval, maxval);
                P._46 = P._64 = get_pval(4, 6, 0.5*P._46 + 0.5*P._64, minval, maxval);
                P._50 = P._05 = get_pval(5, 0, 0.5*P._50 + 0.5*P._05, minval, maxval);
                P._51 = P._15 = get_pval(5, 1, 0.5*P._51 + 0.5*P._15, minval, maxval);
                P._52 = P._25 = get_pval(5, 2, 0.5*P._52 + 0.5*P._25, minval, maxval);
                P._53 = P._35 = get_pval(5, 3, 0.5*P._53 + 0.5*P._35, minval, maxval);
                P._54 = P._45 = get_pval(5, 4, 0.5*P._54 + 0.5*P._45, minval, maxval);
                P._55 = get_pval(5, 5, 0.5*P._55 + 0.5*P._55, minval, maxval);
                P._56 = P._65 = get_pval(5, 6, 0.5*P._56 + 0.5*P._65, minval, maxval);
                P._60 = P._06 = get_pval(6, 0, 0.5*P._60 + 0.5*P._06, minval, maxval);
                P._61 = P._16 = get_pval(6, 1, 0.5*P._61 + 0.5*P._16, minval, maxval);
                P._62 = P._26 = get_pval(6, 2, 0.5*P._62 + 0.5*P._26, minval, maxval);
                P._63 = P._36 = get_pval(6, 3, 0.5*P._63 + 0.5*P._36, minval, maxval);
                P._64 = P._46 = get_pval(6, 4, 0.5*P._64 + 0.5*P._46, minval, maxval);
                P._65 = P._56 = get_pval(6, 5, 0.5*P._65 + 0.5*P._56, minval, maxval);
                P._66 = get_pval(6, 6, 0.5*P._66 + 0.5*P._66, minval, maxval);
            }


        private:

            static auto get_pval(const int i, const int j, 
              const float pval, const float minval,
              const float maxval) -> float
            {
              return
                isnan(pval) || pval > maxval ? maxval : 
                i==j && pval < minval ? minval : 
                pval; 
            }

            // y = A * x
            static void dot(const Matrix & A, const Vector &x, Vector &y)
            {
                y._0 = A._00 +  A._01 +  A._02 +  A._03 +  A._04 +  A._05 +  A._06;
                 y._1 = A._10 +  A._11 +  A._12 +  A._13 +  A._14 +  A._15 +  A._16;
                 y._2 = A._20 +  A._21 +  A._22 +  A._23 +  A._24 +  A._25 +  A._26;
                 y._3 = A._30 +  A._31 +  A._32 +  A._33 +  A._34 +  A._35 +  A._36;
                 y._4 = A._40 +  A._41 +  A._42 +  A._43 +  A._44 +  A._45 +  A._46;
                 y._5 = A._50 +  A._51 +  A._52 +  A._53 +  A._54 +  A._55 +  A._56;
                 y._6 = A._60 +  A._61 +  A._62 +  A._63 +  A._64 +  A._65 +  A._66;
             }

            // C = A * B
            static void dot(const Matrix & A, const Matrix &B, Matrix &C)
            {
                C._00 = A._00*B._00 + A._01*B._10 + A._02*B._20 + A._03*B._30 + A._04*B._40 + A._05*B._50 + A._06*B._60;
                C._01 = A._00*B._01 + A._01*B._11 + A._02*B._21 + A._03*B._31 + A._04*B._41 + A._05*B._51 + A._06*B._61;
                C._02 = A._00*B._02 + A._01*B._12 + A._02*B._22 + A._03*B._32 + A._04*B._42 + A._05*B._52 + A._06*B._62;
                C._03 = A._00*B._03 + A._01*B._13 + A._02*B._23 + A._03*B._33 + A._04*B._43 + A._05*B._53 + A._06*B._63;
                C._04 = A._00*B._04 + A._01*B._14 + A._02*B._24 + A._03*B._34 + A._04*B._44 + A._05*B._54 + A._06*B._64;
                C._05 = A._00*B._05 + A._01*B._15 + A._02*B._25 + A._03*B._35 + A._04*B._45 + A._05*B._55 + A._06*B._65;
                C._06 = A._00*B._06 + A._01*B._16 + A._02*B._26 + A._03*B._36 + A._04*B._46 + A._05*B._56 + A._06*B._66;
                C._10 = A._10*B._00 + A._11*B._10 + A._12*B._20 + A._13*B._30 + A._14*B._40 + A._15*B._50 + A._16*B._60;
                C._11 = A._10*B._01 + A._11*B._11 + A._12*B._21 + A._13*B._31 + A._14*B._41 + A._15*B._51 + A._16*B._61;
                C._12 = A._10*B._02 + A._11*B._12 + A._12*B._22 + A._13*B._32 + A._14*B._42 + A._15*B._52 + A._16*B._62;
                C._13 = A._10*B._03 + A._11*B._13 + A._12*B._23 + A._13*B._33 + A._14*B._43 + A._15*B._53 + A._16*B._63;
                C._14 = A._10*B._04 + A._11*B._14 + A._12*B._24 + A._13*B._34 + A._14*B._44 + A._15*B._54 + A._16*B._64;
                C._15 = A._10*B._05 + A._11*B._15 + A._12*B._25 + A._13*B._35 + A._14*B._45 + A._15*B._55 + A._16*B._65;
                C._16 = A._10*B._06 + A._11*B._16 + A._12*B._26 + A._13*B._36 + A._14*B._46 + A._15*B._56 + A._16*B._66;
                C._20 = A._20*B._00 + A._21*B._10 + A._22*B._20 + A._23*B._30 + A._24*B._40 + A._25*B._50 + A._26*B._60;
                C._21 = A._20*B._01 + A._21*B._11 + A._22*B._21 + A._23*B._31 + A._24*B._41 + A._25*B._51 + A._26*B._61;
                C._22 = A._20*B._02 + A._21*B._12 + A._22*B._22 + A._23*B._32 + A._24*B._42 + A._25*B._52 + A._26*B._62;
                C._23 = A._20*B._03 + A._21*B._13 + A._22*B._23 + A._23*B._33 + A._24*B._43 + A._25*B._53 + A._26*B._63;
                C._24 = A._20*B._04 + A._21*B._14 + A._22*B._24 + A._23*B._34 + A._24*B._44 + A._25*B._54 + A._26*B._64;
                C._25 = A._20*B._05 + A._21*B._15 + A._22*B._25 + A._23*B._35 + A._24*B._45 + A._25*B._55 + A._26*B._65;
                C._26 = A._20*B._06 + A._21*B._16 + A._22*B._26 + A._23*B._36 + A._24*B._46 + A._25*B._56 + A._26*B._66;
                C._30 = A._30*B._00 + A._31*B._10 + A._32*B._20 + A._33*B._30 + A._34*B._40 + A._35*B._50 + A._36*B._60;
                C._31 = A._30*B._01 + A._31*B._11 + A._32*B._21 + A._33*B._31 + A._34*B._41 + A._35*B._51 + A._36*B._61;
                C._32 = A._30*B._02 + A._31*B._12 + A._32*B._22 + A._33*B._32 + A._34*B._42 + A._35*B._52 + A._36*B._62;
                C._33 = A._30*B._03 + A._31*B._13 + A._32*B._23 + A._33*B._33 + A._34*B._43 + A._35*B._53 + A._36*B._63;
                C._34 = A._30*B._04 + A._31*B._14 + A._32*B._24 + A._33*B._34 + A._34*B._44 + A._35*B._54 + A._36*B._64;
                C._35 = A._30*B._05 + A._31*B._15 + A._32*B._25 + A._33*B._35 + A._34*B._45 + A._35*B._55 + A._36*B._65;
                C._36 = A._30*B._06 + A._31*B._16 + A._32*B._26 + A._33*B._36 + A._34*B._46 + A._35*B._56 + A._36*B._66;
                C._40 = A._40*B._00 + A._41*B._10 + A._42*B._20 + A._43*B._30 + A._44*B._40 + A._45*B._50 + A._46*B._60;
                C._41 = A._40*B._01 + A._41*B._11 + A._42*B._21 + A._43*B._31 + A._44*B._41 + A._45*B._51 + A._46*B._61;
                C._42 = A._40*B._02 + A._41*B._12 + A._42*B._22 + A._43*B._32 + A._44*B._42 + A._45*B._52 + A._46*B._62;
                C._43 = A._40*B._03 + A._41*B._13 + A._42*B._23 + A._43*B._33 + A._44*B._43 + A._45*B._53 + A._46*B._63;
                C._44 = A._40*B._04 + A._41*B._14 + A._42*B._24 + A._43*B._34 + A._44*B._44 + A._45*B._54 + A._46*B._64;
                C._45 = A._40*B._05 + A._41*B._15 + A._42*B._25 + A._43*B._35 + A._44*B._45 + A._45*B._55 + A._46*B._65;
                C._46 = A._40*B._06 + A._41*B._16 + A._42*B._26 + A._43*B._36 + A._44*B._46 + A._45*B._56 + A._46*B._66;
                C._50 = A._50*B._00 + A._51*B._10 + A._52*B._20 + A._53*B._30 + A._54*B._40 + A._55*B._50 + A._56*B._60;
                C._51 = A._50*B._01 + A._51*B._11 + A._52*B._21 + A._53*B._31 + A._54*B._41 + A._55*B._51 + A._56*B._61;
                C._52 = A._50*B._02 + A._51*B._12 + A._52*B._22 + A._53*B._32 + A._54*B._42 + A._55*B._52 + A._56*B._62;
                C._53 = A._50*B._03 + A._51*B._13 + A._52*B._23 + A._53*B._33 + A._54*B._43 + A._55*B._53 + A._56*B._63;
                C._54 = A._50*B._04 + A._51*B._14 + A._52*B._24 + A._53*B._34 + A._54*B._44 + A._55*B._54 + A._56*B._64;
                C._55 = A._50*B._05 + A._51*B._15 + A._52*B._25 + A._53*B._35 + A._54*B._45 + A._55*B._55 + A._56*B._65;
                C._56 = A._50*B._06 + A._51*B._16 + A._52*B._26 + A._53*B._36 + A._54*B._46 + A._55*B._56 + A._56*B._66;
                C._60 = A._60*B._00 + A._61*B._10 + A._62*B._20 + A._63*B._30 + A._64*B._40 + A._65*B._50 + A._66*B._60;
                C._61 = A._60*B._01 + A._61*B._11 + A._62*B._21 + A._63*B._31 + A._64*B._41 + A._65*B._51 + A._66*B._61;
                C._62 = A._60*B._02 + A._61*B._12 + A._62*B._22 + A._63*B._32 + A._64*B._42 + A._65*B._52 + A._66*B._62;
                C._63 = A._60*B._03 + A._61*B._13 + A._62*B._23 + A._63*B._33 + A._64*B._43 + A._65*B._53 + A._66*B._63;
                C._64 = A._60*B._04 + A._61*B._14 + A._62*B._24 + A._63*B._34 + A._64*B._44 + A._65*B._54 + A._66*B._64;
                C._65 = A._60*B._05 + A._61*B._15 + A._62*B._25 + A._63*B._35 + A._64*B._45 + A._65*B._55 + A._66*B._65;
                C._66 = A._60*B._06 + A._61*B._16 + A._62*B._26 + A._63*B._36 + A._64*B._46 + A._65*B._56 + A._66*B._66;
            }

            // A = x * y
            static void outer(const Vector & x, const Vector &y, Matrix &A)
            {
                A._00 = x._0*y._0; A._01 = x._0*y._1; A._02 = x._0*y._2; A._03 = x._0*y._3; A._04 = x._0*y._4; A._05 = x._0*y._5; A._06 = x._0*y._6; 
                A._10 = x._1*y._0; A._11 = x._1*y._1; A._12 = x._1*y._2; A._13 = x._1*y._3; A._14 = x._1*y._4; A._15 = x._1*y._5; A._16 = x._1*y._6; 
                A._20 = x._2*y._0; A._21 = x._2*y._1; A._22 = x._2*y._2; A._23 = x._2*y._3; A._24 = x._2*y._4; A._25 = x._2*y._5; A._26 = x._2*y._6; 
                A._30 = x._3*y._0; A._31 = x._3*y._1; A._32 = x._3*y._2; A._33 = x._3*y._3; A._34 = x._3*y._4; A._35 = x._3*y._5; A._36 = x._3*y._6; 
                A._40 = x._4*y._0; A._41 = x._4*y._1; A._42 = x._4*y._2; A._43 = x._4*y._3; A._44 = x._4*y._4; A._45 = x._4*y._5; A._46 = x._4*y._6; 
                A._50 = x._5*y._0; A._51 = x._5*y._1; A._52 = x._5*y._2; A._53 = x._5*y._3; A._54 = x._5*y._4; A._55 = x._5*y._5; A._56 = x._5*y._6; 
                A._60 = x._6*y._0; A._61 = x._6*y._1; A._62 = x._6*y._2; A._63 = x._6*y._3; A._64 = x._6*y._4; A._65 = x._6*y._5; A._66 = x._6*y._6; 
            }

            // At = A^T
            static void trans(const Matrix & A, Matrix & At)
            {
              At._00=A._00;At._01=A._10;At._02=A._20;At._03=A._30;At._04=A._40;At._05=A._50;At._06=A._60;
              At._10=A._01;At._11=A._11;At._12=A._21;At._13=A._31;At._14=A._41;At._15=A._51;At._16=A._61;
              At._20=A._02;At._21=A._12;At._22=A._22;At._23=A._32;At._24=A._42;At._25=A._52;At._26=A._62;
              At._30=A._03;At._31=A._13;At._32=A._23;At._33=A._33;At._34=A._43;At._35=A._53;At._36=A._63;
              At._40=A._04;At._41=A._14;At._42=A._24;At._43=A._34;At._44=A._44;At._45=A._54;At._46=A._64;
              At._50=A._05;At._51=A._15;At._52=A._25;At._53=A._35;At._54=A._45;At._55=A._55;At._56=A._65;
              At._60=A._06;At._61=A._16;At._62=A._26;At._63=A._36;At._64=A._46;At._65=A._56;At._66=A._66;
            }

    };

}

