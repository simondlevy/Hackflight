package dolphin.dolphinapp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

/**
 * Created by Rob Jones on 6/23/15.
 * tutorial link: https://www3.ntu.edu.sg/home/ehchua/programming/android/Android_3D.html
 */
public class Quad {
    private FloatBuffer vertexBuffer;  // Buffer for vertex-array
    private int   numFaces          = 146;
    private float width             = 0.5f;
    private float height            = 0.5f  * width;
    private float depth             = width;
    private float hWidth            = 0.5f  * width;
    private float hHeight           = 0.5f  * height;
    private float hDepth            = 0.5f  * depth;
    private float armLength         = 2.0f  * hWidth;
    private float armWidth          = armLength/10;
    // ARROW CONSTANTS
    private float arrowWidth        = 1.0f  * armWidth;
    private float arrowLength       = 5.0f  * armWidth;
    private float arrowHeight       = 1.5f  * hHeight;
    // PROP CONSTANTS
    private float propWidth         = 1.0f  * armWidth;
    private float propNarrowWidth   = 0.2f  * propWidth;
    private float propLength        = 7.5f  * propWidth;
    private float propNarrowLength  = 0.75f * propLength;
    private float propShortLength   = 0.25f * propLength;
    //PROP PITCH CONSTANTS
    private float tipTU             = 0.9f   * hHeight;
    private float tipTL             = 0.625f * hHeight;
    private float tipBU             = 0.625f * hHeight;
    private float tipBL             = 0.350f * hHeight;

    private float endT              = 0.75f * hHeight;
    private float endB              = 0.5f  * hHeight;

    private float constant1         = ((endT - tipTL)/3) * hHeight;
    private float constant2         = ((endB-tipBL)/3)   * hHeight;

    private float farTU             = tipTU - constant2;
    private float farTL             = tipTL + constant1;
    private float farBU             = tipBU - constant1;
    private float farBL             = tipBL + constant2;

    private float closeTU           = farTU - constant2;
    private float closeTL           = farTL + constant1;
    private float closeBU           = farBU - constant1;
    private float closeBL           = farBL + constant2;

    // COLORS
    private float[] lightGrey       = {0.7f, 0.7f, 0.7f, 0.7f};
    private float[] mediumGrey      = {0.5f, 0.5f, 0.5f, 0.5f};
    private float[] darkGrey        = {0.3f, 0.3f, 0.3f, 0.3f};
    private float[] red             = {1.0f, 0.0f, 0.0f, 1.0f};
    private float[] darkRed         = {0.7f, 0.0f, 0.0f, 0.7f};
    private float[] green           = {0.0f, 1.0f, 0.0f, 1.0f};
    private float[] darkGreen       = {0.0f, 0.7f, 0.0f, 0.7f};
    private float[][] colors        = {  // Colors of the faces
            mediumGrey, // 0.
            mediumGrey, // 1.
            mediumGrey, // 2.
            mediumGrey, // 3.
            mediumGrey, // 4.
            mediumGrey, // 5.
            mediumGrey, // 6.
            mediumGrey, // 7.
            lightGrey,  // 8.
            lightGrey,  // 9.
            mediumGrey, // 10.
            darkGrey,   // 11.
            darkGrey,   // 12.
            lightGrey,  // 13.
            lightGrey,  // 14.
            mediumGrey, // 15.
            darkGrey,   // 16.
            darkGrey,   // 17.
            lightGrey,  // 18.
            lightGrey,  // 19.
            mediumGrey, // 20.
            darkGrey,   // 21.
            darkGrey,   // 22.
            lightGrey,  // 23.
            lightGrey,  // 24.
            mediumGrey, // 25.
            darkGrey,   // 26.
            darkGrey,   // 27.
            red,        // 28.
            red,        // 29.
            red,        // 30.
            red,        // 31.
            red,        // 32.
            red,        // 33.
            red,        // 34.
            red,        // 35.
            lightGrey,  // 36.
            lightGrey,  // 37.
            lightGrey,  // 38.
            lightGrey,  // 39.
            lightGrey,  // 40.
            lightGrey,  // 41.
            darkRed,    // 42.
            darkRed,    // 43.
            darkRed,    // 44.
            red,        // 45.
            red,        // 46.
            darkRed,    // 47.
            darkRed,    // 48.
            red,        // 49.
            red,        // 50.
            darkRed,    // 51.
            darkRed,    // 52.
            red,        // 53.
            red,        // 54.
            darkRed,    // 55.
            darkRed,    // 56.
            darkRed,    // 57.
            red,        // 58.
            red,        // 59.
            darkRed,    // 60.
            darkRed,    // 61.
            red,        // 62.
            red,        // 63.
            darkRed,    // 64.
            darkRed,    // 65.
            red,        // 66.
            red,        // 67.
            darkGreen,  // 68.
            darkGreen,  // 69.
            darkGreen,  // 70.
            green,      // 71.
            green,      // 72.
            darkGreen,  // 73.
            darkGreen,  // 74.
            green,      // 75.
            green,      // 76.
            darkGreen,  // 77.
            darkGreen,  // 78.
            green,      // 79.
            green,      // 80.
            darkGreen,  // 81.
            darkGreen,  // 82.
            darkGreen,  // 83.
            green,      // 84.
            green,      // 85.
            darkGreen,  // 86.
            darkGreen,  // 87.
            green,      // 88.
            green,      // 89.
            darkGreen,  // 90.
            darkGreen,  // 91.
            green,      // 92.
            green,      // 93.
            darkGreen,  // 94.
            darkGreen,  // 95.
            darkGreen,  // 96.
            green,      // 97.
            green,      // 98.
            darkGreen,  // 99.
            darkGreen,  // 100.
            green,      // 101.
            green,      // 102.
            darkGreen,  // 103.
            darkGreen,  // 104.
            green,      // 105.
            green,      // 106.
            darkGreen,  // 107.
            darkGreen,  // 108.
            darkGreen,  // 109.
            green,      // 110.
            green,      // 111.
            darkGreen,  // 112.
            darkGreen,  // 113.
            green,      // 114.
            green,      // 115.
            darkGreen,  // 116.
            darkGreen,  // 117.
            green,      // 118.
            green,      // 119.
            darkRed,    // 120.
            darkRed,    // 121.
            darkRed,    // 122.
            red,        // 123.
            red,        // 124.
            darkRed,    // 125.
            darkRed,    // 126.
            red,        // 127.
            red,        // 128.
            darkRed,    // 129.
            darkRed,    // 130.
            red,        // 131.
            red,        // 132.
            darkRed,    // 133.
            darkRed,    // 134.
            darkRed,    // 135.
            red,        // 136.
            red,        // 137.
            darkRed,    // 138.
            darkRed,    // 139.
            red,        // 140.
            red,        // 141.
            darkRed,    // 142.
            darkRed,    // 143.
            red,        // 144.
            red,        // 145.
    };

    private float[] vertices = {  // Vertices of the 6 faces
            // BACK (FACE #0)
            -hWidth + armWidth, -hHeight, hDepth + armWidth, // #10
            hWidth - armWidth, -hHeight, hDepth + armWidth, // #3
            -hWidth + armWidth, hHeight, hDepth + armWidth, //  #9
            hWidth - armWidth, hHeight, hDepth + armWidth, //  #0
            // FRONT (FACE #1)
            hWidth - armWidth, -hHeight , -hDepth - armWidth, //  #23
            -hWidth + armWidth, -hHeight , -hDepth - armWidth,//  #30
            hWidth - armWidth, hHeight , -hDepth - armWidth, //  #20
            -hWidth + armWidth, hHeight , -hDepth - armWidth,//  #29
            // LEFT (FACE #2)
            -hWidth - armWidth, -hHeight , -hDepth + armWidth,//  #31
            -hWidth - armWidth, -hHeight, hDepth - armWidth, //  #11
            -hWidth - armWidth, hHeight , -hDepth + armWidth, //  #28
            -hWidth - armWidth, hHeight, hDepth - armWidth, //  #8
            // RIGHT (FACE #3)
            hWidth + armWidth, -hHeight, hDepth - armWidth, //  #2
            hWidth + armWidth, -hHeight, -hDepth + armWidth, //  #22
            hWidth + armWidth, +hHeight, hDepth - armWidth, //  #1
            hWidth + armWidth, hHeight, -hDepth + armWidth, //  #21
            // BODY BACK RIGHT CORNER (FACE #4)
            hWidth - armWidth, -hHeight, hDepth + armWidth, // #3
            hWidth + armWidth, -hHeight, hDepth - armWidth, // #2
            hWidth - armWidth, hHeight, hDepth + armWidth, // #0
            hWidth + armWidth, +hHeight, hDepth - armWidth, // #1
            // BODY BACK LEFT CORNER (FACE #5)
            -hWidth - armWidth, -hHeight, hDepth - armWidth, // #11
            -hWidth + armWidth, -hHeight, hDepth + armWidth, // #10
            -hWidth - armWidth, hHeight, hDepth - armWidth, // #8
            -hWidth + armWidth, hHeight, hDepth + armWidth, // #9
            // BODY FRONT RIGHT CORNER (FACE #6)
            hWidth + armWidth, -hHeight, -hDepth + armWidth, // #22
            hWidth - armWidth, -hHeight , -hDepth - armWidth, // #23
            hWidth + armWidth, hHeight, -hDepth + armWidth, // #21
            hWidth - armWidth, hHeight , -hDepth - armWidth, // #20
            // BODY FRONT LEFT CORNER (FACE #7)
            -hWidth + armWidth, -hHeight , -hDepth - armWidth, // #30
            -hWidth - armWidth, -hHeight , -hDepth + armWidth, // #31
            -hWidth + armWidth, hHeight , -hDepth - armWidth,// #29
            -hWidth - armWidth, hHeight , -hDepth + armWidth, // #28
            // BACK RIGHT ARM FRONT FACE (FACE #8)
            hWidth - armWidth, -hHeight/2, hDepth + armWidth, //  #3
            hWidth + armLength - armWidth, -hHeight/2, hDepth + armLength + armWidth, //  #4
            hWidth - armWidth, hHeight/2, hDepth + armWidth, //  #0
            hWidth + armLength - armWidth, hHeight/2, hDepth + armLength + armWidth, //  #7
            // BACK RIGHT ARM BACK FACE (FACE #9)
            hWidth + armLength + armWidth, -hHeight/2, hDepth + armLength - armWidth, // #6
            hWidth + armWidth, -hHeight/2, hDepth - armWidth, //  #2
            hWidth + armLength + armWidth, +hHeight/2, hDepth + armLength - armWidth, // #5
            hWidth + armWidth, +hHeight/2, hDepth - armWidth, //  #1
            // BACK RIGHT ARM END (FACE #10)
            hWidth + armLength - armWidth, -hHeight/2, hDepth + armLength + armWidth, // #4
            hWidth + armLength + armWidth, -hHeight/2, hDepth + armLength - armWidth, // #6
            hWidth + armLength - armWidth, hHeight/2, hDepth + armLength + armWidth, //  #7
            hWidth + armLength + armWidth, +hHeight/2, hDepth + armLength - armWidth, // #5
            // BACK RIGHT ARM TOP (FACE #11)
            hWidth - armWidth, hHeight/2, hDepth + armWidth, //  #0
            hWidth + armLength - armWidth, hHeight/2, hDepth + armLength + armWidth, //  #7
            hWidth + armWidth, hHeight/2, hDepth - armWidth,  //  #1
            hWidth + armLength + armWidth, hHeight/2, hDepth + armLength - armWidth, // #5
            // BACK RIGHT ARM BOTTOM (FACE #12)
            hWidth + armWidth, -hHeight/2, hDepth - armWidth, //  #2
            hWidth + armLength + armWidth, -hHeight/2, hDepth + armLength - armWidth, //  #6
            hWidth - armWidth, -hHeight/2, hDepth + armWidth, //  #3
            hWidth + armLength - armWidth, -hHeight/2, hDepth + armLength + armWidth, //  #4
            // BACK LEFT ARM BACK FACE (FACE #13)
            -hWidth - armLength + armWidth, -hHeight/2, hDepth + armLength + armWidth, //  #14
            -hWidth + armWidth, -hHeight/2, hDepth + armWidth, //  #10
            -hWidth - armLength + armWidth, hHeight/2, hDepth + armLength + armWidth, //  #13
            -hWidth + armWidth, hHeight/2, hDepth + armWidth, //  #9
            // BACK LEFT ARM FRONT FACE (FACE #14)
            -hWidth - armWidth, -hHeight/2, hDepth - armWidth, //  #11
            -hWidth - armLength - armWidth, -hHeight/2, hDepth + armLength - armWidth, //  #15
            -hWidth - armWidth, hHeight/2, hDepth - armWidth, //  #8
            -hWidth - armLength - armWidth, hHeight/2, hDepth + armLength - armWidth, //  #12
            // BACK LEFT ARM END (FACE #15)
            -hWidth - armLength - armWidth, -hHeight/2, hDepth + armLength - armWidth, //  #15
            -hWidth - armLength + armWidth, -hHeight/2, hDepth + armLength + armWidth, //  #14
            -hWidth - armLength - armWidth, hHeight/2, hDepth + armLength - armWidth, //  #12
            -hWidth - armLength + armWidth, hHeight/2, hDepth + armLength + armWidth, //  #13
            // BACK LEFT ARM TOP (FACE #16)
            -hWidth - armLength + armWidth, hHeight/2, hDepth + armLength + armWidth, //  #13
            -hWidth + armWidth, hHeight/2, hDepth + armWidth, //  #9
            -hWidth - armLength - armWidth, hHeight/2, hDepth + armLength - armWidth, //  #12
            -hWidth - armWidth, hHeight/2, hDepth - armWidth, //  #8
            // BACK LEFT ARM BOTTOM (FACE #17)
            -hWidth - armLength - armWidth, -hHeight/2, hDepth + armLength - armWidth, //  #15
            -hWidth - armWidth, -hHeight/2, hDepth - armWidth, //  #11
            -hWidth - armLength + armWidth, -hHeight/2, hDepth + armLength + armWidth, //  #14
            -hWidth + armWidth, -hHeight/2, hDepth + armWidth, //  #10
            // FRONT RIGHT ARM BACK FACE (FACE #18)
            hWidth + armWidth, -hHeight/2, -hDepth + armWidth, //  #22
            hWidth + armLength + armWidth, -hHeight/2, -hDepth - armLength + armWidth, //  #18
            hWidth + armWidth, hHeight/2, -hDepth + armWidth, //  #21
            hWidth + armLength + armWidth, +hHeight/2, -hDepth - armLength + armWidth, //  #17
            // FRONT RIGHT ARM FRONT FACE (FACE #19)
            hWidth + armLength - armWidth, -hHeight/2 , -hDepth - armLength - armWidth, //  #19
            hWidth - armWidth, -hHeight/2 , -hDepth - armWidth, //  #23
            hWidth + armLength - armWidth, hHeight/2 , -hDepth - armLength - armWidth, //  #16
            hWidth - armWidth, hHeight/2 , -hDepth - armWidth, //  #20
            // FRONT RIGHT ARM END (FACE #20)
            hWidth + armLength + armWidth, -hHeight/2, -hDepth - armLength + armWidth, //  #18
            hWidth + armLength - armWidth, -hHeight/2 , -hDepth - armLength - armWidth, //  #19
            hWidth + armLength + armWidth, +hHeight/2, -hDepth - armLength + armWidth, //  #17
            hWidth + armLength - armWidth, hHeight/2 , -hDepth - armLength - armWidth, //  #16
            // FRONT RIGHT ARM TOP (FACE #21)
            hWidth + armWidth, hHeight/2, -hDepth + armWidth, //  #21
            hWidth + armLength + armWidth, +hHeight/2, -hDepth - armLength + armWidth, //  #17
            hWidth - armWidth, hHeight/2 , -hDepth - armWidth, //  #20
            hWidth + armLength - armWidth, hHeight/2 , -hDepth - armLength - armWidth, //  #16
            // FRONT RIGHT ARM BOTTOM (FACE #22)
            hWidth - armWidth, -hHeight/2 , -hDepth - armWidth, //  #23
            hWidth + armLength - armWidth, -hHeight/2 , -hDepth - armLength - armWidth, //  #19
            hWidth + armWidth, -hHeight/2, -hDepth + armWidth, //  #22
            hWidth + armLength + armWidth, -hHeight/2, -hDepth - armLength + armWidth, //  #18
            // FRONT LEFT ARM BACK (FACE #23)
            -hWidth - armLength - armWidth, -hHeight/2 , -hDepth - armLength + armWidth, //  #27
            -hWidth - armWidth, -hHeight/2 , -hDepth + armWidth,//  #31
            -hWidth - armLength - armWidth, hHeight/2 , -hDepth - armLength + armWidth,//  #24
            -hWidth - armWidth, hHeight/2 , -hDepth + armWidth, //  #28
            // FRONT LEFT ARM FRONT (FACE #24)
            -hWidth + armWidth, -hHeight/2 , -hDepth - armWidth,//  #30
            -hWidth - armLength + armWidth, -hHeight/2 , -hDepth - armLength - armWidth,//  #26
            -hWidth + armWidth, hHeight/2 , -hDepth - armWidth,//  #29
            -hWidth - armLength + armWidth, hHeight/2 , -hDepth - armLength - armWidth,//  #25
            // FRONT LEFT ARM END (FACE #25)
            -hWidth - armLength + armWidth, -hHeight/2 , -hDepth - armLength - armWidth,//  #26
            -hWidth - armLength - armWidth, -hHeight/2 , -hDepth - armLength + armWidth, //  #27
            -hWidth - armLength + armWidth, hHeight/2 , -hDepth - armLength - armWidth,//  #25
            -hWidth - armLength - armWidth, hHeight/2 , -hDepth - armLength + armWidth,//  #24
            // FRONT LEFT ARM TOP (FACE #26)
            -hWidth - armLength - armWidth, hHeight/2 , -hDepth - armLength + armWidth,//  #24
            -hWidth - armWidth, hHeight/2 , -hDepth + armWidth, //  #28
            -hWidth - armLength + armWidth, hHeight/2 , -hDepth - armLength - armWidth,//  #25
            -hWidth + armWidth, hHeight/2 , -hDepth - armWidth,//  #29
            // FRONT LEFT ARM BOTTOM (FACE #27)
            -hWidth - armLength + armWidth, -hHeight/2 , -hDepth - armLength - armWidth,//  #26
            -hWidth + armWidth, -hHeight/2 , -hDepth - armWidth,//  #30
            -hWidth - armLength - armWidth, -hHeight/2 , -hDepth - armLength + armWidth, //  #27
            -hWidth - armWidth, -hHeight/2 , -hDepth + armWidth,//  #31
            // ARROW BODY BACK (FACE #28)
            -arrowWidth, hHeight, arrowLength, // #39
            arrowWidth, hHeight, arrowLength, // #38
            -arrowWidth, arrowHeight, arrowLength, // #36
            arrowWidth, arrowHeight, arrowLength, // #37
            // ARROW BODY LEFT (FACE #29)
            -arrowWidth, hHeight, 0.0f, // #35
            -arrowWidth, hHeight, arrowLength, // #39
            -arrowWidth, arrowHeight, 0.0f, // #32
            -arrowWidth, arrowHeight, arrowLength, // #36
            // ARROW BODY RIGHT (FACE #30)
            arrowWidth, hHeight, arrowLength, // #38
            arrowWidth, hHeight, 0.0f, // #34
            arrowWidth, arrowHeight, arrowLength, // #37
            arrowWidth,arrowHeight, 0.0f, // #33
            // ARROW BODY TOP (FACE #31)
            -arrowWidth, arrowHeight, arrowLength, // #36
            arrowWidth, arrowHeight, arrowLength, // #37
            -arrowWidth, arrowHeight, 0.0f, // #32
            arrowWidth,arrowHeight, 0.0f, // #33
            // ARROW HEAD BACK (FACE #32)
            -arrowWidth - 2*arrowWidth, hHeight, 0.0f, // #47
            arrowWidth + 2*arrowWidth, hHeight, 0.0f, // #46
            -arrowWidth - 2*arrowWidth, arrowHeight, 0.0f, // #44
            arrowWidth + 2*arrowWidth, arrowHeight, 0.0f, // #45
            // ARROW HEAD LEFT (FACE #33)
            0.0f, hHeight, -arrowLength, // #43
            -arrowWidth - 2*arrowWidth, hHeight, 0.0f, // #47
            0.0f, arrowHeight, -arrowLength, // #40
            -arrowWidth - 2*arrowWidth, arrowHeight, 0.0f, // #44
            // ARROW HEAD RIGHT (FACE #34)
            arrowWidth + 2*arrowWidth, hHeight, 0.0f, // #46
            0.0f, hHeight, -arrowLength, // #42
            arrowWidth + 2*arrowWidth, arrowHeight, 0.0f, // #45
            0.0f, arrowHeight, -arrowLength, // #41
            // ARROW HEAD TOP (FACE #35)
            -arrowWidth - 2*arrowWidth, arrowHeight, 0.0f, // #44
            arrowWidth + 2*arrowWidth, arrowHeight, 0.0f, // #45
            0.0f, arrowHeight, -arrowLength, // #40
            0.0f, arrowHeight, -arrowLength, // #41
            // TOP BACK SECTOR (FACE #36)
            -hWidth + armWidth, hHeight, hDepth + armWidth, //  #9
            hWidth - armWidth, hHeight, hDepth + armWidth, //  #0
            -hWidth - armWidth, hHeight, hDepth - armWidth, //  #8
            hWidth + armWidth, +hHeight, hDepth - armWidth, //  #1
            // BODY TOP (FACE #37)
            -hWidth - armWidth, hHeight, hDepth - armWidth, // #8
            hWidth + armWidth, +hHeight, hDepth - armWidth, // #1
            -hWidth - armWidth, hHeight , -hDepth + armWidth, // #28
            hWidth + armWidth, hHeight, -hDepth + armWidth, // #21
            // TOP FRONT SECTOR (FACE #38)
            -hWidth - armWidth, hHeight , -hDepth + armWidth, // #28
            hWidth + armWidth, hHeight, -hDepth + armWidth, // #21
            -hWidth + armWidth, hHeight , -hDepth - armWidth, // #29
            hWidth - armWidth, hHeight , -hDepth - armWidth, // #20
            // BOTTOM BACK SECTOR (FACE #39)
            hWidth - armWidth, -hHeight, hDepth + armWidth, //  #3
            -hWidth + armWidth, -hHeight, hDepth + armWidth, //  #10
            hWidth + armWidth, -hHeight, hDepth - armWidth, //  #2
            -hWidth - armWidth, -hHeight, hDepth - armWidth, //  #11
            // BODY BOTTOM (FACE #40)
            -hWidth - armWidth, -hHeight , -hDepth + armWidth,// #31
            hWidth + armWidth, -hHeight, -hDepth + armWidth, // #22
            -hWidth - armWidth, -hHeight, hDepth - armWidth, // #11
            hWidth + armWidth, -hHeight, hDepth - armWidth, // #2
            // BOTTOM FRONT SECTOR (FACE #41)
            hWidth + armWidth, -hHeight, -hDepth + armWidth, // #22
            -hWidth - armWidth, -hHeight , -hDepth + armWidth,// #31
            hWidth - armWidth, -hHeight , -hDepth - armWidth, // #23
            -hWidth + armWidth, -hHeight , -hDepth - armWidth,// #30
            // BACK RIGHT PROPELLER NORTHEAST NARROW TIP (FACE #42)
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, +hDepth+armLength - propLength + propNarrowWidth, // #66
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, +hDepth+armLength - propLength - propNarrowWidth, // #67
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, +hDepth+armLength - propLength + propNarrowWidth, // #65
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, +hDepth+armLength - propLength - propNarrowWidth, // #64
            // BACK RIGHT PROPELLER NORTH FACING FLAT SIDE ON TIP (FACE #43)
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, +hDepth+armLength - propLength - propNarrowWidth, // #67
            hWidth+armLength + propNarrowLength - propWidth, +farBL, +hDepth+armLength - propNarrowLength - propWidth, // #71
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, +hDepth+armLength - propLength - propNarrowWidth, // #64
            hWidth+armLength + propNarrowLength - propWidth, +farTL, +hDepth+armLength - propNarrowLength - propWidth, // #68
            // BACK RIGHT PROPELLER EAST FACING FLAT SIDE ON TIP (FACE #44)
            hWidth+armLength + propNarrowLength + propWidth, +farBU, +hDepth+armLength - propNarrowLength + propWidth, // #70
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, +hDepth+armLength - propLength + propNarrowWidth, // #66
            hWidth+armLength + propNarrowLength + propWidth, +farTU, +hDepth+armLength - propNarrowLength + propWidth, // #69
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, +hDepth+armLength - propLength + propNarrowWidth, // #65
            // BACK RIGHT PROPELLER NORTHEAST TOP FRONT SECTOR (FACE #45)
            hWidth+armLength + propNarrowLength - propWidth, +farTL, +hDepth+armLength - propNarrowLength - propWidth, // #68
            hWidth+armLength + propNarrowLength + propWidth, +farTU, +hDepth+armLength - propNarrowLength + propWidth, // #69
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, +hDepth+armLength - propLength - propNarrowWidth, // #64
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, +hDepth+armLength - propLength + propNarrowWidth, // #65
            // BACK RIGHT PROPELLER NORTHEAST BOTTOM FRONT SECTOR (FACE #46)
            hWidth+armLength + propNarrowLength + propWidth, +farBU, +hDepth+armLength - propNarrowLength + propWidth, // #70
            hWidth+armLength + propNarrowLength - propWidth, +farBL, +hDepth+armLength - propNarrowLength - propWidth, // #71
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, +hDepth+armLength - propLength + propNarrowWidth, // #66
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, +hDepth+armLength - propLength - propNarrowWidth, // #67
            // BACK RIGHT PROPELLER NORTHEAST FRONT SIDE (FACE #47)
            hWidth+armLength + propNarrowLength - propWidth, +farBL, +hDepth+armLength - propNarrowLength - propWidth, // #71
            hWidth+armLength + propShortLength - propWidth, +farBL, +hDepth+armLength - propShortLength - propWidth, // #75
            hWidth+armLength + propNarrowLength - propWidth, +farTL, +hDepth+armLength - propNarrowLength - propWidth, // #68
            hWidth+armLength + propShortLength - propWidth, +closeTL, +hDepth+armLength - propShortLength - propWidth, // #72
            // BACK RIGHT PROPELLER NORTHEAST BACK SIDE (FACE #48)
            hWidth+armLength + propShortLength + propWidth, +farBU, +hDepth+armLength - propShortLength + propWidth, // #74
            hWidth+armLength + propNarrowLength + propWidth, +farBU, +hDepth+armLength - propNarrowLength + propWidth, // #70
            hWidth+armLength + propShortLength + propWidth, +closeTU, +hDepth+armLength - propShortLength + propWidth, // #73
            hWidth+armLength + propNarrowLength + propWidth, +farTU, +hDepth+armLength - propNarrowLength + propWidth, // #69
            // BACK RIGHT PROPELLER NORTHEAST TOP MAIN SECTOR (FACE #49)
            hWidth+armLength + propShortLength - propWidth, +closeTL, +hDepth+armLength - propShortLength - propWidth, // #72
            hWidth+armLength + propShortLength + propWidth, +closeTU, +hDepth+armLength - propShortLength + propWidth, // #73
            hWidth+armLength + propNarrowLength - propWidth, +farTL, +hDepth+armLength - propNarrowLength - propWidth, // #68
            hWidth+armLength + propNarrowLength + propWidth, +farTU, +hDepth+armLength - propNarrowLength + propWidth, // #69
            // BACK RIGHT PROPELLER NORTHEAST BOTTOM MAIN SECTOR (FACE #50)
            hWidth+armLength + propShortLength + propWidth, +farBU, +hDepth+armLength - propShortLength + propWidth, // #74
            hWidth+armLength + propShortLength - propWidth, +farBL, +hDepth+armLength - propShortLength - propWidth, // #75
            hWidth+armLength + propNarrowLength + propWidth, +farBU, +hDepth+armLength - propNarrowLength + propWidth, // #70
            hWidth+armLength + propNarrowLength - propWidth, +farBL, +hDepth+armLength - propNarrowLength - propWidth, // #71
            // BACK RIGHT NORTHEAST PROPELLER SOUTH FACING SIDE (FACE #51)
            hWidth+armLength + propNarrowWidth, +endB, +hDepth+armLength + propNarrowWidth, // #78
            hWidth+armLength + propShortLength + propWidth, +farBU, +hDepth+armLength - propShortLength + propWidth, // #74
            hWidth+armLength + propNarrowWidth, +endT, +hDepth+armLength + propNarrowWidth, // #77
            hWidth+armLength + propShortLength + propWidth, +closeTU, +hDepth+armLength - propShortLength + propWidth, // #73
            // BACK RIGHT NORTHEAST PROPELLER WEST FACING SIDE (FACE #52)
            hWidth+armLength + propShortLength - propWidth, +farBL, +hDepth+armLength - propShortLength - propWidth, // #75
            hWidth+armLength - propNarrowWidth, +endB, +hDepth+armLength - propNarrowWidth, // #79
            hWidth+armLength + propShortLength - propWidth, +closeTL, +hDepth+armLength - propShortLength - propWidth, // #72
            hWidth+armLength - propNarrowWidth, +endT, +hDepth+armLength - propNarrowWidth, // #76
            // BACK RIGHT NORTHEAST PROPELLER TOP BACK SECTOR (FACE #53)
            hWidth+armLength - propNarrowWidth, +endT, +hDepth+armLength - propNarrowWidth, // #76
            hWidth+armLength + propNarrowWidth, +endT, +hDepth+armLength + propNarrowWidth, // #77
            hWidth+armLength + propShortLength - propWidth, +closeTL, +hDepth+armLength - propShortLength - propWidth, // #72
            hWidth+armLength + propShortLength + propWidth, +closeTU, +hDepth+armLength - propShortLength + propWidth, // #73
            // BACK RIGHT NORTHEAST PROPELLER BOTTOM BACK SECTOR (FACE #54)
            hWidth+armLength + propNarrowWidth, +endB, +hDepth+armLength + propNarrowWidth, // #78
            hWidth+armLength - propNarrowWidth, +endB, +hDepth+armLength - propNarrowWidth, // #79
            hWidth+armLength + propShortLength + propWidth, +farBU, +hDepth+armLength - propShortLength + propWidth, // #74
            hWidth+armLength + propShortLength - propWidth, +farBL, +hDepth+armLength - propShortLength - propWidth, // #75
            // BACK RIGHT SOUTHWEST PROPELLER NARROW TIP (FACE #55)
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, +hDepth+armLength + propLength - propNarrowWidth, // #91
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, +hDepth+armLength + propLength + propNarrowWidth, // #90
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, +hDepth+armLength + propLength - propNarrowWidth, // #88
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, +hDepth+armLength + propLength + propNarrowWidth, // #89
            // BACK RIGHT SOUTHWEST PROPELLER SOUTH FACING FLAT SIDE (FACE #56)
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, +hDepth+armLength + propLength + propNarrowWidth, // #90
            hWidth+armLength - propNarrowLength + propWidth, +farBL, +hDepth+armLength + propNarrowLength + propWidth, // #86
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, +hDepth+armLength + propLength + propNarrowWidth, // #89
            hWidth+armLength - propNarrowLength + propWidth, +farTL, +hDepth+armLength + propNarrowLength + propWidth, // #85
            // BACK RIGHT SOUTHWEST PROPELLER WEST FACING FLAT SIDE (FACE #57)
            hWidth+armLength - propNarrowLength - propWidth, +farBU, +hDepth+armLength + propNarrowLength - propWidth, // #87
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, +hDepth+armLength + propLength - propNarrowWidth, // #91
            hWidth+armLength - propNarrowLength - propWidth, +farTU, +hDepth+armLength + propNarrowLength - propWidth, // #84
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, +hDepth+armLength + propLength - propNarrowWidth, // #88
            // BACK RIGHT SOUTHWEST PROPELLER TOP BACK SECTION (FACE #58)
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, +hDepth+armLength + propLength - propNarrowWidth, // #88
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, +hDepth+armLength + propLength + propNarrowWidth, // #89
            hWidth+armLength - propNarrowLength - propWidth, +farTU, +hDepth+armLength + propNarrowLength - propWidth, // #84
            hWidth+armLength - propNarrowLength + propWidth, +farTL, +hDepth+armLength + propNarrowLength + propWidth, // #85
            // BACK RIGHT SOUTHWEST PROPELLER BOTTOM BACK SECTION (FACE #59)
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, +hDepth+armLength + propLength + propNarrowWidth, // #90
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, +hDepth+armLength + propLength - propNarrowWidth, // #91
            hWidth+armLength - propNarrowLength + propWidth, +farBL, +hDepth+armLength + propNarrowLength + propWidth, // #86
            hWidth+armLength - propNarrowLength - propWidth, +farBU, +hDepth+armLength + propNarrowLength - propWidth, // #87
            // BACK RIGHT SOUTHWEST PROPELLER FRONT SIDE (FACE #60)
            hWidth+armLength - propShortLength - propWidth, +closeBU, +hDepth+armLength + propShortLength - propWidth, // #83
            hWidth+armLength - propNarrowLength - propWidth, +farBU, +hDepth+armLength + propNarrowLength - propWidth, // #87
            hWidth+armLength - propShortLength - propWidth, +closeTU, +hDepth+armLength + propShortLength - propWidth, // #80
            hWidth+armLength - propNarrowLength - propWidth, +farTU, +hDepth+armLength + propNarrowLength - propWidth, // #84
            // BACK RIGHT SOUTHWEST PROPELLER BACK SIDE (FACE #61)
            hWidth+armLength - propNarrowLength + propWidth, +farBL, +hDepth+armLength + propNarrowLength + propWidth, // #86
            hWidth+armLength - propShortLength + propWidth, +closeBL, +hDepth+armLength + propShortLength + propWidth, // #82
            hWidth+armLength - propNarrowLength + propWidth, +farTL, +hDepth+armLength + propNarrowLength + propWidth, // #85
            hWidth+armLength - propShortLength + propWidth, +closeTL, +hDepth+armLength + propShortLength + propWidth, // #81
            // BACK RIGHT SOUTHWEST PROPELLER TOP MAIN SECTION (FACE #62)
            hWidth+armLength - propNarrowLength - propWidth, +farTU, +hDepth+armLength + propNarrowLength - propWidth, // #84
            hWidth+armLength - propNarrowLength + propWidth, +farTL, +hDepth+armLength + propNarrowLength + propWidth, // #85
            hWidth+armLength - propShortLength - propWidth, +closeTU, +hDepth+armLength + propShortLength - propWidth, // #80
            hWidth+armLength - propShortLength + propWidth, +closeTL, +hDepth+armLength + propShortLength + propWidth, // #81
            // BACK RIGHT SOUTHWEST PROPELLER BOTTOM MAIN SECTION (FACE #63)
            hWidth+armLength - propNarrowLength + propWidth, +farBL, +hDepth+armLength + propNarrowLength + propWidth, // #86
            hWidth+armLength - propNarrowLength - propWidth, +farBU, +hDepth+armLength + propNarrowLength - propWidth, // #87
            hWidth+armLength - propShortLength + propWidth, +closeBL, +hDepth+armLength + propShortLength + propWidth, // #82
            hWidth+armLength - propShortLength - propWidth, +closeBU, +hDepth+armLength + propShortLength - propWidth, // #83
            // BACK RIGHT SOUTHWEST PROPELLER NORTH FACING SIDE (FACE #64)
            hWidth+armLength - propNarrowWidth, +endB, +hDepth+armLength - propNarrowWidth, // #79
            hWidth+armLength - propShortLength - propWidth, +closeBU, +hDepth+armLength + propShortLength - propWidth, // #83
            hWidth+armLength - propNarrowWidth, +endT, +hDepth+armLength - propNarrowWidth, // #76
            hWidth+armLength - propShortLength - propWidth, +closeTU, +hDepth+armLength + propShortLength - propWidth, // #80
            // BACK RIGHT SOUTHWEST PROPELLER EAST FACING SIDE (FACE #65)
            hWidth+armLength - propShortLength + propWidth, +closeBL, +hDepth+armLength + propShortLength + propWidth, // #82
            hWidth+armLength + propNarrowWidth, +endB, +hDepth+armLength + propNarrowWidth, // #78
            hWidth+armLength - propShortLength + propWidth, +closeTL, +hDepth+armLength + propShortLength + propWidth, // #81
            hWidth+armLength + propNarrowWidth, +endT, +hDepth+armLength + propNarrowWidth, // #77
            // BACK RIGHT SOUTHWEST PROPELLER TOP FRONT SECTOR (FACE #66)
            hWidth+armLength - propShortLength - propWidth, +closeTU, +hDepth+armLength + propShortLength - propWidth, // #80
            hWidth+armLength - propShortLength + propWidth, +closeTL, +hDepth+armLength + propShortLength + propWidth, // #81
            hWidth+armLength - propNarrowWidth, +endT, +hDepth+armLength - propNarrowWidth, // #76
            hWidth+armLength + propNarrowWidth, +endT, +hDepth+armLength + propNarrowWidth, // #77
            // BACK RIGHT SOUTHWEST PROPELLER BOTTOM FRONT SECTOR (FACE #67)
            hWidth+armLength - propShortLength + propWidth, +closeBL, +hDepth+armLength + propShortLength + propWidth, // #82
            hWidth+armLength - propShortLength - propWidth, +closeBU, +hDepth+armLength + propShortLength - propWidth, // #83
            hWidth+armLength + propNarrowWidth, +endB, +hDepth+armLength + propNarrowWidth, // #78
            hWidth+armLength - propNarrowWidth, +endB, +hDepth+armLength - propNarrowWidth, // #79


            //FRONT LEFT PROPELLER
            // FRONT LEFT PROPELLER NORTHEAST NARROW TIP (FACE #68)
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #94
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #95
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #93
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #92
            // FRONT LEFT PROPELLER NORTH FLAT FACING SIDE ON TIP (FACE #69)
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #95
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #99
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #92
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #96
            // FRONT LEFT PROPELLER EAST FACING FLAT SIDE ON TIP (FACE #70)
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #98
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #94
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #97
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #93
            // FRONT LEFT PROPELLER NORTHEAST TOP FRONT SECTOR (FACE #71)
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #96
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #97
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #92
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #93
            // FRONT LEFT PROPELLER NORTHEAST BOTTOM FRONT SECTOR (FACE #72)
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #98
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #99
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #94
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #95
            // FRONT LEFT PROPELLER NORTHEAST FRONT SIDE (FACE #73)
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #99
            -hWidth-armLength + propShortLength - propWidth, +farBL, -hDepth-armLength - propShortLength - propWidth, // #103
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #96
            -hWidth-armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #100
            // FRONT LEFT PROPELLER NORTHEAST BACK SIDE (FACE #74)
            -hWidth-armLength + propShortLength + propWidth, +farBU, -hDepth-armLength - propShortLength + propWidth, // #102
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #98
            -hWidth-armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #101
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #97
            // FRONT LEFT PROPELLER NORTHEAST TOP MAIN SECTOR (FACE #75)
            -hWidth-armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #100
            -hWidth-armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #101
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #96
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #97
             // FRONT LEFT PROPELLER NORTHEAST BOTTOM MAIN SECTOR (FACE #76)
            -hWidth-armLength + propShortLength + propWidth, +farBU, -hDepth-armLength - propShortLength + propWidth, // #102
            -hWidth-armLength + propShortLength - propWidth, +farBL, -hDepth-armLength - propShortLength - propWidth, // #103
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #98
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #99
            // FRONT LEFT NORTHEAST PROPELLER SOUTH FACING SIDE (FACE #77)
            -hWidth-armLength + propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #106
            -hWidth-armLength + propShortLength + propWidth, +farBU, -hDepth-armLength - propShortLength + propWidth, // #102?
            -hWidth-armLength + propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, // #105
            -hWidth-armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #101
            // FRONT LEFT NORTHEAST PROPELLER WEST FACING SIDE (FACE #78)
            -hWidth-armLength + propShortLength - propWidth, +farBL, -hDepth-armLength - propShortLength - propWidth, // #103?
            -hWidth-armLength - propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #107
            -hWidth-armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #100
            -hWidth-armLength - propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #104
            // FRONT LEFT NORTHEAST PROPELLER TOP BACK SECTOR (FACE #79)
            -hWidth-armLength - propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #104
            -hWidth-armLength + propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, // #105
            -hWidth-armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #100
            -hWidth-armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #101
            // FRONT LEFT NORTHEAST PROPELLER BOTTOM BACK SECTOR (FACE #80)
            -hWidth-armLength + propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #106
            -hWidth-armLength - propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #107
            -hWidth-armLength + propShortLength + propWidth, +farBU, -hDepth-armLength - propShortLength + propWidth, // #102
            -hWidth-armLength + propShortLength - propWidth, +farBL, -hDepth-armLength - propShortLength - propWidth, // #103
            // FRONT LEFT SOUTHWEST PROPELLER NARROW TIP (FACE #81)
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #119
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #118
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #116
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #117
            // FRONT LEFT SOUTHWEST PROPELLER SOUTH FACING FLAT SIDE (FACE #82)
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #118
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #114
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #117
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #113
            // FRONT LEFT SOUTHWEST PROPELLER WEST FACING FLAT SIDE (FACE #83)
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #115
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #119
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #112
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #116
            // FRONT LEFT SOUTHWEST PROPELLER TOP BACK SECTION (FACE #84)
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #116
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #117
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #112
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #113
            // FRONT LEFT SOUTHWEST PROPELLER BOTTOM BACK SECTION (FACE #85)
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #118
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #119
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #114
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #115
            // FRONT LEFT SOUTHWEST PROPELLER FRONT SIDE (FACE #86)
            -hWidth-armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #111
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #115
            -hWidth-armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #108
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #112
            // FRONT LEFT SOUTHWEST PROPELLER BACK SIDE (FACE #87)
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #114
            -hWidth-armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #110
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #113
            -hWidth-armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #109
            // FRONT LEFT SOUTHWEST PROPELLER TOP MAIN SECTION (FACE #88)
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #112
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #113
            -hWidth-armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #108
            -hWidth-armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #109
            // FRONT LEFT SOUTHWEST PROPELLER BOTTOM MAIN SECTION (FACE #89)
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #114
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #115
            -hWidth-armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #110
            -hWidth-armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #111
            // FRONT LEFT SOUTHWEST PROPELLER NORTH FACING SIDE (FACE #90)
            -hWidth-armLength - propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #107
            -hWidth-armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #111
            -hWidth-armLength - propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #104
            -hWidth-armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #108
            // FRONT LEFT SOUTHWEST PROPELLER EAST FACING SIDE (FACE #91)
            -hWidth-armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #110
            -hWidth-armLength + propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #106
            -hWidth-armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #109
            -hWidth-armLength + propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, // #105
            // FRONT LEFT SOUTHWEST PROPELLER TOP FRONT SECTOR (FACE #92)
            -hWidth-armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #108
            -hWidth-armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #109
            -hWidth-armLength - propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #104
            -hWidth-armLength + propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, // #105
            // FRONT LEFT SOUTHWEST PROPELLER BOTTOM FRONT SECTOR (FACE #93)
            -hWidth-armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #110
            -hWidth-armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #111
            -hWidth-armLength + propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #106
            -hWidth-armLength - propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #107

            // FRONT RIGHT PROPELLER
            // FRONT RIGHT NORTHWEST PROPELLER NARROW TIP (FACE #94)
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #122
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #123
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #121
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #120
            // FRONT RIGHT NORTHWEST PROPELLER NORTH SIDE (FACE #95)
            hWidth+armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #126
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #122
            hWidth+armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #125
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #121
            // FRONT RIGHT NORTHWEST PROPELLER WEST SIDE (FACE #96)
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #123
            hWidth+armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #127
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #120
            hWidth+armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #124
            // FRONT RIGHT NORTHWEST PROPELLER TOP FRONT SECTION (FACE #97)
            hWidth+armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #124
            hWidth+armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #125
            hWidth+armLength - propLength - propNarrowWidth, +tipTU, -hDepth-armLength - propLength + propNarrowWidth, // #120
            hWidth+armLength - propLength + propNarrowWidth, +tipTL, -hDepth-armLength - propLength - propNarrowWidth, // #121
            // FRONT RIGHT NORTHWEST PROPELLER BOTTOM FRONT SECTION (FACE #98)
            hWidth+armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #126
            hWidth+armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #127
            hWidth+armLength - propLength + propNarrowWidth, +tipBL, -hDepth-armLength - propLength - propNarrowWidth, // #122
            hWidth+armLength - propLength - propNarrowWidth, +tipBU, -hDepth-armLength - propLength + propNarrowWidth, // #123
            // FRONT RIGHT NORTHWEST PROPELLER FRONT (FACE #99)
            hWidth+armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength - propShortLength - propWidth, // #130
            hWidth+armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #126
            hWidth+armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #129
            hWidth+armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #125
            // FRONT RIGHT NORTHWEST PROPELLER BACK (FACE #100)
            hWidth+armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #127
            hWidth+armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength - propShortLength + propWidth, // #131
            hWidth+armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #124
            hWidth+armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #128
            // FRONT RIGHT NORTHWEST PROPELLER TOP MAIN SECTION (FACE #101)
            hWidth+armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #128
            hWidth+armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #129
            hWidth+armLength - propNarrowLength - propWidth, +farTU, -hDepth-armLength - propNarrowLength + propWidth, // #124
            hWidth+armLength - propNarrowLength + propWidth, +farTL, -hDepth-armLength - propNarrowLength - propWidth, // #125
            // FRONT RIGHT NORTHWEST PROPELLER BOTTOM MAIN SECTION (FACE #102)
            hWidth+armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength - propShortLength - propWidth, // #130
            hWidth+armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength - propShortLength + propWidth, // #131
            hWidth+armLength - propNarrowLength + propWidth, +farBL, -hDepth-armLength - propNarrowLength - propWidth, // #126
            hWidth+armLength - propNarrowLength - propWidth, +farBU, -hDepth-armLength - propNarrowLength + propWidth, // #127
            // FRONT RIGHT NORTHWEST PROPELLER SOUTH FACING SIDE (FACE #103)
            hWidth+armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength - propShortLength + propWidth, // #131
            hWidth+armLength + propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #134
            hWidth+armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #128
            hWidth+armLength - propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, //#132
            // FRONT RIGHT NORTHWEST PROPELLER EAST FACING SIDE (FACE #104)
            hWidth+armLength - propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #135
            hWidth+armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength - propShortLength - propWidth, // #130
            hWidth+armLength + propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #133
            hWidth+armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #129
            // FRONT RIGHT NORTHWEST PROPELLER TOP BACK SECTION (FACE #105)
            hWidth+armLength - propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, //#132
            hWidth+armLength + propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #133
            hWidth+armLength - propShortLength - propWidth, +closeTU, -hDepth-armLength - propShortLength + propWidth, // #128
            hWidth+armLength - propShortLength + propWidth, +closeTL, -hDepth-armLength - propShortLength - propWidth, // #129
            // FRONT RIGHT NORTHWEST PROPELLER BOTTOM BACK SECTION (FACE #106)
            hWidth+armLength - propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #135
            hWidth+armLength + propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #134
            hWidth+armLength - propShortLength + propWidth, +closeBL, -hDepth-armLength - propShortLength - propWidth, // #130
            hWidth+armLength - propShortLength - propWidth, +closeBU, -hDepth-armLength - propShortLength + propWidth, // #131
            // FRONT RIGHT SOUTHEAST PROPELLER NARROW TIP (FACE #107)
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #147
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #146
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #144
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #145
            // FRONT RIGHT SOUTHEAST PROPELLER SOUTH (FACE #108)
            hWidth+armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #143
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #147
            hWidth+armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #140
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #144
            // FRONT RIGHT SOUTHEAST PROPELLER EAST (FACE #109)
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #146
            hWidth+armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #142
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #145
            hWidth+armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #141
            // FRONT RIGHT SOUTHEAST PROPELLER TOP BACK SECTION (FACE #110)
            hWidth+armLength + propLength - propNarrowWidth, +tipTL, -hDepth-armLength + propLength + propNarrowWidth, // #144
            hWidth+armLength + propLength + propNarrowWidth, +tipTU, -hDepth-armLength + propLength - propNarrowWidth, // #145
            hWidth+armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #140
            hWidth+armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #141
            // FRONT RIGHT SOUTHEAST PROPELLER BOTTOM BACK SECTION (FACE #111)
            hWidth+armLength + propLength + propNarrowWidth, +tipBU, -hDepth-armLength + propLength - propNarrowWidth, // #146
            hWidth+armLength + propLength - propNarrowWidth, +tipBL, -hDepth-armLength + propLength + propNarrowWidth, // #147
            hWidth+armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #142
            hWidth+armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #143
            // FRONT RIGHT SOUTHEAST PROPELLER FRONT SIDE (FACE #112)
            hWidth+armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #142
            hWidth+armLength + propShortLength + propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #138
            hWidth+armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #141
            hWidth+armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #137
            // FRONT RIGHT SOUTHEAST PROPELLER BACK SIDE (FACE #113)
            hWidth+armLength + propShortLength - propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #139
            hWidth+armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #143
            hWidth+armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #136
            hWidth+armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #140
            // FRONT RIGHT SOUTHEAST PROPELLER TOP MAIN SECTION (FACE #114)
            hWidth+armLength + propNarrowLength - propWidth, +farTL, -hDepth-armLength + propNarrowLength + propWidth, // #140
            hWidth+armLength + propNarrowLength + propWidth, +farTU, -hDepth-armLength + propNarrowLength - propWidth, // #141
            hWidth+armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #136
            hWidth+armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #137
            // FRONT RIGHT SOUTHEAST PROPELLER BOTTOM MAIN SECTION (FACE #115)
            hWidth+armLength + propNarrowLength + propWidth, +farBU, -hDepth-armLength + propNarrowLength - propWidth, // #142
            hWidth+armLength + propNarrowLength - propWidth, +farBL, -hDepth-armLength + propNarrowLength + propWidth, // #143
            hWidth+armLength + propShortLength + propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #138
            hWidth+armLength + propShortLength - propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #139
            // FRONT RIGHT SOUTHEAST PROPELLER NORTH SIDE (FACE #116)
            hWidth+armLength + propShortLength + propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #138
            hWidth+armLength + propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #134
            hWidth+armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #137
            hWidth+armLength + propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #133
            // FRONT RIGHT SOUTHEAST PROPELLER WEST SIDE (FACE #117)
            hWidth+armLength - propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #135
            hWidth+armLength + propShortLength - propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #139
            hWidth+armLength - propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, //#132
            hWidth+armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #136
            // FRONT RIGHT SOUTHEAST PROPELLER TOP FRONT SECTION (FACE #118)
            hWidth+armLength + propShortLength - propWidth, +closeTL, -hDepth-armLength + propShortLength + propWidth, // #136
            hWidth+armLength + propShortLength + propWidth, +closeTU, -hDepth-armLength + propShortLength - propWidth, // #137
            hWidth+armLength - propNarrowWidth, +endT, -hDepth-armLength + propNarrowWidth, //#132
            hWidth+armLength + propNarrowWidth, +endT, -hDepth-armLength - propNarrowWidth, // #133
            // FRONT RIGHT SOUTHEAST PROPELLER BOTTOM FRONT SECTION (FACE #119)
            hWidth+armLength + propShortLength + propWidth, +closeBU, -hDepth-armLength + propShortLength - propWidth, // #138
            hWidth+armLength + propShortLength - propWidth, +closeBL, -hDepth-armLength + propShortLength + propWidth, // #139
            hWidth+armLength + propNarrowWidth, +endB, -hDepth-armLength - propNarrowWidth, // #134
            hWidth+armLength - propNarrowWidth, +endB, -hDepth-armLength + propNarrowWidth, // #135

            // BACK LEFT PROPELLER
            // BACK LEFT NORTHWEST PROPELLER NARROW TIP (FACE #120)
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, hDepth+armLength - propLength - propNarrowWidth, // #150
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, hDepth+armLength - propLength + propNarrowWidth, // #151
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, hDepth+armLength - propLength - propNarrowWidth, // #149
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, hDepth+armLength - propLength + propNarrowWidth, // #148
            // BACK LEFT NORTHWEST PROPELLER NORTH SIDE (FACE #121)
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, hDepth+armLength - propNarrowLength - propWidth, // #154
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, hDepth+armLength - propLength - propNarrowWidth, // #150
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, hDepth+armLength - propNarrowLength - propWidth, // #153
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, hDepth+armLength - propLength - propNarrowWidth, // #149
            // BACK LEFT NORTHWEST PROPELLER WEST SIDE (FACE #122)
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, hDepth+armLength - propLength + propNarrowWidth, // #151
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, hDepth+armLength - propNarrowLength + propWidth, // #155
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, hDepth+armLength - propLength + propNarrowWidth, // #148
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, hDepth+armLength - propNarrowLength + propWidth, // #152
            // BACK LEFT NORTHWEST PROPELLER TOP FRONT SECTION (FACE #123)
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, hDepth+armLength - propNarrowLength + propWidth, // #152
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, hDepth+armLength - propNarrowLength - propWidth, // #153
            -hWidth-armLength - propLength - propNarrowWidth, +tipTU, hDepth+armLength - propLength + propNarrowWidth, // #148
            -hWidth-armLength - propLength + propNarrowWidth, +tipTL, hDepth+armLength - propLength - propNarrowWidth, // #149
            // BACK LEFT NORTHWEST PROPELLER BOTTOM FRONT SECTION (FACE #124)
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, hDepth+armLength - propNarrowLength - propWidth, // #154
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, hDepth+armLength - propNarrowLength + propWidth, // #155
            -hWidth-armLength - propLength + propNarrowWidth, +tipBL, hDepth+armLength - propLength - propNarrowWidth, // #150
            -hWidth-armLength - propLength - propNarrowWidth, +tipBU, hDepth+armLength - propLength + propNarrowWidth, // #151
            // BACK LEFT NORTHWEST PROPELLER FRONT (FACE #125)
            -hWidth-armLength - propShortLength + propWidth, +closeBL, hDepth+armLength - propShortLength - propWidth, // #158
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, hDepth+armLength - propNarrowLength - propWidth, // #154
            -hWidth-armLength - propShortLength + propWidth, +closeTL, hDepth+armLength - propShortLength - propWidth, // #157
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, hDepth+armLength - propNarrowLength - propWidth, // #153
            // BACK LEFT NORTHWEST PROPELLER BACK (FACE #126)
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, hDepth+armLength - propNarrowLength + propWidth, // #155
            -hWidth-armLength - propShortLength - propWidth, +closeBU, hDepth+armLength - propShortLength + propWidth, // #159
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, hDepth+armLength - propNarrowLength + propWidth, // #153
            -hWidth-armLength - propShortLength - propWidth, +closeTU, hDepth+armLength - propShortLength + propWidth, // #156
            // BACK LEFT NORTHWEST PROPELLER TOP MAIN SECTION (FACE #127)
            -hWidth-armLength - propShortLength - propWidth, +closeTU, hDepth+armLength - propShortLength + propWidth, // #156
            -hWidth-armLength - propShortLength + propWidth, +closeTL, hDepth+armLength - propShortLength - propWidth, // #157
            -hWidth-armLength - propNarrowLength - propWidth, +farTU, hDepth+armLength - propNarrowLength + propWidth, // #152
            -hWidth-armLength - propNarrowLength + propWidth, +farTL, hDepth+armLength - propNarrowLength - propWidth, // #153
            // BACK LEFT NORTHWEST PROPELLER BOTTOM MAIN SECTION (FACE #128)
            -hWidth-armLength - propShortLength + propWidth, +closeBL, hDepth+armLength - propShortLength - propWidth, // #158
            -hWidth-armLength - propShortLength - propWidth, +closeBU, hDepth+armLength - propShortLength + propWidth, // #159
            -hWidth-armLength - propNarrowLength + propWidth, +farBL, hDepth+armLength - propNarrowLength - propWidth, // #154
            -hWidth-armLength - propNarrowLength - propWidth, +farBU, hDepth+armLength - propNarrowLength + propWidth, // #155
            // BACK LEFT NORTHWEST PROPELLER SOUTH FACING SIDE (FACE #129)
            -hWidth-armLength - propShortLength - propWidth, +closeBU, hDepth+armLength - propShortLength + propWidth, // #159
            -hWidth-armLength + propNarrowWidth, +endB, hDepth+armLength - propNarrowWidth, // #163
            -hWidth-armLength - propShortLength - propWidth, +closeTU, hDepth+armLength - propShortLength + propWidth, // #156
            -hWidth-armLength - propNarrowWidth, +endT, hDepth+armLength + propNarrowWidth, //#160
            // BACK LEFT NORTHWEST PROPELLER EAST FACING SIDE (FACE #130)
            -hWidth-armLength - propNarrowWidth, +endB, hDepth+armLength + propNarrowWidth, // #162
            -hWidth-armLength - propShortLength + propWidth, +closeBL, hDepth+armLength - propShortLength - propWidth, // #158
            -hWidth-armLength + propNarrowWidth, +endT, hDepth+armLength - propNarrowWidth, // #161
            -hWidth-armLength - propShortLength + propWidth, +closeTL, hDepth+armLength - propShortLength - propWidth, // #157
            // BACK LEFT NORTHWEST PROPELLER TOP BACK SECTION (FACE #131)
            -hWidth-armLength - propNarrowWidth, +endT, hDepth+armLength + propNarrowWidth, //#160
            -hWidth-armLength + propNarrowWidth, +endT, hDepth+armLength - propNarrowWidth, // #161
            -hWidth-armLength - propShortLength - propWidth, +closeTU, hDepth+armLength - propShortLength + propWidth, // #156
            -hWidth-armLength - propShortLength + propWidth, +closeTL, hDepth+armLength - propShortLength - propWidth, // #157
            // BACK LEFT NORTHWEST PROPELLER BOTTOM BACK SECTION (FACE #132)
            -hWidth-armLength - propNarrowWidth, +endB, hDepth+armLength + propNarrowWidth, // #162
            -hWidth-armLength + propNarrowWidth, +endB, hDepth+armLength - propNarrowWidth, // #163
            -hWidth-armLength - propShortLength + propWidth, +closeBL, hDepth+armLength - propShortLength - propWidth, // #158
            -hWidth-armLength - propShortLength - propWidth, +closeBU, hDepth+armLength - propShortLength + propWidth, // #159
            // BACK LEFT SOUTHEAST PROPELLER NARROW TIP (FACE #133)
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, hDepth+armLength + propLength + propNarrowWidth, // #175
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, hDepth+armLength + propLength - propNarrowWidth, // #174
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, hDepth+armLength + propLength + propNarrowWidth, // #172
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, hDepth+armLength + propLength - propNarrowWidth, // #173
            // BACK LEFT SOUTHEAST PROPELLER SOUTH (FACE #134)
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, hDepth+armLength + propNarrowLength + propWidth, // #171
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, hDepth+armLength + propLength + propNarrowWidth, // #175
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, hDepth+armLength + propNarrowLength + propWidth, // #168
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, hDepth+armLength + propLength + propNarrowWidth, // #172
            // BACK LEFT SOUTHEAST PROPELLER EAST (FACE #135)
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, hDepth+armLength + propLength - propNarrowWidth, // #174
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, hDepth+armLength + propNarrowLength - propWidth, // #170
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, hDepth+armLength + propLength - propNarrowWidth, // #173
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, hDepth+armLength + propNarrowLength - propWidth, // #169
            // BACK LEFT SOUTHEAST PROPELLER TOP BACK SECTION (FACE #136)
            -hWidth-armLength + propLength - propNarrowWidth, +tipTL, hDepth+armLength + propLength + propNarrowWidth, // #172
            -hWidth-armLength + propLength + propNarrowWidth, +tipTU, hDepth+armLength + propLength - propNarrowWidth, // #173
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, hDepth+armLength + propNarrowLength + propWidth, // #168
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, hDepth+armLength + propNarrowLength - propWidth, // #169
            // BACK LEFT SOUTHEAST PROPELLER BOTTOM BACK SECTION (FACE #137)
            -hWidth-armLength + propLength + propNarrowWidth, +tipBU, hDepth+armLength + propLength - propNarrowWidth, // #174
            -hWidth-armLength + propLength - propNarrowWidth, +tipBL, hDepth+armLength + propLength + propNarrowWidth, // #175
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, hDepth+armLength + propNarrowLength - propWidth, // #170
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, hDepth+armLength + propNarrowLength + propWidth, // #171
            // BACK LEFT SOUTHEAST PROPELLER FRONT SIDE (FACE #138)
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, hDepth+armLength + propNarrowLength - propWidth, // #170
            -hWidth-armLength + propShortLength + propWidth, +closeBU, hDepth+armLength + propShortLength - propWidth, // #166
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, hDepth+armLength + propNarrowLength - propWidth, // #169
            -hWidth-armLength + propShortLength + propWidth, +closeTU, hDepth+armLength + propShortLength - propWidth, // #165
            // BACK LEFT SOUTHEAST PROPELLER BACK SIDE (FACE #139)
            -hWidth-armLength + propShortLength - propWidth, +closeBL, hDepth+armLength + propShortLength + propWidth, // #167
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, hDepth+armLength + propNarrowLength + propWidth, // #171
            -hWidth-armLength + propShortLength - propWidth, +closeTL, hDepth+armLength + propShortLength + propWidth, // #164
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, hDepth+armLength + propNarrowLength + propWidth, // #168
            // BACK LEFT SOUTHEAST PROPELLER TOP MAIN SECTION (FACE #140)
            -hWidth-armLength + propNarrowLength - propWidth, +farTL, hDepth+armLength + propNarrowLength + propWidth, // #168
            -hWidth-armLength + propNarrowLength + propWidth, +farTU, hDepth+armLength + propNarrowLength - propWidth, // #169
            -hWidth-armLength + propShortLength - propWidth, +closeTL, hDepth+armLength + propShortLength + propWidth, // #164
            -hWidth-armLength + propShortLength + propWidth, +closeTU, hDepth+armLength + propShortLength - propWidth, // #165
            // BACK LEFT SOUTHEAST PROPELLER BOTTOM MAIN SECTION (FACE #141)
            -hWidth-armLength + propNarrowLength + propWidth, +farBU, hDepth+armLength + propNarrowLength - propWidth, // #170
            -hWidth-armLength + propNarrowLength - propWidth, +farBL, hDepth+armLength + propNarrowLength + propWidth, // #171
            -hWidth-armLength + propShortLength + propWidth, +closeBU, hDepth+armLength + propShortLength - propWidth, // #166
            -hWidth-armLength + propShortLength - propWidth, +closeBL, hDepth+armLength + propShortLength + propWidth, // #167
            // BACK LEFT SOUTHEAST PROPELLER NORTH SIDE (FACE #142)
            -hWidth-armLength + propShortLength + propWidth, +closeBU, hDepth+armLength + propShortLength - propWidth, // #166
            -hWidth-armLength + propNarrowWidth, +endB, hDepth+armLength - propNarrowWidth, // #162
            -hWidth-armLength + propShortLength + propWidth, +closeTU, hDepth+armLength + propShortLength - propWidth, // #165
            -hWidth-armLength + propNarrowWidth, +endT, hDepth+armLength - propNarrowWidth, // #161
            // BACK LEFT SOUTHEAST PROPELLER WEST SIDE (FACE #143)
            -hWidth-armLength - propNarrowWidth, +endB, hDepth+armLength + propNarrowWidth, // #163
            -hWidth-armLength + propShortLength - propWidth, +closeBL, hDepth+armLength + propShortLength + propWidth, // #167
            -hWidth-armLength - propNarrowWidth, +endT, hDepth+armLength + propNarrowWidth, //#160
            -hWidth-armLength + propShortLength - propWidth, +closeTL, hDepth+armLength + propShortLength + propWidth, // #164
            // BACK LEFT SOUTHEAST PROPELLER TOP FRONT SECTION (FACE #144)
            -hWidth-armLength + propShortLength - propWidth, +closeTL, hDepth+armLength + propShortLength + propWidth, // #164
            -hWidth-armLength + propShortLength + propWidth, +closeTU, hDepth+armLength + propShortLength - propWidth, // #165
            -hWidth-armLength - propNarrowWidth, +endT, hDepth+armLength + propNarrowWidth, //#160
            -hWidth-armLength + propNarrowWidth, +endT, hDepth+armLength - propNarrowWidth, // #161
            // BACK LEFT SOUTHEAST PROPELLER BOTTOM FRONT SECTION (FACE #145)
            -hWidth-armLength + propShortLength + propWidth, +closeBU, hDepth+armLength + propShortLength - propWidth, // #166
            -hWidth-armLength + propShortLength - propWidth, +closeBL, hDepth+armLength + propShortLength + propWidth, // #167
            -hWidth-armLength + propNarrowWidth, +endB, hDepth+armLength - propNarrowWidth, // #162
            -hWidth-armLength - propNarrowWidth, +endB, hDepth+armLength + propNarrowWidth, // #163
    };

    // Constructor - Set up the buffers
    public Quad() {
        // Setup vertex-array buffer. Vertices in float. An float has 4 bytes
        ByteBuffer vbb = ByteBuffer.allocateDirect(vertices.length * 4);
        vbb.order(ByteOrder.nativeOrder()); // Use native byte order
        vertexBuffer = vbb.asFloatBuffer(); // Convert from byte to float
        vertexBuffer.put(vertices);         // Copy data into buffer
        vertexBuffer.position(0);           // Rewind
    }

    // Draw the shape
    public void draw(GL10 gl) {
        gl.glFrontFace(GL10.GL_CCW);    // Front face in counter-clockwise orientation
        gl.glEnable(GL10.GL_CULL_FACE); // Enable cull face
        gl.glCullFace(GL10.GL_BACK);    // Cull the back face (don't display)

        gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
        gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffer);

        // Render all the faces
        for (int face = 0; face < numFaces; face++) {
            // Set the color for each of the faces
            gl.glColor4f(colors[face][0], colors[face][1], colors[face][2], colors[face][3]);
            // Draw the primitive from the vertex-array directly
            gl.glDrawArrays(GL10.GL_TRIANGLE_STRIP, face*4, 4);
        }
        gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
        gl.glDisable(GL10.GL_CULL_FACE);
    }
}