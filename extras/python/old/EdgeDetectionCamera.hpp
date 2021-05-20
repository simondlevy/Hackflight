/*
   MulticopterSim OpenCVCamera subclass using edge detection

   Copyright(C) 2019 Simon D. Levy

   MIT License
   */

#pragma once

#include "OpenCVCamera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class EdgeDetectionCamera : public OpenCVCamera {

    private:

        // Camera params
        static constexpr Resolution_t RES = RES_640x480;
        static constexpr float FOV        = 135;

        // Edge-detection params
        static const uint8_t KERNEL_SIZE = 3;
        static const uint8_t LO_THRESH   = 50;
        static const uint8_t HI_THRESH   = 100;

    protected:

        virtual void processImage(cv::Mat image) override
        {
            // Make a grayscale copy of the image
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            /// Reduce noise with a 3x3 convolusion kernel
            cv::Mat edges;
            cv::blur(gray, edges, cv::Size(KERNEL_SIZE,KERNEL_SIZE));

            /// Run Canny edge detector
            cv::Canny(edges, edges, LO_THRESH, HI_THRESH, KERNEL_SIZE);

            // Display the edge-detected image
            cv::imshow("Edge Detection", edges);
            cv::waitKey(1);
        }

    public:

        EdgeDetectionCamera(void) 
            : OpenCVCamera(FOV, RES)
        {
        }

}; 
