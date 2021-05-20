/*
 * Abstract camera class for MulticopterSim using OpenCV
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../MainModule/Camera.hpp"

#include <opencv2/imgproc/imgproc.hpp>

class OpenCVCamera : public Camera {

    private:

        // RBGA image created from render target on main thread
        cv::Mat _rbga_image;

        // RGB image sent to subclass for processing
        cv::Mat _image;

    protected:

        OpenCVCamera(float fov, Resolution_t res, float x=Camera::X, float y=Camera::Y, float z=Camera::Z)
            : Camera(fov, res, x, y, z)
         {
            // Create a private RBGA image for acquiring render target on main thread
            _rbga_image = cv::Mat::zeros(_rows, _cols, CV_8UC4);

            // Create a public OpenCV BGR image for uses by other classes
            _image = cv::Mat::zeros(_rows, _cols, CV_8UC3);
        }

        virtual void processImageBytes(uint8_t * bytes) override
        { 
            // Copy the RBGA pixels to the private image
            FMemory::Memcpy(_rbga_image.data, bytes, _rows*_cols*4);

            // Convert RGBA => RGB for public image
            cv::cvtColor(_rbga_image, _image, cv::COLOR_RGBA2RGB);

            // Virtual method implemented in subclass
            processImage(_image);
        }

        // Override this method for your video application
        virtual void processImage(cv::Mat image) = 0;

}; // Class OpenCVCamera
