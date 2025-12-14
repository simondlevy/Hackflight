/* 
   Lidar simulator

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// OpenCV
#include <opencv2/opencv.hpp>

class SimMultiRanger {

    public:

         SimMultiRanger(
                const uint16_t width,
                const uint16_t height, 
                const uint16_t min_distance_mm,
                const uint16_t max_distance_mm)
        {
            _width = width;
            _height = height; 
            _min_distance_mm = min_distance_mm;
            _max_distance_mm = max_distance_mm;
        }

        void show(const int16_t * distance_mm, const uint16_t scaleup) 
        {
            const uint16_t new_width = _width * scaleup;
            const uint16_t new_height = _height * scaleup;

            cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

            for (uint8_t j=0; j<_height; ++j) {

                for (uint8_t k=0; k<_width; ++k) {

                    const double d = distance_mm[k * _width + j];

                    cv::rectangle(img,
                            cv::Point(k*scaleup, j*scaleup),
                            cv::Point((k+1)*scaleup, (j+1)*scaleup),
                            d == -1 ? 255 : (uint8_t)((d-_min_distance_mm) /
                                (float)(_max_distance_mm - _min_distance_mm) * 255), 
                            -1);
                }
            }

            cv::imshow("lidar", img);

            cv::waitKey(1);
        }


    private:

         uint16_t _min_distance_mm;
         uint16_t _max_distance_mm;
         uint16_t _width;
         uint16_t _height; 
};
