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

class SimRangefinder {

    public:

        double _min_distance_m;
        double _max_distance_m;
        int _width;
        int _height; 
        double _field_of_view_radians;

        SimRangefinder(
                const int width,
                const int height, 
                const double min_distance_m,
                const double max_distance_m,
                const double field_of_view_radians)
        {
            _width = width;
            _height = height; 
            _min_distance_m = min_distance_m;
            _max_distance_m = max_distance_m;
            _field_of_view_radians = field_of_view_radians;
        }

        void show(const int16_t * distance_mm, const uint16_t scaleup) 
        {
            const uint16_t new_width = _width * scaleup;
            const uint16_t new_height = _height * scaleup;

            cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

            const double min_distance_mm = _min_distance_m * 1000;
            const double max_distance_mm = _max_distance_m * 1000;

            for (uint8_t x=0; x<_width; ++x) {

                for (uint8_t y=0; y<_height; ++y) {

                    const double d = distance_mm[y * _width + x];

                    cv::rectangle(img,
                            cv::Point(x*scaleup, y*scaleup),
                            cv::Point((x+1)*scaleup, (y+1)*scaleup),
                            d == -1 ? 255 : (uint8_t)((d-min_distance_mm) /
                                (double)(max_distance_mm - min_distance_mm) * 255), 
                            -1);
                }
            }

            cv::imshow("lidar", img);

            cv::waitKey(1);
        }

        void report(const int16_t * distance_mm) 
        {
            for (int i=0; i<8; ++i) {
                for (int j=0; j<8; ++j) {
                    const int16_t d = distance_mm[i*8+j];
                    if (d < 0) {
                        printf(" ---- ");
                    }
                    else {
                        printf("%5d ", d);
                    }
                }
                printf("\n \n \n");
            }
            printf("\n-----------------------------------------------\n \n");
        }
};
