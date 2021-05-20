"""
"""

class EdgeDetectionCamera(OpenCVCamera):

    #Class variables?
    RES = RES_640x480
    FOV = 135

    KERNEL_SIZE = 3
    LO_THRESH = 50
    HI_THRESH = 100

    def __init__(self, None):
        
        OpenCVCamera(EdgeDetectionCamera.FOV, 
                    EdgeDetectionCamera.RES)





        def processImage(#CV:Matt image?)
            #CvT color

           cv.Mat gray;
           cv.cvtColor(image, gray, cv.COLOR_BGR2GRAY);

           /// Reduce noise with a 3x3 convolusion kernel
           cv::Mat edges;
           cv::blur(gray, edges, cv::Size(KERNEL_SIZE,KERNEL_SIZE));

           /// Run Canny edge detector
           cv::Canny(edges, edges, LO_THRESH, HI_THRESH, KERNEL_SIZE);

           // Display the edge-detected image
           cv::imshow("Edge Detection", edges);
           cv::waitKey(1);

##  What is CV?/
        
        
