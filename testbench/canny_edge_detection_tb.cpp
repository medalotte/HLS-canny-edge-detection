#include <hls_opencv.h>
#include "canny_edge_detection.h"

int main() {
    hls::stream<rgb_image> axis_in, axis_out;

    // read image 
    cv::Mat src = cv::imread(INPUT_IMAGE);
    cv::Mat dst = src;

    // cv::Mat -> AXI4-Stream
    cvMat2AXIvideo(src, axis_in);

    // Canny edge detection
    unsigned char hist_hthr = 80;
    unsigned char hist_lthr = 20;
    canny_edge_detection(axis_in, axis_out, hist_hthr, hist_lthr);

    // AXI4-Stream -> cv::Mat
    AXIvideo2cvMat(axis_out, dst);

    // write image
    cv::imwrite(OUTPUT_IMAGE, dst);

    return 0;
}
