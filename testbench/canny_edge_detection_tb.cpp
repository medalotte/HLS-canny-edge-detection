/*
  ------------------------------------
  (C) Kudo Yuya, Nov. 2018. All rights reserved.
  Last Modified 2018-11-17
  ------------------------------------
  C simulation test bench file for HLS-canny-edge-detection
  ------------------------------------
*/

#include <hls_opencv.h>
#include "../src/canny_edge_detection.h"

int main() {
    hls::stream<ap_axiu<24,1,1,1> > gen_axis_in, gen_axis_out;
    hls::stream<rgb_image> im_axis_in, im_axis_out;

    // read image 
    cv::Mat src = cv::imread(INPUT_IMAGE);
    cv::Mat dst = src;

    // cv::Mat -> AXI4-Stream
    cvMat2AXIvideo(src, gen_axis_in);

    // convert axis type (ap_axiu -> hlsimproc::im_axis)
    ap_axiu<24,1,1,1> gen_axis_reader;
    rgb_image im_axis_writer;

    for(int yi = 0; yi < MAX_HEIGHT; yi++) {
        for(int xi = 0; xi < MAX_WIDTH; xi++) {
            gen_axis_in >> gen_axis_reader;
            
            im_axis_writer.data = gen_axis_reader.data;
            im_axis_writer.user = gen_axis_reader.user;
            im_axis_writer.last = gen_axis_reader.last;
            
            im_axis_in << im_axis_writer;
        }
    }

    // canny edge detection
    unsigned char hist_hthr = 80;
    unsigned char hist_lthr = 20;
    canny_edge_detection(im_axis_in, im_axis_out, hist_hthr, hist_lthr);

    // convert axis type (hlsimproc::im_axis -> ap_axiu)
    ap_axiu<24,1,1,1> gen_axis_writer;
    rgb_image im_axis_reader;

    for(int yi = 0; yi < MAX_HEIGHT; yi++) {
        for(int xi = 0; xi < MAX_WIDTH; xi++) {
            im_axis_out >> im_axis_reader;

            gen_axis_writer.data = im_axis_reader.data;
            gen_axis_writer.user = im_axis_reader.user;
            gen_axis_writer.last = im_axis_reader.last;

            gen_axis_out << gen_axis_writer;
        }
    }

    // AXI4-Stream -> cv::Mat
    AXIvideo2cvMat(gen_axis_out, dst);

    // write image
    cv::imwrite(OUTPUT_IMAGE, dst);

    return 0;
}
