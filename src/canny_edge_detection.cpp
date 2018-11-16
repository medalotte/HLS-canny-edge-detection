/*
------------------------------------
(C) Kudo Yuya, September 2018. All rights reserved.
Last Modified 2018-09-13
------------------------------------
*/

#include "canny_edge_detection.h"

using namespace hls;
using namespace hlsimproc;

unsigned char fifo1[MAX_WIDTH * MAX_HEIGHT];
unsigned char fifo2[MAX_WIDTH * MAX_HEIGHT];
vector_image  fifo3[MAX_WIDTH * MAX_HEIGHT];
unsigned char fifo4[MAX_WIDTH * MAX_HEIGHT];
unsigned char fifo5[MAX_WIDTH * MAX_HEIGHT];
unsigned char fifo6[MAX_WIDTH * MAX_HEIGHT];
unsigned char fifo7[MAX_WIDTH * MAX_HEIGHT];

// Top Function
void canny_edge_detection(stream<rgb_image>& axis_in, stream<rgb_image>& axis_out, unsigned char& hist_hthr, unsigned char& hist_lthr) {
    // interface directive
    #pragma HLS INTERFACE axis port=axis_in
    #pragma HLS INTERFACE axis port=axis_out
    #pragma HLS INTERFACE s_axilite port=hist_hthr bundle=CONTROL_BUS clock=s_axi_aclk
    #pragma HLS INTERFACE s_axilite port=hist_lthr bundle=CONTROL_BUS clock=s_axi_aclk
    #pragma HLS INTERFACE ap_ctrl_none port=return
    // pipeline directive
    #pragma HLS DATAFLOW
    // FIFO directive
    #pragma HLS STREAM variable=fifo1 depth=1 dim=1
    #pragma HLS STREAM variable=fifo2 depth=1 dim=1
    #pragma HLS STREAM variable=fifo3 depth=1 dim=1
    #pragma HLS STREAM variable=fifo4 depth=1 dim=1
    #pragma HLS STREAM variable=fifo5 depth=1 dim=1
    #pragma HLS STREAM variable=fifo6 depth=1 dim=1
    #pragma HLS STREAM variable=fifo7 depth=1 dim=1

    // AXI4-Stream -> GrayScale image
	HlsImProc::AXIS2GrayArray<MAX_WIDTH, MAX_HEIGHT>(axis_in, fifo1);

    // exe gaussian bler
	HlsImProc::GaussianBlur<MAX_WIDTH, MAX_HEIGHT>(fifo1, fifo2);

	// exe sobel filter
	HlsImProc::Sobel<MAX_WIDTH, MAX_HEIGHT>(fifo2, fifo3);

	// exe non-maximum suppression
	HlsImProc::NonMaxSuppression<MAX_WIDTH, MAX_HEIGHT>(fifo3, fifo4);

	// exe zero padding at boundary pixel
    unsigned int padding_size = 5;
    HlsImProc::ZeroPadding<MAX_WIDTH, MAX_HEIGHT>(fifo4, fifo5, padding_size);

    // exe hysteresis threshold
    HlsImProc::HystThreshold<MAX_WIDTH, MAX_HEIGHT>(fifo5, fifo6, hist_hthr, hist_lthr);

    // exe comparison operation at neighboring pixels after exe hysteresis threshold
    HlsImProc::HystThresholdComp<MAX_WIDTH, MAX_HEIGHT>(fifo6, fifo7);

    // GrayScale image -> AXI4-Stream
    HlsImProc::GrayArray2AXIS<MAX_WIDTH, MAX_HEIGHT>(fifo7, axis_out);
}
