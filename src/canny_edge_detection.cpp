/*
The MIT License (MIT)

Copyright (c) 2019 Yuya Kudo.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "canny_edge_detection.h"

using namespace hls;
using namespace hlsimproc;

uint8_t fifo1[MAX_WIDTH * MAX_HEIGHT];
uint8_t fifo2[MAX_WIDTH * MAX_HEIGHT];
GradPix fifo3[MAX_WIDTH * MAX_HEIGHT];
uint8_t fifo4[MAX_WIDTH * MAX_HEIGHT];
uint8_t fifo5[MAX_WIDTH * MAX_HEIGHT];
uint8_t fifo6[MAX_WIDTH * MAX_HEIGHT];
uint8_t fifo7[MAX_WIDTH * MAX_HEIGHT];

// Top Function
void canny_edge_detection(stream<ImAxis<24> >& axis_in, stream<ImAxis<24> >& axis_out,
                          uint8_t& hist_hthr, uint8_t& hist_lthr) {
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
    const uint32_t PADDING_SIZE = 5;
    HlsImProc::ZeroPadding<MAX_WIDTH, MAX_HEIGHT>(fifo4, fifo5, PADDING_SIZE);

    // exe hysteresis threshold
    HlsImProc::HystThreshold<MAX_WIDTH, MAX_HEIGHT>(fifo5, fifo6, hist_hthr, hist_lthr);

    // exe comparison operation at neighboring pixels after exe hysteresis threshold
    HlsImProc::HystThresholdComp<MAX_WIDTH, MAX_HEIGHT>(fifo6, fifo7);

    // GrayScale image -> AXI4-Stream
    HlsImProc::GrayArray2AXIS<MAX_WIDTH, MAX_HEIGHT>(fifo7, axis_out);
}
