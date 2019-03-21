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

#ifndef SRC_CANNY_EDGE_DETECTION_H_
#define SRC_CANNY_EDGE_DETECTION_H_

#include <stdint.h>

#include <hls_stream.h>
#include <ap_axi_sdata.h>

#include "HlsImProc.hpp"

#define MAX_WIDTH  512
#define MAX_HEIGHT 512

//--- for test bench
#define INPUT_IMAGE  "lenna.png"
#define OUTPUT_IMAGE "out.png"
#define CANNY_HTHR   80
#define CANNY_LTHR   20
//---

void canny_edge_detection(hls::stream<hlsimproc::ImAxis<24> >& axis_in, hls::stream<hlsimproc::ImAxis<24> >& axis_out,
                          uint8_t& hist_hthr, uint8_t& hist_lthr);

#endif /* SRC_CANNY_EDGE_DETECTION_H_ */
