#ifndef SRC_CANNY_EDGE_DETECTION_H_
#define SRC_CANNY_EDGE_DETECTION_H_

#include <stdint.h>

#include <hls_stream.h>
#include <ap_axi_sdata.h>

#include "HlsImProc.hpp"

#define MAX_WIDTH  1280
#define MAX_HEIGHT 720

//--- for test bench
#define INPUT_IMAGE  "test_img.bmp"
#define OUTPUT_IMAGE "out.bmp"
//---

void canny_edge_detection(hls::stream<hlsimproc::ImAxis<24> >& axis_in, hls::stream<hlsimproc::ImAxis<24> >& axis_out,
                          uint8_t& hist_hthr, uint8_t& hist_lthr);

#endif /* SRC_CANNY_EDGE_DETECTION_H_ */
