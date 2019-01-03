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
