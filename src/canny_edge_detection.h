#ifndef SRC_CANNY_EDGE_DETECTION_H_
#define SRC_CANNY_EDGE_DETECTION_H_

#include <hls_stream.h>
#include <ap_axi_sdata.h>

#include "HlsImProc.hpp"

#define MAX_WIDTH  1280
#define MAX_HEIGHT 720

//--- for test bench
#define INPUT_IMAGE  "test_img.bmp"
#define OUTPUT_IMAGE "out.bmp"
//---

typedef hlsimproc::im_axis<24> rgb_image;

void canny_edge_detection(hls::stream<rgb_image>& axis_in, hls::stream<rgb_image>& axis_out, unsigned char& hist_hthr, unsigned char& hist_lthr);

#endif /* SRC_CANNY_EDGE_DETECTION_H_ */
