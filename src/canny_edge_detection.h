#ifndef __CANNY_EDGE_DETECTION__
#define __CANNY_EDGE_DETECTION__

#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define MAX_WIDTH  1280
#define MAX_HEIGHT 720

//--- for test bench
#define INPUT_IMAGE  "test_img.bmp"
#define OUTPUT_IMAGE "out.bmp"
//---

typedef ap_axiu<24,1,1,1> rgb_image;

void canny_edge_detection(hls::stream<rgb_image>& axis_in, hls::stream<rgb_image>& axis_out, unsigned char& hist_hthr, unsigned char& hist_lthr);

#endif /* !__CANNY_EDGE_DETECTION__ */
