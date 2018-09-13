/*
  ------------------------------------
  (C) Kudo Yuya, September 2018. All rights reserved.
  Last Modified 2018-09-13
  ------------------------------------
  Class of image processing for Vivado HLS
  ------------------------------------
*/

#ifndef __HLS_IM_PROC__
#define __HLS_IM_PROC__

#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_math.h>

namespace hlsimproc
{
    //--- definition of gradient direction
    enum EDIR {
        DIR_0,
        DIR_45,
        DIR_90,
        DIR_135
    };

    //--- struct of image that have gradient info
    struct vector_image {
        unsigned char value;
        unsigned char grad;
    };
    
    //--- class of exe image processing
    template<int WIDTH, int HEIGHT>
    class HlsImProc
    {
    public:
        // AXI4-Stream -> GrayScale image
        void AXIS2GrayArray(hls::stream<ap_axiu<24,1,1,1> >& axis_src, unsigned char* dst);
        // GrayScale image -> AXI4-Stream
        void GrayArray2AXIS(unsigned char* src, hls::stream<ap_axiu<24,1,1,1> >& axis_dst);
        // exe gaussian bler
        void GaussianBlur(unsigned char* src, unsigned char* dst);
        // exe sobel filter
        void Sobel(unsigned char* src, vector_image* dst);
        // exe non-maximum suppression
        void NonMaxSuppression(vector_image* src, unsigned char* dst);
        // exe hysteresis threshold
        void HystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr);
        // exe comparison operation at neighboring pixels after exe hysteresis threshold
        void HystThresholdComp(unsigned char* src, unsigned char* dst);
        // exe zero padding at boundary pixel
        void ZeroPadding(unsigned char* src, unsigned char* dst, unsigned int padding_size);
    };

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::AXIS2GrayArray(hls::stream<ap_axiu<24,1,1,1> >& axis_src, unsigned char* dst) {
        ap_axiu<24,1,1,1> axis_reader; // for read AXI4-Stream
        bool sof = false;              // Start of Frame
        bool eol = false;              // End of Line

        // wait for the user signal to be asserted
        while (!sof) {
            #pragma HLS PIPELINE II=1
            #pragma HLS LOOP_TRIPCOUNT avg=0 max=0
            
            axis_src >> axis_reader;
            sof = axis_reader.user.to_int();
        }

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            eol = false;
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // get pix until the last signal to be asserted
                if(sof || eol) {
                    // when frame is started (first pix have already latched)
                    // or
                    // when WIDTH param set more than actual frame size
                    sof = false;
                    eol = axis_reader.last.to_int();
                }
                else {
                    axis_src >> axis_reader;
                    eol = axis_reader.last.to_int();
                }

                //--- grayscale processing
                int pix_gray;

                // Y = B*0.144 + G*0.587 + R*0.299
                pix_gray = 9437*(axis_reader.data & 0x0000ff)
                    + 38469*((axis_reader.data & 0x00ff00) >> 8 )
                    + 19595*((axis_reader.data & 0xff0000) >> 16);

                pix_gray >>= 16;

                // to consider saturation
                if(pix_gray < 0) {
                    pix_gray = 0;
                }
                else if(pix_gray > 255) {
                    pix_gray = 255;
                }

                // output
                dst[xi + yi*WIDTH] = pix_gray;
            }

            // when WIDTH param set less than actual frame size
            // wait for the last signal to be asserted
            while (!eol) {
                #pragma HLS pipeline II=1
                #pragma HLS loop_tripcount avg=0 max=0
                axis_src >> axis_reader;
                eol = axis_reader.last.to_int();
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::GrayArray2AXIS(unsigned char* src, hls::stream<ap_axiu<24,1,1,1> >& axis_dst) {
        ap_axiu<24,1,1,1> axis_writer; // for write AXI4-Stream

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                unsigned int pix_out = src[xi + yi*WIDTH];
                axis_writer.data = pix_out << 16 | pix_out << 8 | pix_out;

                // assert user signal at start of frame
                if (xi == 0 && yi == 0) {
                    axis_writer.user = 1;
                }
                else {
                    axis_writer.user = 0;
                }
                // assert last signal at end of line
                if (xi == (WIDTH - 1)) {
                    axis_writer.last = 1;
                }
                else {
                    axis_writer.last = 0;
                }

                // output
                axis_dst << axis_writer;
            }
        }
    }
        
    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::GaussianBlur(unsigned char* src, unsigned char* dst) {
        const int KERNEL_SIZE = 5;

        unsigned char line_buf[KERNEL_SIZE][WIDTH];
        unsigned char window_buf[KERNEL_SIZE][KERNEL_SIZE];

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0
        
        //-- 5x5 Gaussian kernel (8bit left shift)
        const int GAUSS_KERNEL[KERNEL_SIZE][KERNEL_SIZE] = { {1,  4,  6,  4, 1},
                                                             {4, 16, 24, 16, 4},
                                                             {6, 24, 36, 24, 6},
                                                             {4, 16, 24, 16, 4},
                                                             {1,  4,  6,  4, 1} };

        #pragma HLS ARRAY_PARTITION variable=GAUSS_KERNEL complete dim=0

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- gaussian bler
                int pix_gauss;
                
                //-- line buffer
                for(int yl = 0; yl < KERNEL_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                
                // write to line buffer
                line_buf[KERNEL_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- window buffer
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                
                // write to window buffer
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    window_buf[yw][KERNEL_SIZE - 1] = line_buf[yw][xi];
                }

                //-- convolution
                pix_gauss = 0;
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
                        pix_gauss += window_buf[yw][xw] * GAUSS_KERNEL[yw][xw];
                    }
                }

                // 8bit right shift
                pix_gauss >>= 8;

                // output
                dst[xi + yi*WIDTH] = pix_gauss;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::Sobel(unsigned char* src, vector_image* dst) {
        const int KERNEL_SIZE = 3;

        unsigned char line_buf[KERNEL_SIZE][WIDTH];
        unsigned char window_buf[KERNEL_SIZE][KERNEL_SIZE];

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0
        
        //-- 3x3 Horizontal Sobel kernel
        const int H_SOBEL_KERNEL[KERNEL_SIZE][KERNEL_SIZE] = {  { 1,  0, -1},
                                                                { 2,  0, -2},
                                                                { 1,  0, -1}   };
        //-- 3x3 vertical Sobel kernel
        const int V_SOBEL_KERNEL[KERNEL_SIZE][KERNEL_SIZE] = {  { 1,  2,  1},
                                                                { 0,  0,  0},
                                                                {-1, -2, -1}   };

        #pragma HLS ARRAY_PARTITION variable=H_SOBEL_KERNEL complete dim=0
        #pragma HLS ARRAY_PARTITION variable=V_SOBEL_KERNEL complete dim=0

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- sobel
                int pix_sobel;
                int grad_sobel;

                //-- line buffer
                for(int yl = 0; yl < KERNEL_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // write to line buffer
                line_buf[KERNEL_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- window buffer
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // write to window buffer
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    window_buf[yw][KERNEL_SIZE - 1] = line_buf[yw][xi];
                }

                //-- convolution
                int pix_h_sobel = 0;
                int pix_v_sobel = 0;

                // convolution using by holizonal kernel
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
                        pix_h_sobel += window_buf[yw][xw] * H_SOBEL_KERNEL[yw][xw];
                    }
                }

                // convolution using by vertical kernel
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
                        pix_v_sobel += window_buf[yw][xw] * V_SOBEL_KERNEL[yw][xw];
                    }
                }

                pix_sobel = hls::sqrt(float(pix_h_sobel * pix_h_sobel + pix_v_sobel * pix_v_sobel));

                // to consider saturation
                if(255 < pix_sobel) {
                    pix_sobel = 255;
                }

                // evaluate gradient direction
                int t_int;
                if(pix_h_sobel != 0) {
                    t_int = pix_v_sobel * 256 / pix_h_sobel;
                }
                else {
                    t_int = 0x7FFFFFFF;
                }

                // 112.5° ~ 157.5° (tan 112.5° ~= -2.4142, tan 157.5° ~= -0.4142)
                if(-618 < t_int && t_int <= -106) {
                    grad_sobel = DIR_135;
                }
                // -22.5° ~ 22.5° (tan -22.5° ~= -0.4142, tan 22.5° = 0.4142)
                else if(-106 < t_int && t_int <= 106) {
                    grad_sobel = DIR_0;
                }
                // 22.5° ~ 67.5° (tan 22.5° ~= 0.4142, tan 67.5° = 2.4142)
                else if(106 < t_int && t_int < 618) {
                    grad_sobel = DIR_45;
                }
                // 67.5° ~ 112.5° (to inf)
                else {
                    grad_sobel = DIR_90;
                }

                // output
                if((KERNEL_SIZE < xi && xi < WIDTH - KERNEL_SIZE) &&
                   (KERNEL_SIZE < yi && yi < HEIGHT - KERNEL_SIZE)) {
                    dst[xi + yi*WIDTH].value = pix_sobel;
                    dst[xi + yi*WIDTH].grad  = grad_sobel;
                }
                else {
                    dst[xi + yi*WIDTH].value = 0;
                    dst[xi + yi*WIDTH].grad  = 0;
                }
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::NonMaxSuppression(vector_image* src, unsigned char* dst) {
        const int WINDOW_SIZE = 3;

        vector_image line_buf[WINDOW_SIZE][WIDTH];
        vector_image window_buf[WINDOW_SIZE][WINDOW_SIZE];

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
                unsigned char value_nms;
                unsigned char grad_nms;

                //-- line buffer
                for(int yl = 0; yl < WINDOW_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // write to line buffer
                line_buf[WINDOW_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- window buffer
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // write to window buffer
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    window_buf[yw][WINDOW_SIZE - 1] = line_buf[yw][xi];
                }

                value_nms = window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2].value;
                grad_nms = window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2].grad;
                // grad 0° -> left, right
                if(grad_nms == DIR_0) {
                    if(value_nms < window_buf[WINDOW_SIZE / 2][0].value ||
                       value_nms < window_buf[WINDOW_SIZE / 2][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 45° -> upper left, bottom right
                else if(grad_nms == DIR_45) {
                    if(value_nms < window_buf[0][0].value ||
                       value_nms < window_buf[WINDOW_SIZE - 1][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 90° -> upper, bottom
                else if(grad_nms == DIR_90) {
                    if(value_nms < window_buf[0][WINDOW_SIZE - 1].value ||
                       value_nms < window_buf[WINDOW_SIZE - 1][WINDOW_SIZE / 2].value) {
                        value_nms = 0;
                    }
                }
                // grad 135° -> bottom left, upper right
                else if(grad_nms == DIR_135) {
                    if(value_nms < window_buf[WINDOW_SIZE - 1][0].value ||
                       value_nms < window_buf[0][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }

                // output
                if((WINDOW_SIZE < xi && xi < WIDTH - WINDOW_SIZE) &&
                   (WINDOW_SIZE < yi && yi < HEIGHT - WINDOW_SIZE)) {
                    dst[xi + yi*WIDTH] = value_nms;
                }
                else {
                    dst[xi + yi*WIDTH] = 0;
                }
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::HystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr) {
        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- hysteresis threshold
                int pix_hyst;

                if(src[xi + yi*WIDTH] < lthr) {
                    pix_hyst = 0;
                }
                else if(src[xi + yi*WIDTH] > hthr) {
                    pix_hyst = 255;
                }
                else {
                    pix_hyst = 1;
                }
                
                // output
                dst[xi + yi*WIDTH] = pix_hyst;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::HystThresholdComp(unsigned char* src, unsigned char* dst) {
        const int WINDOW_SIZE = 3;

        unsigned char line_buf[WINDOW_SIZE][WIDTH];
        unsigned char window_buf[WINDOW_SIZE][WINDOW_SIZE];

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0

        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
                unsigned char pix_hyst = 0;

                //-- line buffer
                for(int yl = 0; yl < WINDOW_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // write to line buffer
                line_buf[WINDOW_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- window buffer
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // write to window buffer
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    window_buf[yw][WINDOW_SIZE - 1] = line_buf[yw][xi];
                }

                //-- comparison operation
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE; xw++) {
                        if(window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2] != 0) {
                            if(window_buf[yw][xw] == 0xFF) {
                                pix_hyst = 0xFF;
                            }
                        }
                    }
                }

                // output
                dst[xi + yi*WIDTH] = pix_hyst;
            }
        }
    }
    
    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::ZeroPadding(unsigned char* src, unsigned char* dst, unsigned int padding_size) {
        // image proc loop
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off
                
                // output
                unsigned char pix = src[xi + yi*WIDTH];
                if((padding_size < xi && xi < WIDTH - padding_size) &&
                   (padding_size < yi && yi < HEIGHT - padding_size)) {
                    dst[xi + yi*WIDTH] = pix;
                }
                else {
                    dst[xi + yi*WIDTH] = 0;
                }
            }
        }
    }
}

#endif /* !__HLS_IM_PROC__ */
