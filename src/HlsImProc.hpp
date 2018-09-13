/*
  ------------------------------------
  (C) Kudo Yuya, September 2018. All rights reserved.
  Last Modified 2018-09-13
  -----------------------------------
  Vivado HLS対応画像処理実行クラス
  ------------------------------------
*/

#ifndef __HLS_IM_PROC__
#define __HLS_IM_PROC__

#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_math.h>

namespace hlsimproc
{
	//--- 勾配方向の定義
    enum EDIR {
        DIR_0,
        DIR_45,
        DIR_90,
        DIR_135
    };

	//--- 勾配情報を持った画像データ配列の構造体
	struct vector_image {
		unsigned char value;
		unsigned char grad;
	};

    //--- 画像処理実行クラス
    template<int WIDTH, int HEIGHT>
    class HlsImProc
    {
    public:
        // AXI4-Stream形式の画像をGrayScale化してunsigned char配列に格納する
        void AXIS2GrayArray(hls::stream<ap_axiu<24,1,1,1> >& axis_src, unsigned char* dst);
        // GrayScale化されたunsigned char配列をAXI4-Stream形式の画像に変換する
        void GrayArray2AXIS(unsigned char* src, hls::stream<ap_axiu<24,1,1,1> >& axis_dst);
        // ガウシアンフィルタ（5x5）を実行する
        void GaussianBlur(unsigned char* src, unsigned char* dst);
        // ソーベルフィルタを実行する
        void Sobel(unsigned char* src, vector_image* dst);
        // non-maximum suppression（非極大抑制）を実行する
        void NonMaxSuppression(vector_image* src, unsigned char* dst);
        // ヒステリシス閾値化を実行する
        void HystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr);
        // ヒステリシス閾値化後の画像に対して近傍ピクセルとの比較演算を実行する
        void HystThresholdComp(unsigned char* src, unsigned char* dst);
        // 境界ピクセルを0で埋める
        void ZeroPadding(unsigned char* src, unsigned char* dst, unsigned int padding_size);
    };

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::AXIS2GrayArray(hls::stream<ap_axiu<24,1,1,1> >& axis_src, unsigned char* dst) {
        ap_axiu<24,1,1,1> axis_reader; // AXI4-Streamの読み取り用変数
        bool sof = false;              // Start of Frame
        bool eol = false;              // End of Line

        // user信号のアサートを待つ
        while (!sof) {
            #pragma HLS PIPELINE II=1
            #pragma HLS LOOP_TRIPCOUNT avg=0 max=0
            axis_src >> axis_reader;
            sof = axis_reader.user.to_int();
        }

		// 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
			eol = false;
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

				// last信号がアサートされるまでpix取得
				if(sof || eol) {
					// user信号がアサートされているpxは取得済み
					// WIDTHが実際のフレームサイズ以上に設定された場合
					sof = false;
					eol = axis_reader.last.to_int();
				}
                else {
					axis_src >> axis_reader;
					eol = axis_reader.last.to_int();
				}

				//--- グレイスケール処理
				int pix_gray;  // 出力画素値

				// Y = B*0.144 + G*0.587 + R*0.299
				// 16ビット左シフトさせた近似値を使用
				pix_gray =  9437* (axis_reader.data & 0x0000ff)
                    + 38469*((axis_reader.data & 0x00ff00) >> 8 )
                    + 19595*((axis_reader.data & 0xff0000) >> 16);

				pix_gray >>= 16;

				// 飽和処理（丸め誤差によるオーバーフロー対策）
				if(pix_gray < 0) {
					pix_gray = 0;
				}
                else if(pix_gray > 255) {
					pix_gray = 255;
				}

				// dataを書き込む
				dst[xi + yi*WIDTH] = pix_gray;
			}

			// WIDTHが実際のフレームサイズ以下に設定された場合
			// last信号がアサートするまで読み込む
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
		ap_axiu<24,1,1,1> axis_writer; // AXI4-Streamの書き込み用変数

		// 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

				// dataを書き込む
				unsigned int pix_out = src[xi + yi*WIDTH];
				axis_writer.data = pix_out << 16 | pix_out << 8 | pix_out;

				// フレームの先頭でuser信号をアサートする
				if (xi == 0 && yi == 0) {
					axis_writer.user = 1;
				}
                else {
					axis_writer.user = 0;
				}
				// 各ラインの末尾でlast信号をアサートする
				if (xi == (WIDTH - 1)) {
					axis_writer.last = 1;
				}
                else {
					axis_writer.last = 0;
				}

				// AXI4-Stream出力
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

		// 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

				//--- ガウシアンフィルタ
				int pix_gauss; // 出力画素値

				//-- ラインバッファ
                for(int yl = 0; yl < KERNEL_SIZE - 1; yl++) {
					line_buf[yl][xi] = line_buf[yl + 1][xi];
				}
                
				// 入力
				line_buf[KERNEL_SIZE - 1][xi] = src[xi + yi*WIDTH];

				//-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE - 1; xw++) {
						window_buf[yw][xw] = window_buf[yw][xw + 1];
					}
				}
                
				// ラインバッファの各列を入力
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
					window_buf[yw][KERNEL_SIZE - 1] = line_buf[yw][xi];
				}

				//-- 畳み込み演算
				pix_gauss = 0;
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
						pix_gauss += window_buf[yw][xw] * GAUSS_KERNEL[yw][xw];
					}
				}

				// 8bit right shift
				pix_gauss >>= 8;

				// 出力
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

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- ソーベルフィルタ
                int pix_sobel;  // 出力画素値
                int grad_sobel; // 勾配方向

                //-- ラインバッファ
                for(int yl = 0; yl < KERNEL_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[KERNEL_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    window_buf[yw][KERNEL_SIZE - 1] = line_buf[yw][xi];
                }

                //-- 畳み込み演算
                // 20180510 halfにするとノルム算出時にオーバーフローが起こる
                int pix_h_sobel = 0;
                int pix_v_sobel = 0;

                // 水平方向の畳み込み
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
                        pix_h_sobel += window_buf[yw][xw] * H_SOBEL_KERNEL[yw][xw];
                    }
                }

                // 垂直方向の畳み込み
                for(int yw = 0; yw < KERNEL_SIZE; yw++) {
                    for(int xw = 0; xw < KERNEL_SIZE; xw++) {
                        pix_v_sobel += window_buf[yw][xw] * V_SOBEL_KERNEL[yw][xw];
                    }
                }

                // 出力画素値を算出
                pix_sobel = hls::sqrt(float(pix_h_sobel * pix_h_sobel + pix_v_sobel * pix_v_sobel));

                // 飽和処理
                if(255 < pix_sobel) {
                    pix_sobel = 255;
                }

                // 勾配方向を算出
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

                // 出力
                if((KERNEL_SIZE < xi && xi < WIDTH - KERNEL_SIZE) &&
                   (KERNEL_SIZE < yi && yi < HEIGHT - KERNEL_SIZE)) {
                    dst[xi + yi*WIDTH].value = pix_sobel;
                    dst[xi + yi*WIDTH].grad  = grad_sobel;
                }
                // 境界ピクセルは0を出力する
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

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
                unsigned char value_nms;  // 出力画素値
                unsigned char grad_nms;   // 出力画素の勾配方向

                //-- ラインバッファ
                for(int yl = 0; yl < WINDOW_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[WINDOW_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    window_buf[yw][WINDOW_SIZE - 1] = line_buf[yw][xi];
                }

                //-- 勾配方向に対する比較演算
                value_nms = window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2].value;
                grad_nms = window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2].grad;
                // grad 0° -> 左, 右
                if(grad_nms == DIR_0) {
                    if(value_nms < window_buf[WINDOW_SIZE / 2][0].value ||
                       value_nms < window_buf[WINDOW_SIZE / 2][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 45° -> 左上, 右下
                else if(grad_nms == DIR_45) {
                    if(value_nms < window_buf[0][0].value ||
                       value_nms < window_buf[WINDOW_SIZE - 1][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 90° -> 上, 下
                else if(grad_nms == DIR_90) {
                    if(value_nms < window_buf[0][WINDOW_SIZE - 1].value ||
                       value_nms < window_buf[WINDOW_SIZE - 1][WINDOW_SIZE / 2].value) {
                        value_nms = 0;
                    }
                }
                // grad 135° -> 左下, 右上
                else if(grad_nms == DIR_135) {
                    if(value_nms < window_buf[WINDOW_SIZE - 1][0].value ||
                       value_nms < window_buf[0][WINDOW_SIZE - 1].value) {
                        value_nms = 0;
                    }
                }

                // 出力
                if((WINDOW_SIZE < xi && xi < WIDTH - WINDOW_SIZE) &&
                   (WINDOW_SIZE < yi && yi < HEIGHT - WINDOW_SIZE)) {
                    dst[xi + yi*WIDTH] = value_nms;
                }
                else {
                    // 境界ピクセルは0を出力する
                    dst[xi + yi*WIDTH] = 0;
                }
            }
        }
    }

	template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::HystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- ヒステリシス閾値化
                int pix_hyst;  // 出力画素値

                if(src[xi + yi*WIDTH] < lthr) {
                    pix_hyst = 0;
                }
                else if(src[xi + yi*WIDTH] > hthr) {
                    pix_hyst = 255;
                }
                else {
                    pix_hyst = 1;
                }
                // 出力
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

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
            	unsigned char pix_hyst = 0;  // 出力画素値

                //-- ラインバッファ
                for(int yl = 0; yl < WINDOW_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[WINDOW_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    window_buf[yw][WINDOW_SIZE - 1] = line_buf[yw][xi];
                }

                //-- 比較演算
                for(int yw = 0; yw < WINDOW_SIZE; yw++) {
                    for(int xw = 0; xw < WINDOW_SIZE; xw++) {
                        if(window_buf[WINDOW_SIZE / 2][WINDOW_SIZE / 2] != 0) {
                            if(window_buf[yw][xw] == 0xFF) {
                                pix_hyst = 0xFF;
                            }
                        }
                    }
                }

                // 出力
                dst[xi + yi*WIDTH] = pix_hyst;
            }
        }
    }
    
    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::ZeroPadding(unsigned char* src, unsigned char* dst, unsigned int padding_size) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off
                
                // 出力
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
