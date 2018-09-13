/*
------------------------------------
(C) Kudo Yuya, September 2018. All rights reserved.
Last Modified 2018-09-13
------------------------------------
Cannyエッジ検出を行うHLS向けC++コード
------------------------------------
*/

#include "canny_edge_detection.h"
#include "HlsImProc.hpp"
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
	// インターフェース指定
    #pragma HLS INTERFACE axis port=axis_in
    #pragma HLS INTERFACE axis port=axis_out
    #pragma HLS INTERFACE s_axilite port=hist_hthr bundle=CONTROL_BUS clock=s_axi_aclk
    #pragma HLS INTERFACE s_axilite port=hist_lthr bundle=CONTROL_BUS clock=s_axi_aclk
    #pragma HLS INTERFACE ap_ctrl_none port=return
	// データフロー指定
    #pragma HLS DATAFLOW
    // FIFO指定
    #pragma HLS STREAM variable=fifo1 depth=1 dim=1
    #pragma HLS STREAM variable=fifo2 depth=1 dim=1
    #pragma HLS STREAM variable=fifo3 depth=1 dim=1
    #pragma HLS STREAM variable=fifo4 depth=1 dim=1
    #pragma HLS STREAM variable=fifo5 depth=1 dim=1
    #pragma HLS STREAM variable=fifo6 depth=1 dim=1
    #pragma HLS STREAM variable=fifo7 depth=1 dim=1

    //--- 画像処理記述
    HlsImProc<MAX_WIDTH, MAX_HEIGHT> improc;
    
    // AXI4-StreamをGrayScale化してunsigned charの配列に格納
    improc.AXIS2GrayArray(axis_in, fifo1);
    // ガウシアンフィルタ
    improc.GaussianBlur(fifo1, fifo2);
    // ソーベルフィルタ
    improc.Sobel(fifo2, fifo3);
    // non-maximum suppression
    improc.NonMaxSuppression(fifo3, fifo4);
    // 境界ピクセルをzero paddingする
    unsigned int padding_size = 5;
    improc.ZeroPadding(fifo4, fifo5, padding_size);
    // ヒステリシス閾値処理
    improc.HystThreshold(fifo5, fifo6, hist_hthr, hist_lthr);
    // 近傍ピクセルとの比較演算
    improc.HystThresholdComp(fifo6, fifo7);
    // エッジ画像をAXI4-Streamに変換
    improc.GrayArray2AXIS(fifo7, axis_out);
}
