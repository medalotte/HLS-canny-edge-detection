#include <hls_opencv.h>
#include "canny_edge_detection.h"

int main() {
    hls::stream<rgb_image> axis_in, axis_out;

    // 画像ファイル読み込み
    cv::Mat src = cv::imread(INPUT_IMAGE);
    cv::Mat dst = src;

    // cv::MatをAXI4-Streamに変換
    cvMat2AXIvideo(src, axis_in);

    // Cannyエッジ検出
    unsigned char hist_hthr = 80;
    unsigned char hist_lthr = 20;
    canny_edge_detection(axis_in, axis_out, hist_hthr, hist_lthr);

    // AXI4-Streamをcv::Matに変換
    AXIvideo2cvMat(axis_out, dst);

    // 画像ファイル書き込み
    cv::imwrite(OUTPUT_IMAGE, dst);

    return 0;
}
