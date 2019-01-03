# HLS-canny-edge-detection
### Implement Canny edge detection to FPGA as IP using by Vivado HLS

- Protocol of input and output are AXI4-Stream
- IP core made by this code can run close to 1pix/clock because of pipeline processing
- You can make other image processing module that are like sequential access based on this code design

### example
<img src="testbench/lenna.png" alt="C simulation result">

<img src="assets/out.png" alt="C simulation result">

### cited
[Akira Yamawaki, Seiichi Serikawa, “A describing method of
an image processing software in C for a high-level synthesis
considering a function chaining,” IEICE trans. inf. & syst.,
vol.E101-D, no.2, February 2018.](https://www.jstage.jst.go.jp/article/transinf/E101.D/2/E101.D_2017RCP0001/_article)
