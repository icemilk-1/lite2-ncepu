# OpenCV Zoo and Benchmark

A zoo for models tuned for OpenCV DNN with benchmarks on different platforms.

Guidelines:

- Clone this repo to download all models and demo scripts:
  ```shell
  # Install git-lfs from https://git-lfs.github.com/
  git clone https://github.com/opencv/opencv_zoo && cd opencv_zoo
  git lfs install
  git lfs pull
  ```
- To run benchmarks on your hardware settings, please refer to [benchmark/README](../../../../../../../OpenCV_China/opencv_zoo/benchmark/README.md).

## Models & Benchmark Results

| Model                                                   | Task                          | Input Size | INTEL-CPU (ms)                                                                                 | RPI-CPU (ms) | JETSON-GPU (ms) | KV3-NPU (ms) | D1-CPU (ms) |
| ------------------------------------------------------- | ----------------------------- | ---------- |------------------------------------------------------------------------------------------------| ------------ | --------------- | ------------ | ----------- |
| [YuNet](../../../../../../../OpenCV_China/opencv_zoo/models/face_detection_yunet)                  | Face Detection                | 160x120    | ![](https://via.placeholder.com/15/c5f015/c5f015.png) 1.45                                     | 6.22         | 12.18           | 4.04         | 86.69       |
| [SFace](../../../../../../../OpenCV_China/opencv_zoo/models/face_recognition_sface)                | Face Recognition              | 112x112    | <img src="https://via.placeholder.com/15/c5f015/c5f015.png" style="pointer-events: none;">8.65 | 99.20        | 24.88           | 46.25        | ---         |
| [FER](./models/facial_expression_recognition/)          | Facial Expression Recognition | 112x112    | [![](https://via.placeholder.com/15/c5f015/c5f015.png)]()4.43                                 | 49.86        | 31.07           | 29.80        | ---         |
| [LPD-YuNet](./models/license_plate_detection_yunet/)    | License Plate Detection       | 320x240    | ---                                                                                            | 168.03       | 56.12           | 29.53        | ---         |
| [YOLOX](./models/object_detection_yolox/)               | Object Detection              | 640x640    | 176.68                                                                                         | 1496.70      | 388.95          | 420.98       | ---         |
| [NanoDet](./models/object_detection_nanodet/)           | Object Detection              | 416x416    | 157.91                                                                                         | 220.36       | 64.94           | 116.64       | ---         |
| [DB-IC15](../../../../../../../OpenCV_China/opencv_zoo/models/text_detection_db)                   | Text Detection                | 640x480    | 142.91                                                                                         | 2835.91      | 208.41          | ---          | ---         |
| [DB-TD500](../../../../../../../OpenCV_China/opencv_zoo/models/text_detection_db)                  | Text Detection                | 640x480    | 142.91                                                                                         | 2841.71      | 210.51          | ---          | ---         |
| [CRNN-EN](../../../../../../../OpenCV_China/opencv_zoo/models/text_recognition_crnn)               | Text Recognition              | 100x32     | 50.21                                                                                          | 234.32       | 196.15          | 125.30       | ---         |
| [CRNN-CN](../../../../../../../OpenCV_China/opencv_zoo/models/text_recognition_crnn)               | Text Recognition              | 100x32     | 73.52                                                                                          | 322.16       | 239.76          | 166.79       | ---         |
| [PP-ResNet](../../../../../../../OpenCV_China/opencv_zoo/models/image_classification_ppresnet)     | Image Classification          | 224x224    | 56.05                                                                                          | 602.58       | 98.64           | 75.45        | ---         |
| [MobileNet-V1](../../../../../../../OpenCV_China/opencv_zoo/models/image_classification_mobilenet) | Image Classification          | 224x224    | 9.04                                                                                           | 92.25        | 33.18           | 145.66\*     | ---         |
| [MobileNet-V2](../../../../../../../OpenCV_China/opencv_zoo/models/image_classification_mobilenet) | Image Classification          | 224x224    | 8.86                                                                                           | 74.03        | 31.92           | 146.31\*     | ---         |
| [PP-HumanSeg](../../../../../../../OpenCV_China/opencv_zoo/models/human_segmentation_pphumanseg)   | Human Segmentation            | 192x192    | 19.92                                                                                          | 105.32       | 67.97           | 74.77        | ---         |
| [WeChatQRCode](../../../../../../../OpenCV_China/opencv_zoo/models/qrcode_wechatqrcode)            | QR Code Detection and Parsing | 100x100    | 7.04                                                                                           | 37.68        | ---             | ---          | ---         |
| [DaSiamRPN](../../../../../../../OpenCV_China/opencv_zoo/models/object_tracking_dasiamrpn)         | Object Tracking               | 1280x720   | 36.15                                                                                          | 705.48       | 76.82           | ---          | ---         |
| [YoutuReID](../../../../../../../OpenCV_China/opencv_zoo/models/person_reid_youtureid)             | Person Re-Identification      | 128x256    | 35.81                                                                                          | 521.98       | 90.07           | 44.61        | ---         |
| [MP-PalmDet](../../../../../../../OpenCV_China/opencv_zoo/models/palm_detection_mediapipe)         | Palm Detection                | 192x192    | 11.09                                                                                          | 63.79        | 83.20           | 33.81        | ---         |
| [MP-HandPose](../../../../../../../OpenCV_China/opencv_zoo/models/handpose_estimation_mediapipe)   | Hand Pose Estimation          | 224x224    | 4.28                                                                                           | 36.19        | 40.10           | 19.47        | ---         |

\*: Models are quantized in per-channel mode, which run slower than per-tensor quantized models on NPU.

Hardware Setup:

- `INTEL-CPU`: [Intel Core i7-5930K](https://www.intel.com/content/www/us/en/products/sku/82931/intel-core-i75930k-processor-15m-cache-up-to-3-70-ghz/specifications.html) @ 3.50GHz, 6 cores, 12 threads.
- `RPI-CPU`: [Raspberry Pi 4B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/), Broadcom BCM2711, Quad core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz.
- `JETSON-GPU`: [NVIDIA Jetson Nano B01](https://developer.nvidia.com/embedded/jetson-nano-developer-kit), 128-core NVIDIA Maxwell GPU.
- `KV3-NPU`: [Khadas VIM3](https://www.khadas.com/vim3), 5TOPS Performance. Benchmarks are done using **quantized** models. You will need to compile OpenCV with TIM-VX following [this guide](https://github.com/opencv/opencv/wiki/TIM-VX-Backend-For-Running-OpenCV-On-NPU) to run benchmarks. The test results use the `per-tensor` quantization model by default.
- `D1-CPU`: [Allwinner D1](https://d1.docs.aw-ol.com/en), [Xuantie C906 CPU](https://www.t-head.cn/product/C906?spm=a2ouz.12986968.0.0.7bfc1384auGNPZ) (RISC-V, RVV 0.7.1) @ 1.0GHz, 1 core. YuNet is supported for now. Visit [here](https://github.com/fengyuentau/opencv_zoo_cpp) for more details.

***Important Notes***:

- The data under each column of hardware setups on the above table represents the elapsed time of an inference (preprocess, forward and postprocess).
- The time data is the median of 10 runs after some warmup runs. Different metrics may be applied to some specific models.
- Batch size is 1 for all benchmark results.
- `---` represents the model is not availble to run on the device.
- View [benchmark/config](../../../../../../../OpenCV_China/opencv_zoo/benchmark/config) for more details on benchmarking different models.

## Some Examples

Some examples are listed below. You can find more in the directory of each model!

### Face Detection with [YuNet](./models/face_detection_yunet/)

![largest selfie](../../../../../../../OpenCV_China/opencv_zoo/models/face_detection_yunet/examples/largest_selfie.jpg)

### Facial Expression Recognition with [Progressive Teacher](./models/facial_expression_recognition/)

![fer demo](../../../../../../../OpenCV_China/opencv_zoo/models/facial_expression_recognition/examples/selfie.jpg)

### Human Segmentation with [PP-HumanSeg](./models/human_segmentation_pphumanseg/)

![messi](../../../../../../../OpenCV_China/opencv_zoo/models/human_segmentation_pphumanseg/examples/messi.jpg)

### License Plate Detection with [LPD_YuNet](./models/license_plate_detection_yunet/)

![license plate detection](../../../../../../../OpenCV_China/opencv_zoo/models/license_plate_detection_yunet/examples/lpd_yunet_demo.gif)

### Object Detection with [NanoDet](./models/object_detection_nanodet/) & [YOLOX](./models/object_detection_yolox/)

![nanodet demo](../../../../../../../OpenCV_China/opencv_zoo/models/object_detection_nanodet/samples/1_res.jpg)

![yolox demo](../../../../../../../OpenCV_China/opencv_zoo/models/object_detection_yolox/samples/3_res.jpg)

### Object Tracking with [DaSiamRPN](./models/object_tracking_dasiamrpn/)

![webcam demo](../../../../../../../OpenCV_China/opencv_zoo/models/object_tracking_dasiamrpn/examples/dasiamrpn_demo.gif)

### Palm Detection with [MP-PalmDet](./models/palm_detection_mediapipe/)

![palm det](../../../../../../../OpenCV_China/opencv_zoo/models/palm_detection_mediapipe/examples/mppalmdet_demo.gif)

### Hand Pose Estimation with [MP-HandPose](models/handpose_estimation_mediapipe/)

![handpose estimation](../../../../../../../OpenCV_China/opencv_zoo/models/handpose_estimation_mediapipe/examples/mphandpose_demo.webp)

### QR Code Detection and Parsing with [WeChatQRCode](./models/qrcode_wechatqrcode/)

![qrcode](../../../../../../../OpenCV_China/opencv_zoo/models/qrcode_wechatqrcode/examples/wechat_qrcode_demo.gif)

### Chinese Text detection [DB](./models/text_detection_db/)

![mask](../../../../../../../OpenCV_China/opencv_zoo/models/text_detection_db/examples/mask.jpg)

### English Text detection [DB](./models/text_detection_db/)

![gsoc](../../../../../../../OpenCV_China/opencv_zoo/models/text_detection_db/examples/gsoc.jpg)

### Text Detection with [CRNN](./models/text_recognition_crnn/)

![crnn_demo](../../../../../../../OpenCV_China/opencv_zoo/models/text_recognition_crnn/examples/CRNNCTC.gif)

## License

OpenCV Zoo is licensed under the [Apache 2.0 license](LICENSE). Please refer to licenses of different models.
