import numpy as np
import cv2 as cv

from opencv_zoo.models.qrcode_wechatqrcode.wechatqrcode import WeChatQRCode
from utils.ColorDetection import ColorDetection


if __name__ == '__main__':
    # try to use CUDA
    if cv.cuda.getCudaEnabledDeviceCount() != 0:
        backend = cv.dnn.DNN_BACKEND_CUDA
        target = cv.dnn.DNN_TARGET_CUDA
    else:
        backend = cv.dnn.DNN_BACKEND_DEFAULT
        target = cv.dnn.DNN_TARGET_CPU
        print('CUDA is not set, will fall back to CPU.')

    # Instantiate WeChatQRCode
    model = WeChatQRCode("./opencv_zoo/models/qrcode_wechatqrcode/detect_2021nov.prototxt",
        "./opencv_zoo/models/qrcode_wechatqrcode/detect_2021nov.caffemodel",
        "./opencv_zoo/models/qrcode_wechatqrcode/sr_2021nov.prototxt",
        "./opencv_zoo/models/qrcode_wechatqrcode/sr_2021nov.caffemodel",
        backendId=backend,
        targetId=target)

    detector = ColorDetection(np.array([0, 145, 196]), np.array([14, 245, 245]))
    cap = cv.VideoCapture(0)
    # ottle_width = 5 # cm
    pixel_width = 60  # pixels when distance is 50cm

    # find QR code then go there
    while(True):
        ret, frame = cap.read()
        if not ret:
            continue
        res, points = model.infer(frame)




    while(True):
        ret, frame = cap.read()
        if not ret:
            continue
        bbox = detector.detect(frame)
        if bbox is not None:
            cv.rectangle(frame, bbox[0], bbox[1], (0, 255, 0))
            pixels = bbox[1][0] - bbox[0][0]
            print("distance: {:.4f}cm".format(pixel_width * 50 / pixels))
        cv.imshow("Arm", frame)
        k = cv.waitKey(1)
        if k == 113 or k == 81:
            break
        if k == 13:
            # drive arm
            print("drive arm")

    cap.release()
    cv.destroyAllWindows()
