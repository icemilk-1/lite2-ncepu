import argparse

import numpy as np
import cv2 as cv

from mp_persondet import MPPersonDet

def str2bool(v):
    if v.lower() in ['on', 'yes', 'true', 'y', 't']:
        return True
    elif v.lower() in ['off', 'no', 'false', 'n', 'f']:
        return False
    else:
        raise NotImplementedError

backends = [cv.dnn.DNN_BACKEND_OPENCV, cv.dnn.DNN_BACKEND_CUDA]
targets = [cv.dnn.DNN_TARGET_CPU, cv.dnn.DNN_TARGET_CUDA, cv.dnn.DNN_TARGET_CUDA_FP16]
help_msg_backends = "Choose one of the computation backends: {:d}: OpenCV implementation (default); {:d}: CUDA"
help_msg_targets = "Chose one of the target computation devices: {:d}: CPU (default); {:d}: CUDA; {:d}: CUDA fp16"
try:
    backends += [cv.dnn.DNN_BACKEND_TIMVX]
    targets += [cv.dnn.DNN_TARGET_NPU]
    help_msg_backends += "; {:d}: TIMVX"
    help_msg_targets += "; {:d}: NPU"
except:
    print('This version of OpenCV does not support TIM-VX and NPU. Visit https://github.com/opencv/opencv/wiki/TIM-VX-Backend-For-Running-OpenCV-On-NPU for more information.')

parser = argparse.ArgumentParser(description='Person Detector from MediaPipe')
parser.add_argument('--input', '-i', type=str, help='Usage: Set path to the input image. Omit for using default camera.')
parser.add_argument('--model', '-m', type=str, default='./person_detection_mediapipe_2023mar.onnx', help='Usage: Set model path, defaults to person_detection_mediapipe_2023mar.onnx.')
parser.add_argument('--backend', '-b', type=int, default=backends[0], help=help_msg_backends.format(*backends))
parser.add_argument('--target', '-t', type=int, default=targets[0], help=help_msg_targets.format(*targets))
parser.add_argument('--score_threshold', type=float, default=0.5, help='Usage: Set the minimum needed confidence for the model to identify a person, defaults to 0.5. Smaller values may result in faster detection, but will limit accuracy. Filter out persons of confidence < conf_threshold.')
parser.add_argument('--nms_threshold', type=float, default=0.3, help='Usage: Suppress bounding boxes of iou >= nms_threshold. Default = 0.3.')
parser.add_argument('--save', '-s', type=str, default=False, help='Usage: Set “True” to save file with results (i.e. bounding box, confidence level). Invalid in case of camera input. Default will be set to “False”.')
parser.add_argument('--vis', '-v', type=str2bool, default=True, help='Usage: Default will be set to “True” and will open a new window to show results. Set to “False” to stop visualizations from being shown. Invalid in case of camera input.')
args = parser.parse_args()

def visualize(image, results, fps=None):
    output = image.copy()

    if fps is not None:
        cv.putText(output, 'FPS: {:.2f}'.format(fps), (0, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

    for idx, person in enumerate(results):
        score = person[-1]
        person_landmarks = person[4:-1].reshape(4, 2).astype(np.int32)

        hip_point = person_landmarks[0]
        full_body = person_landmarks[1]
        shoulder_point = person_landmarks[2]
        upper_body = person_landmarks[3]

        # draw circle for full body
        radius = np.linalg.norm(hip_point - full_body).astype(np.int32)
        cv.circle(output, hip_point, radius, (255, 0, 0), 2)

        # draw circle for upper body
        radius = np.linalg.norm(shoulder_point - upper_body).astype(np.int32)
        cv.circle(output, shoulder_point, radius, (0, 255, 255), 2)

        # draw points for each keypoint
        for p in person_landmarks:
            cv.circle(output, p, 2, (0, 0, 255), 2)

        # put score
        cv.putText(output, 'Score: {:.4f}'.format(score), (0, output.shape[0] - 48), cv.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

    cv.putText(output, 'Yellow: upper body circle', (0, output.shape[0] - 36), cv.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))
    cv.putText(output, 'Blue: full body circle', (0, output.shape[0] - 24), cv.FONT_HERSHEY_DUPLEX, 0.5, (255, 0, 0))
    cv.putText(output, 'Red: keypoint', (0, output.shape[0] - 12), cv.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

    return output

if __name__ == '__main__':
    # Instantiate MPPersonDet
    model = MPPersonDet(modelPath=args.model,
                      nmsThreshold=args.nms_threshold,
                      scoreThreshold=args.score_threshold,
                      backendId=args.backend,
                      targetId=args.target)

    # If input is an image
    if args.input is not None:
        image = cv.imread(args.input)

        # Inference
        results = model.infer(image)
        if len(results) == 0:
            print('Person not detected')

        # Draw results on the input image
        image = visualize(image, results)

        # Save results if save is true
        if args.save:
            print('Resutls saved to result.jpg\n')
            cv.imwrite('result.jpg', image)

        # Visualize results in a new window
        if args.vis:
            cv.namedWindow(args.input, cv.WINDOW_AUTOSIZE)
            cv.imshow(args.input, image)
            cv.waitKey(0)
    else: # Omit input to call default camera
        deviceId = 0
        cap = cv.VideoCapture(deviceId)

        tm = cv.TickMeter()
        while cv.waitKey(1) < 0:
            hasFrame, frame = cap.read()
            if not hasFrame:
                print('No frames grabbed!')
                break

            # Inference
            tm.start()
            results = model.infer(frame)
            tm.stop()

            # Draw results on the input image
            frame = visualize(frame, results, fps=tm.getFPS())

            # Visualize results in a new Window
            cv.imshow('MPPersonDet Demo', frame)

            tm.reset()

