import argparse
import os
import sys
from time import sleep
from typing import Optional, List
import cv2
import cv2.typing as cvt
from dt_apriltags import Detector as at_Detector
from dt_apriltags import Detection

from . import Calibrator
from . import Camera

class Detector:
    def __init__(self, tag_size_mm: Optional[float] = None, calibrator: Optional[Calibrator] = None):
        self.at_detector = at_Detector(families='tag36h11', #Using tagStandard52h13 crashes here :(
                                       nthreads=1,
                                       quad_decimate=1.0,
                                       quad_sigma=0.0,
                                       refine_edges=1,
                                       decode_sharpening=0.25,
                                       debug=0)
        self.tag_size = tag_size_mm # TODO: Verify whether this is in m or mm
        self.calibrator = calibrator

    def detect(self, img, calibration: Optional[Calibrator] = None) -> List[Detection]:
        cal = None
        if self.calibrator is not None:
            cal = self.calibrator
        if calibration is not None:
            cal = calibration
        
        if cal is not None:
            if self.tag_size is None:
                print("No tag size set, cannot estimate pose")
            else:
                return self.at_detector.detect(img,
                                           estimate_tag_pose=True,
                                           camera_params=cal.get_camera_params(),
                                           tag_size=self.tag_size)
        return self.at_detector.detect(img)

    @staticmethod
    def overlay_tags(img, tags, output_filename:Optional[str]=None, input_color=False, show: bool=False) -> cvt.MatLike:
        color_img = img if input_color else cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                         (0, 255, 0), 5)

            cv2.putText(color_img, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))
        if output_filename is None:
            if show:
                cv2.imshow(f'Detected tags for image', color_img)
                cv2.waitKey(1)
                # sleep(2)
        else:
            output_path = os.path.join('../output', output_filename)
            output_dir = os.path.dirname(output_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            if not cv2.imwrite(output_path, color_img):
                print(f"DIDN'T SAVE {output_path}")
        return color_img

    def run_from_camera(self):
        camera = Camera()
        img = camera.get_image()
        tags = self.detect(img)  # Detecting the tag
        return tags

    def run_from_file(self):
        input_folder = 'images'
        output_folder = 'detect_test'

        image_filenames = [file for file in os.listdir(input_folder)]
        # images = ['./tag52_13_00006_bordered_close.png', './tag52_13_00006_bordered_gap.png']
        tags_list = {}
        for image in image_filenames:
            print(f'Processing: {image}')
            img = cv2.imread(os.path.join(input_folder, image), cv2.IMREAD_GRAYSCALE)

            if img is None:
                print("ERROR: Image is none")
                continue
            tags = self.detect(img)  # Detecting the tag
            self.overlay_tags(img, tags, os.path.join(output_folder, image))
            tags_list[image] = tags
        return tags_list


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Program that detects april tags from an image")
    parser.add_argument('-c', '--camera', action='store_true', help='Use the Basler GbE camera as the input',
                        required=False)
    parser.add_argument('-s', '--size', help='Specifies the tag size in m', required=False, default=0.05,
                        type=lambda x: float(x))

    args = parser.parse_args()
    detector = Detector(args.size)
    if args.camera:
        detector.run_from_camera()
    else:
        detector.run_from_file()
