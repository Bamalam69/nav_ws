from __future__ import annotations
from pypylon import pylon
from typing import Optional
# from . import Calibrator
import cv2
import glob
import numpy as np
import pickle
from typing_extensions import Self

CAMERA_SERIAL = '23884525'

NUM_CALIBRATION_IMAGES = 10

class Camera:
    def __init__(self, calibrator: Optional[Calibrator] = None, color=True, auto=True, ip_address="192.168.3.3"):
        self.color = color
        self.auto_exposure = auto
        self.calibrator = calibrator

        info = pylon.DeviceInfo()
        info.SetPropertyValue('IpAddress', ip_address)

        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        print(f"Number of devices: {len(devices)}")

        camera_device = None
        for cam in devices:
            if cam.GetSerialNumber() == CAMERA_SERIAL:
                camera_device = cam
                break

        if camera_device is None:
            raise Exception('Could not select camera')

        print(f'Selected camera {camera_device.GetModelName()} {camera_device.GetSerialNumber()}')

        self.camera = pylon.InstantCamera(tl_factory.CreateDevice(camera_device))
        self.camera.Open()
        self._configure_camera()
        new_width = self.camera.Width.GetValue() - self.camera.Width.GetInc()
        if new_width >= self.camera.Width.GetMin():
            self.camera.Width.SetValue(new_width)

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    # def start_grabbing(self):
    #     self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages)

    @property
    def height(self):
        return self.camera.Height.GetValue()
    
    @property
    def width(self):
        return self.camera.Width.GetValue()

    @property
    def intrinsics(self):
        if self.calibrator is not None:
            return self.calibrator.camera_matrix.flatten().tolist()
        return [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        ]
    
    @property
    def distortion_parameters(self):
        if self.calibrator is not None:
            return self.calibrator.dist.flatten().tolist()
        return [0, 0, 0, 0, 0]

    @property
    def rectification(self):
        # Monocular camera, so no rectification
        return list(map(float,[
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        ]))

    @property
    def projection_matrix(self):
        if self.calibrator is not None:
            return list(map(float, [
                self.calibrator.camera_matrix[0][0], 0, self.calibrator.camera_matrix[0][2], 0,
                0, self.calibrator.camera_matrix[1][1], self.calibrator.camera_matrix[1][2], 0,
                0, 0, 1, 0
            ]))
        return [
            1, 0, 0, 0,
            0, 1, 0, 0, 
            0, 0, 1, 0
        ]

    def _configure_camera(self):
        if self.auto_exposure:
            self.camera.ExposureAuto = 'Continuous'
            self.camera.GainAuto = 'Continuous'
        else:
            self.camera.GainAuto = 'Off'
            self.camera.GainRaw = 100
            self.camera.ExposureAuto = 'Off'
            self.camera.ExposureTimeRaw = 10000
        self.camera.BalanceWhiteAuto = 'Continuous'
        # print(self.camera.PixelFormat.Symbolics)
        self.camera.PixelFormat = 'BayerRG8' if self.color else 'Mono8'
        print(self.camera.PixelFormat.GetValue())

    def get_image(self):
        # if self.camera.IsGrabbing():
        #     grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        #     if grab_result.GrabSucceeded():
        #         return self.converter.Convert(grab_result).GetArray()
        if not hasattr(self, 'camera'):
            print("Not connected to a camera, cannot get image!")
            return None
        while True:
            img = self.camera.GrabOne(5000).GetArray()
            if img.shape[0] == 0 or img.shape[1] == 0:
                print("Warning: Got invalid image, trying again")
                continue
            break
        return img

    def close_camera(self):
        if hasattr(self, 'camera') and self.camera.IsOpen():
            self.camera.Close()
        else:
            print("Warning: Tried to close camera that wasn't open, did nothing")

