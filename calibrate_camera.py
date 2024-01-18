from basler_lib import Calibrator

def main():
    calibrator = Calibrator()
    calibrator.calibrate(count=5)
    calibrator.save_calibration('calibration_20.pickle')

if __name__ == "__main__":
    main()