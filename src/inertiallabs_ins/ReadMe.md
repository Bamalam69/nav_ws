# InertialLabs INS

## Can't access the Serial Port?
You've gotta make sure the USB port it is plugged into has read and write previledges. One way to do this is by invoking:
```bash
sudo chmod -R 777 /dev/ttyUSB0
```