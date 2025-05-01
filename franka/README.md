### REST API for Franka Research Robot

Build the server:

```
cd cpp
mkdir build
cd build
cmake ..
# make sure no errors
make
# if no errors
./motion_server
```

Test using python:

```
# under franka folde
cd python
python3 rest.py <x> <y> <z> <time> <thetaz> <thetay> <thetax>
# try with small values in meter for x, y and z. Time in sec. Use slow time like 5. Still have bugs on thetax,y,z
```

### Intel RealSense D457

By default, D457 interface is GMSL FAKRA. To enable USB-C, transfer the switch from MPI to USB using this [guide](https://support.intelrealsense.com/hc/en-us/community/posts/14840675121043-RealSense-D457-USBC-mode-camera-not-detected-Ubuntu-22-04).
