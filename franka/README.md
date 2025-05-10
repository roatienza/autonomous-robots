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

Test using python.
**NOTE** : API changes. Instead of absolute angles in randians, the angle parameters are in delta of the init angles. All in degrees.
```
# under franka folder
cd python
# python3 rest.py <x> <y> <z> <time> <delta_tetha_z_deg> <delta_tetha_y_deg> <delta_tetha_x_deg>
python3 rest.py --parameters 0.6 0 0.2 5.0 0 10 0
# try with small values in meter for x, y and z. Time in sec. Use slow time like 5. Still have bugs on thetax,y,z
```

### Intel RealSense D457

By default, D457 interface is GMSL FAKRA. To enable USB-C, transfer the switch from MPI to USB using this [guide](https://support.intelrealsense.com/hc/en-us/community/posts/14840675121043-RealSense-D457-USBC-mode-camera-not-detected-Ubuntu-22-04).
