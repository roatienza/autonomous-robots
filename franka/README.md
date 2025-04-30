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
python3 rest.py <x> <y> <z> <thetax> <thetay> <thetaz>
# try with small values in meter for x, y and z. Still have bugs on thetax,y,z
```
