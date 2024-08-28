# collision_check

## Installation

### install pybullet
```bash
pip install pybullet
```

### Clone the Repository

```bash
git clone https://github.com/shez12/collision_check.git
cd collision_check
git clone https://github.com/ompl/ompl.git
```
# Visit [OMPL](https://github.com/ompl/ompl) GitHub Repository and install dependencies

    Boost (version 1.58 or higher)
    CMake (version 3.12 or higher)
    Eigen (version 3.3 or higher)
    Py++ (needed to generate Python bindings)
    Doxygen (needed to create a local copy of the documentation at https://ompl.kavrakilab.org/core)
    Flann (FLANN can be used for nearest neighbor queries by OMPL)
    Spot (Used for constructing finite automata from LTL formulae.)

# cmake 
```bash
cd ompl
mkdir build/Release
cd build/Release
cmake ../.. -DPYTHON_EXECUTABLE=/usr/bin/python3.8 # your python address
make -j 4 update_bindings 
make -j 4 # Replace "4" with the number of cores on your machine
```

```bash
# Export OMPL Python bindings to Python environment
export PYTHONPATH=/home/usr/Desktop/collision_check/ompl/py-bindings:$PYTHONPATH
```



## Reference
- [OMPL GitHub Repository](https://github.com/ompl/ompl)
- [pybullet_ompl GitHub Repository](https://github.com/lyfkyle/pybullet_ompl)
