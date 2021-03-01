# windy_guidance_test_bed (WIP)
Python bound environment for testing wind-aware guidance logic (programmed in c++) for fixed-wing UAVs.

:: Ubuntu 20.04, Python3.

## Prerequisites
Pybind11 is a header library that can be used to wrap c/c++ functions for exploitation in python scripts.
For more info [read the docs](https://pybind11.readthedocs.io/en/stable/basics.html).
Pybind11 needs to either be included locally within any Python project directory, or installed with your choice of Python environment.
To get things running while minimizing headache, follow these steps:

For whatever reason, cmake won't find the package unless installed with conda (using pip install doesn't work)

1) Install miniconda3: https://conda.io/projects/conda/en/latest/user-guide/install/linux.html (or whatever conda package you like)

2) Install pybind11 using conda: ``conda install -c conda-forge pybind11``

3) When wanting to work on/with these scripts, ``conda activate base`` from the terminal (or whatever other environment you alternatively setup/name)

Now the ``find_package(pybind11 REQUIRED)`` in the CMakeLists.txt will be successful, assuming you **operate within a conda environment**.

NOTE: once done working within the conda environment - ``conda deactivate``

More dependencies (again within conda env):
```
sudo apt install libeigen3-dev
conda install -c anaconda numpy
conda install -c conda-forge matplotlib
```

## Build the python bindings / c++ binaries with cmake
From the root directory:
```
mkdir build
cd build
cmake ..
make cpp_functions
```

To run tests...
```
cd build
make test_path
./test_path
```

## Run the script(s)
e.g.
``python3 pychecks/test_line_path_bindings.py``
