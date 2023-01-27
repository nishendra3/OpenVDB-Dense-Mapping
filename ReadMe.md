<!-- ReadMe File For the VDB wapper  -->

# VDB Wrapper

This is a wrapper for the VDB library. It is a C++ library that provides a set of classes for manupulating sparse volumes.

## Features

* Read and write VDB files
* Create VDB files from point clouds
* Free Space query on point clouds using VDB
* nearest neighbor search on point clouds using VDB
* Unit tests for all the above features


## Installation

### Prerequisites

* CMake 3.0 or higher
* C++17 compiler
* PCL library 
* OpenVDB 4.0.2 or higher

### Installing OpenVDB from github

Follow the instructions on the OpenVDB github page to install the library.
URL: www.github.com/AcademySoftwareFoundation/openvdb

### Installing PCL from github

Follow the instructions on the PCL github page to install the library.
URL: www.github.com/PointCloudLibrary/pcl


### Build

```bash
mkdir build
cd build
cmake ..
make
```

## Running the tests

*   Change the path to the input directory in `main.cpp`
*   Set the test to run from the provided list of tests in `main.cpp`
* The list of tests are as follows:
    * `FileIO` - Tests the file IO functions
    * `DataAccess` - Tests the data access functions
    * `FreeSpaceQuery` - Tests the free space query functions
    * `NearestNeighborSearch` - Tests the nearest neighbor search functionality

## Bug Reporting

Please report any bugs or issues to the owner of this repository.



