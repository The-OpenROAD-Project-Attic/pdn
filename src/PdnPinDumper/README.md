# PdnPinDumper
**PdnPinDumper** is a temporary utility to get MACRO pin location for PDN construction.
It is based on TritonRoute database and will be replaced by 
[OpenROAD](https://theopenroadproject.org/) database once available.

## Installation ##
PdnPinDumper is tested in 64-bit CentOS 6/7 environments with the following
prerequisites:
* A compatible C++ compiler supporting C++17 (GCC 7 and above)
* Boost >= 1.68.0
* Bison >= 3.0.4
* zlib >= 1.2.7
* CMake >= 3.1

To install PdnPinDumper:
```
$ cd PdnPinDumper
$ mkdir build
$ cd build
$ cmake -DBOOST_ROOT=<BOOST_ROOT> ../
$ make
```
   
## Supported Technologies ##
* CLN65LP (with limited selection of macros)

## License ##
* [BSD 3-clause License](LICENSE) 

