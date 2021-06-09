## Setup Samba

```bash
sudo nano /etc/samba/smb.conf
```

```bash
[no_password_folder]
    path = /home
    writeable = yes
    guest ok = yes
    writable = yes
    available = yes
    browsable = yes
    public = yes
    force user = root
```


## C Libraries

```c
#ifdef __MINGW32__
//pacman -S mingw64/mingw-w64-x86_64-openblas
//-lopenblas
#include <OpenBLAS/lapack.h>
#include <OpenBLAS/lapacke.h>
#include <OpenBLAS/cblas.h>
#else
//sudo apt-get install libblas-dev
//sudo apt-get install libopenblas-dev
//sudo apt-get install liblapacke-dev
#include <lapacke.h>
#include <cblas.h>
#endif
```




## Installing on Windows MSYS2

* https://www.msys2.org/


```
pacman -S mingw-w64-x86_64-qt-creator
pacman -S mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-make
pacman -S mingw-w64-x86_64-boost
```

```
git clone https://github.com/nanomsg/nng
mkdir build
cd build
cmake .. -G "MinGW Makefiles" -DCMAKE_INSTALL_PREFIX=$MINGW_PREFIX
mingw32-make
mingw32-make install
```





## Installing on Linux
```
sudo apt-get install libblas-dev
sudo apt-get install libcblas-dev
sudo apt-get install liblapacke-dev

sudo apt-get install cmake
git clone https://github.com/nanomsg/nng
mkdir build
cd build
sudo cmake ..
sudo make
sudo make test
sudo make install
sudo ldconfig


```