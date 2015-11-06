##### ara-fw-test-public

1. Download NDK  
```
http://dl.google.com/android/ndk/android-ndk-r10e-linux-x86_64.bin  
```
2. Make it executable  
```
chmod a+x android-ndk-r10e-linux-x86_64.bin  
```
3. Install NDK  
```
./android-ndk-r10e-linux-x86_64.bin  
```
4. Extract toolchain  
```
sudo /home/<USER>/android-ndk-r10e/build/tools/make-standalone-toolchain.sh \  
   --toolchain=aarch64-linux-android-4.9 --system=linux-x86_64 --platform=android-21 \  
   --install-dir=/opt/android-toolchain-aarch64-4.9-host-linux-x86_64-android-21  
```
5. Make toolchain runnable  
```
cd /opt  
sudo chmod -R +x android-toolchain-aarch64-4.9-host-linux-x86_64-android-21  
```
6. Clone and build this repository  
```
git clone https://github.com/darryln/ara-fw-test-public.git  
cd ara-fw-test-public  
make all  
```
7. You can also `make --always-make` to force rebuild, and `make clean`.  
8. Test app executables are placed under `ara-fw-test-public/build/apps/`.  

##### Notes
* Functional test apps dump the command line args by calling dumpargs, which is part of the common test app library, libfwtest.a.

* To add code to libfwtest.a, put your .c file in apps/lib and declare the functions in apps/lib/include/libfwtest.h.  All .c files under apps/lib get built into libfwtest.a automatically, no need to update the Makefile for it.

* New test apps can be created under apps/greybus, apps/stress, apps/performance, and apps/other
  * mkdir <name of test app>
  * copy an existing test app and Makefile there
  * modify Makefile and source files as needed.
  * There is no need to modify higher-level Makefiles, the newly added test app will get built automatically.
