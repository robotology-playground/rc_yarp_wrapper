# rc_yarp_wrapper

## References
[Official Documents](http://doc.rc-visard.com/en/gigevision.html#)

## Dependencies
1. [Yarp](https://github.com/robotology/yarp)
2. [rc_genicam_api](https://github.com/roboception/rc_genicam_api)

## Build
1. Build and Install **rc_genicam_api**. Remember to add its library path to the *.bashrc* file, e.g. in the case **rc_genicam_api** has been installed in `~/.local`
```
	export LIBRARY_PATH=$LIBRARY_PATH:~/.local/lib:~/.local/lib/rc_genicam_api
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.local/lib:~/.local/lib/rc_genicam_api
```
2. Build and install this repo normally

## How to
1. Run [rcdiscover-gui](https://github.com/roboception/rcdiscover) to look for the camera(s)
2. Run `gc_config -l` to get the name of camera(s) which you want to connect to.
3. Run this wrapper:
    ```
    rc_yarp_wrapper --mono 1 --combined 0 --confidence 1 --disp 1 --scale 2
    ```
    with *important* parameters:
    - mono: enable/disable streaming monochrome images from single camera
    - combined: enable/disable streaming monochrome images from two cameras (stacked vertically)
    - confidence: enable/disable streaming confidence images of disparity algorithm
    - disp: enable/disable streaming disparity images, which allows the depth calculation
    - scale: scaling factor to reduce images size for efficient streaming. Original images have size of 1280x960
  
4. Obtain depth estimate of a `Rectangular` surrounding a pixel by using `rpc` command:
    ```
    yarp rpc /rc_img_conveyor/rpc
    ```
    Then:
    ```
    Rect tlx tly width height step
    ```
  
