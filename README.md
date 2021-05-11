# cpsspatialar

![](docs/splash.png)


## Building the Code in Visual Studio 2019

Cf. the last section of the [docs](docs/docs.pdf)...

## Sample Invocations

Cf. [here](docs/sample_invocations.txt) (find the corresponding sample images [here](docs/sample_data))...

## Navigating the 3D Viewer of `calibrateProj`

Besides serving to calibrate the projector and compute homographies, `calibrateProj` enables visualizing the geometric setup for a selected input calibration image (indicated using the `visImIdx` argument); important keys for navigating the 3D viewer:

* F1: align OpenGL viewport with currently selected camera/projector
* F2: switch to next camera/projector; order: (i) camera, (ii) projector, (iii) virtual projector (obtained by getting projector to point downwards to ground plane by rotating about intersection point with ground plane of projector's view direction), (iv) final virtual projector (virtual projector additonally rotated with respect to axes of camera and optionally placed lower towards ground plane)
* F3: display various distances
* F4: display center points of circles pattern as projected to ground plane by (i) projector (red) or by (ii) final virtual projector (green) 
* ESC: exit
