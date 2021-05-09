# cpsspatialar

![](docs/splash.png)


## Building the Code

Cf. the last section of the [docs](https://github.com/m-hornacek/cpsspatialar/blob/main/docs/docs.pdf)

## Sample Invocations

```
cd C:\Users\micha\Desktop\spatial-ar\cpsspatialar\src\build\bin\Release


# SPLIT UP ZED RASTERS

splitZed.exe C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\in\*.png C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outRight

splitZed.exe C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\in\*.png C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outRight


# CALIBRATE (STEREO) CAMERA

calibrateCam.exe 0.0565 4 6 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out 10 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outRight


# CALIBRATE PROJECTOR

calibrateProj.exe 0.0565 4 6 4 11 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out 14 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\acircles_pattern_960x600.png 1.5


# APPLY HOMOGRAPHIES

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_0.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_1.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_1.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_2.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_2.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_3.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_3.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_4.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_4.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_5.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_5.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_6.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_6.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_7.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_7.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_8.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_8.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_9.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_9.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_10.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_10.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_11.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_11.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_12.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_12.png

applyHomography.exe C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_13.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\out\holodeck_13.png
```
