todo

- lib to be shared by apps, containing classes Camera, Plane, PointCloud, ...
- split code into following apps:
  - calibrateCam
    - input: board params, images for cam_0[, images for cam_1]
	- output: cam_0.yml[, cam_1.yml] (note that extrinsics given in camera coordinate frame of cam_0)
  - calibrateProj
    - input: images, board params, point params, cam_0.yml[, --homography flag]
	- output: proj.yml[, homography_<i>.yml per input image i]
  - applyHomography
    - input: image, homography YML output by calibrateProj
	- output: warped image
	
Note that image center will be fixed point (= stays fixed after applying homography)