## Advanced Lane Finding
Udacity - Self-Driving Car NanoDegree

The objective is to detect the lanes from the image/video and find the possible path for the car to move between the lanes. The steps followed to achieve this are :

* Perform camera calibration - Find Camera matrix and the distortion coefficient through checkerboard images.
* Threshold images to extract only lanes from image.
* Transform images to different perspective for focusing lanes.
* Detect lane pixels and fit to a polynomial function.
* Determine the curvature of the lane and car position offset value with respect to center.
* Highlight path of the car between the lanes.

### Camera Calibration

Due to radial distortion, straight lines will appear curved. Its effect is more as we move away from the center of image. Similarly, another distortion is the tangential distortion which occurs because image taking lens is not aligned perfectly parallel to the imaging plane. So some areas in image may look nearer than expected.

For stereo applications, these distortions need to be corrected first. To find all these parameters, what we have to do is to provide some sample images of a well defined pattern (eg, chess board). We find some specific points in it ( square corners in chess board). We know its coordinates in real world space and we know its coordinates in image. 

Images are taken from a static camera and chess boards are placed at different locations and orientations. So we need to know (X,Y,Z) values. With the knowledge of point location in 3D, the corresponding 2D points can be taken and used for finding the camera parameters. 

 `cv2.calibrateCamera(objpoints, imgpoints, image-shape[::-1],None,None)` is the OpenCV funtion to find camera parameters. 3D points are called object points and 2D image points are called image points.
 
 ![distorted](https://user-images.githubusercontent.com/37708330/46498569-13b4f300-c81e-11e8-9d8c-4ea37ac46448.png)

### Thresholded Output

Now our aim is to detect only the lanes´ from the entire image. Various methods of thresholding can be applied for this image. I have used combinations of different thresholds. It could be based on gradient magnitude, direction or color space. Similarly, there exist different color space (an abstract mathematical model which simply describes the range of colors as tuples of numbers, typically as 3 or 4 values or color components e.g. RGB). 

Some of the color space are :
 * HSV 
 * HSL
 * LUV
 * LAB
 * YPbPr 
 * YCbCr
 * ICtCp
 * CMYK
 * YIQ
 * YUV
 * YDbDr
 
![threshold](https://user-images.githubusercontent.com/37708330/46498313-5f1ad180-c81d-11e8-83bd-a95794e3bf08.png)

### Prespective Transform and Fit Curve

The goal of this step is to transform the undistorted image to a "birds eye view" of the road which focuses only on the lane lines and displays them in such a way that they appear to be relatively parallel to eachother (as opposed to the converging lines you would normally see). To achieve the perspective transformation I first applied the OpenCV functions `getPerspectiveTransform` and `warpPerspective` which take a matrix of four source points on the undistorted image and remaps them to four destination points on the warped image. The source and destination points were selected manually by visualizing the locations of the lane lines on a series of test images.

Now we have to find a position of left or right line at the bottom of binary warped image via detection of peaks in computed histogram for bottom part of binarized image (the bottom half of binarized image). The calculated histogram is smoothed by gaussian filter and then is used for peak detection with some thresholding: one for noise filtering and other for filtering an expected distance between detected peak and expected position of line at the bottom of image. As a result, the function returns the x value of detected peak, which is used as starting point for lane detection along vertical Y direction.

`find_lane_pixels` function is to find the pixels which contributes to the lane pixels. With that information, wefit a second order polynomial to each lane line using `np.polyfit`.

![curve_identified](https://user-images.githubusercontent.com/37708330/46499622-e0279800-c820-11e8-9762-42bf332e5bfd.png)

### Calcualte Curvature and Offset from Centre

With the detected polynomial function, we calculated the meters space to be used here to calculate the curvature. To find the vehicle position on the center by a second order polynomial f(y)=A y^2 +B y + C, the radius of curvature is given by R = [(1+(2 Ay +B)^2 )^3/2]/|2A|. I have used the offset, curve_radius function to calculate the parameters.

### Highlight Path Between Lanes

With the detected polynomial function, we calculate the range of the lane pixels (left and right lanes). With the position,  this function `cv2.fillPoly(lane_area, points, (0,255, 0))` forms the polygon with the specified color. The polygon formed gets updated whenever the detected lane position changes. So accordingly the path of the lane gets adjusted with respect to the lane curvature. 

![lane area detected](https://user-images.githubusercontent.com/37708330/46499619-df8f0180-c820-11e8-9c78-a963ce45e709.png)

### Discussion

