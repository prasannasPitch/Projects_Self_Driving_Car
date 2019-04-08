## Advanced Lane Finding
Udacity - Self-Driving Car NanoDegree

![bw](https://user-images.githubusercontent.com/37708330/53917991-200bd900-4066-11e9-81cd-692177d466dd.png)

## Table Content: ##
- [Motivation Lane Detection](#motivation)
- [General Workflow](#imp)
- [Camera Calibration](#psu)
- [Thresholded Output](#thresh)
- [Prespective Transform](#trans)
- [Find Lane pixels](#trans)
- [Find LaneCalcualte Curvature and Offset from Centrepixels](#curve)
- [Convolution Approach](#conv)
- [Result](#result)
- [Files](#files)


 <a name="motivation"></a>
### Motivation Lane Detection

<p align="justify">
The lines drawn on roads indicate to human drivers where the lanes are and act as a guiding reference to which direction to steer the vehicle accordingly and convention to how vehicle agents interact harmoniously on the road. Similarly for a self driving car, the ability to detect the lanes and navigate through  streets/highways is essential. Dedicated lane detection involving segmentation algorithms proved good for these kind of approaches. I have implememented one possible solution for lane detection and complex semantic segmentation methods can be built on this.  </p>

 <a name="imp"></a>
### General Workflow

The objective is to detect the lanes from the image/video and find the possible path for the car to move between the lanes. The steps followed to achieve this are :

* Perform camera calibration - Find Camera matrix and the distortion coefficient through checkerboard images.
* Threshold images to extract only lanes from image.
* Transform images to different perspective for focusing lanes.
* Detect lane pixels and fit to a polynomial function.
* Determine the curvature of the lane and car position offset value with respect to center.
* Highlight path of the car between the lanes.


 <a name="psu"></a>
### Camera Calibration
<p align="justify">
Due to radial distortion, straight lines will appear curved. Its effect is more as we move away from the center of image. Similarly, another distortion is the tangential distortion which occurs because image taking lens is not aligned perfectly parallel to the imaging plane. So some areas in image may look nearer than expected.

For stereo applications, these distortions need to be corrected first. To find all these parameters, what we have to do is to provide some sample images of a well defined pattern (eg, chess board). We find some specific points in it ( square corners in chess board). We know its coordinates in real world space and we know its coordinates in image. 

Images are taken from a static camera and chess boards are placed at different locations and orientations. So we need to know (X,Y,Z) values. With the knowledge of point location in 3D, the corresponding 2D points can be taken and used for finding the camera parameters. 
 </p>

 `cv2.calibrateCamera(objpoints, imgpoints, image-shape[::-1],None,None)` is the OpenCV funtion to find camera parameters. 3D points are called object points and 2D image points are called image points.
 
 ![distorted](https://user-images.githubusercontent.com/37708330/46498569-13b4f300-c81e-11e8-9d8c-4ea37ac46448.png)

 <a name="thresh"></a>
### Thresholded Output
<p align="justify">
Now our aim is to detect only the lanes´ from the entire image. Various methods of thresholding can be applied for this image. I have used combinations of different thresholds. It could be based on gradient magnitude, direction or color space. Similarly, there exist different color space (an abstract mathematical model which simply describes the range of colors as tuples of numbers, typically as 3 or 4 values or color components e.g. RGB).   </p>

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


 <a name="trans"></a>
### Prespective Transform
<p align="justify">
The goal of this step is to transform the undistorted image to a "birds eye view" of the road which focuses only on the lane lines and displays them in such a way that they appear to be relatively parallel to eachother (as opposed to the converging lines you would normally see). To achieve the perspective transformation I first applied the OpenCV functions `getPerspectiveTransform` and `warpPerspective` which take a matrix of four source points on the undistorted image and remaps them to four destination points on the warped image. The source and destination points were selected manually by visualizing the locations of the lane lines on a series of test images. </p>

![image](https://user-images.githubusercontent.com/37708330/55750667-151cdd80-5a44-11e9-9aee-9135e910babc.png)

 <a name="lane"></a>
### Find Lane pixels
<p align="justify">
Now we have to find a position of left or right line at the bottom of binary warped image via detection of peaks in computed histogram for bottom part of binarized image (the bottom half of binarized image). The calculated histogram is smoothed by gaussian filter and then is used for peak detection with some thresholding: one for noise filtering and other for filtering an expected distance between detected peak and expected position of line at the bottom of image. As a result, the function returns the x value of detected peak, which is used as starting point for lane detection along vertical Y direction. In order to detect the lane pixels from the warped image, the following steps are performed.  </p>

1. A histogram of the lower half of the warped image is created. 

![image](https://user-images.githubusercontent.com/37708330/55748238-688c2d00-5a3e-11e9-9fdf-b91124c03f12.png)

`find_lane_pixels` function is to find the pixels which contributes to the lane pixels. `np.polyfit` method is used to fit a polynomial for the detected lane pixels. 

2. By finding the peak of the histogram, we find the starting point for searching our lane pixels.
3. By Sliding Window technique, we identify the most likely coordinates of the lane lines in a window, which slides vertically through the image for both the left and right line.
4. The coordinates previously calculated, a second order polynomial is calculated for both the left and right lane line. Numpy's function np.polyfit will be used to calculate the polynomials.

![curve_identified](https://user-images.githubusercontent.com/37708330/46499622-e0279800-c820-11e8-9762-42bf332e5bfd.png)

![image](https://user-images.githubusercontent.com/37708330/55750498-b3f50a00-5a43-11e9-9801-73f248a205b8.png)

 <a name="curve"></a>
### Calcualte Curvature and Offset from Centre
<p align="justify">
With the detected polynomial function, we calculated the meters space to be used here to calculate the curvature. To find the vehicle position on the center by a second order polynomial f(y)=A y^2 +B y + C, the radius of curvature is given by R = [(1+(2 Ay +B)^2 )^3/2]/|2A|. I have used the offset, curve_radius function to calculate the parameters.

With the detected polynomial function, we calculate the range of the lane pixels (left and right lanes). With the position,  this function `cv2.fillPoly(lane_area, points, (0,255, 0))` forms the polygon with the specified color. The polygon formed gets updated whenever the detected lane position changes. So accordingly the path of the lane gets adjusted with respect to the lane curvature. 
 </p>
 
![lane area detected](https://user-images.githubusercontent.com/37708330/46499619-df8f0180-c820-11e8-9c78-a963ce45e709.png)

 <a name="conv"></a>
### Convolution Approach

Another way to approach the sliding window method is to apply a convolution, which will maximize the number of "hot" pixels in each window. A convolution is the summation of the product of two separate signals, in our case the window template and the vertical slice of the pixel image.

We slide our window template across the image from left to right and any overlapping values are summed together, creating the convolved signal. The peak of the convolved signal is where there was the highest overlap of pixels and the most likely position for the lane marker.

#### Update :

* By further tuning the threshold and taking into consideration of intercept values, the challenge video is almost solved. Files can be seen in the "Challenge folder".

  <a name="result"></a>
 ### Result

![ezgif com-video-to-gif 1](https://user-images.githubusercontent.com/37708330/46537145-d6e60c00-c8b0-11e8-8e56-95864f0eb998.gif)

Full Project Video : [Video Link](https://youtu.be/oyZ-jrVh1gE)  
Full Challenge Video :[Video Link](https://youtu.be/tLq8MKu_7Lw)

 <a name="files"></a>
## Files

* Pipeline_Image.ipynb for Images
* Pipeline_Video.ipynb for Video
* Pipeline_ChallengeVideo.ipynb for challenge video
