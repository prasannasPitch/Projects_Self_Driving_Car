# Behavioral Cloning Project
Self-Driving Car Engineer Nanodegree Program

## Table Content: ##
- [Motivation for Behavioral Cloning](#motivation)
- [Collection of Training Data - Simulator](#sim)
- [Data Exploration](#data_explore)
- [Preprocessing Step](#preprocess)
- [Model Architecture](#model)
- [Result](#result)
- [Files](#files)

## Motivation for Behavioral Cloning <a name="motivation"></a>
<p align="justify">
While neural networks are responsible for recent breakthroughs in problems like computer vision, machine translation and time series prediction – they can also combine with techniques such as behavioral cloning to add up enough cognizance to any autonomous system. The end-to-end approach such as Nvidia’s BB8 trains a Deep Convolution Neural Network to generate steering angle based on the input camera observation. This CNN learns from the behavior of a human expert driver and tries to replicate the control of the driver for the learned situation. The technique to replicate the human behavior is called behavioral cloning. 
 </p>
 
 
 ## Collection of Training Data - Simulator <a name="sim"></a>
 
 <p align="justify">
 For collecting training data, we use the simulated environment given by Udacity Simulator. It has two operating modes in it : 1. Training mode - where we could drive the car through the tracks provided. 2. Autonomous mode - where the designed model is utilized to run the car autonomously. </p>
 
 ![image](https://user-images.githubusercontent.com/37708330/50380946-d5d7a600-0679-11e9-892f-46caeab8f234.png)


The training data should be collected by considering the below mentioned points.

- the car should stay in the center of the road as much as possible
- if the car veers off to the side, it should recover back to center
- driving counter-clockwise can help the model generalize
- flipping the images is a quick way to augment the data
- collecting data from the second track can also help generalize the model
- we want to avoid over fitting or under fitting when training the model
- knowing when to stop collecting more data


## Data Exploration <a name="data_explore"></a>

The simulator gives the frames of our driving in the selected directory.

- IMG folder - this folder contains all the frames of the driving.
- driving_log.csv - each row in this sheet correlates the image with the steering angle, throttle, brake, and speed of the car. In this project we use only the steering angle.

<img width="756" alt="driving-log-output" src="https://user-images.githubusercontent.com/37708330/50382704-f4ec2d00-06a5-11e9-977b-8124729816f6.png">


From the log file we could see that there are three images for a selected scene. This corresponds to three different cameras placed in left, right and center. For this project, recording recoveries from the sides of the road back to center is effective. But it is also possible to use all three camera images to train the model.

![output_14_0](https://user-images.githubusercontent.com/37708330/50383091-3e8c4600-06ad-11e9-8eff-ca401311a1c5.png)

## Preprocessing Step <a name="preprocess"></a>

In this step, we use two methods for preprocessing images. 

- Changing presprective for using multiple cameras
- Flipping images to avoid overfitting 
- Normalize the images with zero SD
- Crop Image to extract road

#### Changing presprective for using multiple cameras

 <p align="justify">
It is understandable that having more training images can improve the performance of the model. But images from the simulator are from different cameras placed in different places. a. For example, if we train the model to associate a given image from the center camera with a left turn, then we could also train the model to associate the corresponding image from the left camera with a somewhat softer left turn. And we could train the model to associate the corresponding image from the right camera with an even harder left turn. From the perspective of the left camera, the steering angle would be less than the steering angle from the center camera. From the right camera's perspective, the steering angle would be larger than the angle from the center camera.
</p>

![carnd-using-multiple-cameras](https://user-images.githubusercontent.com/37708330/50383212-59f85080-06af-11e9-896c-1655bbc172de.png)

 <p align="justify">
Therefore, for using all three images,  we want to feed the left and right camera images to the model as if they are coming from the center camera. This way, we can teach the model how to steer if the car drifts off to the left or the right. Figuring out how much to add or subtract from the center angle will involve some experimentation. Finally while driving in autonomous mode, we need to predict with the center camera image. In this project, for the given sample training data, I have used 0.2 (added to center steering angle for left camera images & subracted form center steering angle for right camera images).
 </p>
 
 #### Flipping images to avoid overfitting
 
  <p align="justify">
 Track one has a left turn bias. If the training data has only drive around the first track in a clock-wise direction, the data will be biased towards left turns. One way to combat the bias is to flip the images and negate the driving angle so that a balanced set of data is obtained. 
  </p>

![output_14_2](https://user-images.githubusercontent.com/37708330/50383345-0a675400-06b2-11e9-88c0-1bf99ef47ca1.png)

#### Normalize the images with zero SD
  <p align="justify">
In Keras, lambda layers can be used to create arbitrary functions that operate on each image as it passes through the layer. In this project, a lambda layer is a convenient way to parallelize image normalization. The lambda layer will also ensure that the model will normalize input images when making predictions in drive.py. That lambda layer could take each pixel in an image and run it through the formulas:  </p>

pixel_normalized = pixel / 255 

pixel_mean_centered = pixel_normalized - 0.5 

#### Crop Image to extract road
  <p align="justify">
Keras provides the Cropping2D layer for image cropping within the model. This is relatively fast, because the model is parallelized on the GPU, so many images are cropped simultaneously. By contrast, image cropping outside the model on the CPU is relatively slow. The Cropping2D layer might be useful for choosing an area of interest that excludes the sky and/or the hood of the car.

Here is an example of an input image and its cropped version after passing through a Cropping2D layer:  </p>

![image](https://user-images.githubusercontent.com/37708330/50383681-418d3380-06b9-11e9-88ef-0420d707c859.png)

## Model Architecture <a name="model"></a>


###  Model Overview

* This model is inspired from the model provided by NVIDIA as suggested by Udacity. The model architecture is described by [NVIDIA](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf). As an input this model takes in image of the shape (60,266,3) but our dashboard images/training images are of size (160,320,3).

![nvidia](https://user-images.githubusercontent.com/37708330/50383527-f6bdec80-06b5-11e9-9b18-2cd8bac97b0e.JPG)

### Batch Training

On analyzing the dataset, it was found that the sample dataset from udacity comprises of 9 laps of track 1 with recovery data. For splitting of data for validation set, 20% of training data is randomly selected and given to validation dataset. Finally, mini batch size of 32 images are taken for updating loss function. Generator function is used to avoid loading of all the images in the memory and instead generate it at the run time. 

### Layers

The final model consists of:
* 24@5x5 convolution filter with 5x5 stride and ELU activation.
* 36@5x5 convolution filter with 5x5 stride and ELU activation.
* 48@5x5 convolution filter with 5x5 stride and ELU activation.
* 64@3x3 convolution filter with 5x5 stride and ELU activation.
* 64@3x3 convolution filter with 5x5 stride and ELU activation.
* 100 node fully connected layer with ELU activation followed by a dropout.
* 50 node fully connected layer with ELU activation.
* 10 node fully connected layer with ELU activation.
* 1 node output layer.

### Hyper Parameters

- No of epochs= 5
- Optimizer Used- Adam
- Learning Rate- Default 0.001
- Validation Data split- 0.2
- Generator batch size= 32
- Correction factor- 0.2
- Loss Function Used- MSE (Regression problem)

## Result <a name="result"></a>

The model was able to navigate in the track 1 without running outside of the road.

![2pmwrs](https://user-images.githubusercontent.com/37708330/50383991-443e5780-06be-11e9-81a6-1b396597b676.gif)

Full Video : [Video Link](https://youtu.be/cDPw8Onx3qk)



## Files <a name="files"></a>
* `model.py` - The script used to create and train the model.
* `drive.py` - The script to drive the car.
* `model.h5` - The model weights.



### Dependencies
This lab requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

The following resources can be found in this github repository:
* drive.py
* video.py
* writeup_template.md

The simulator can be downloaded from the classroom. In the classroom, we have also provided sample data that you can optionally use to help train your model.

### Details About Files In This Directory

Usage of `drive.py` requires you have saved the trained model as an h5 file, i.e. `model.h5`. See the [Keras documentation](https://keras.io/getting-started/faq/#how-can-i-save-a-keras-model) for how to create this file using the following command:
```sh
model.save(filepath)
```

Once the model has been saved, it can be used with drive.py using this command:

```sh
python drive.py model.h5
```

The above command will load the trained model and use the model to make predictions on individual images in real-time and send the predicted angle back to the server via a websocket connection.

Note: There is known local system's setting issue with replacing "," with "." when using drive.py. When this happens it can make predicted steering values clipped to max/min values. If this occurs, a known fix for this is to add "export LANG=en_US.utf8" to the bashrc file.
