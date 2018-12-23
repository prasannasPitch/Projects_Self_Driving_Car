import os
import numpy as np
from sklearn.preprocessing import LabelBinarizer
import csv
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import cv2
import sklearn
import matplotlib.pyplot as plt


#write all the data in training_data variable through the csv file
training_data = [] 
with open('./data_udacity/data/driving_log.csv') as csvfile: 
    reader = csv.reader(csvfile)
    #skip heading
    next(reader, None) 
    for line in reader:
        training_data.append(line)


#split the training data for training and validation
train_data, validation_data = train_test_split(training_data,test_size=0.2)


#preprcess the images to carry out corrections in image - generator function which uses yield to return
def preprocess_generator(images, batch_size=32):
    shuffle(images)
    count_images = len(images)
    while 1:  # Loop forever so the generator never terminates
        for seen_images in range(0, count_images, batch_size):
            batch_images = images[seen_images:seen_images+batch_size]
            corrected_image = []
            steering_op = []
            for each_image in batch_images:
                    for i in range(0,3): 
                        image_name = './data_udacity/data/IMG/'+each_image[i].split('/')[-1]
                        center_image = cv2.cvtColor(cv2.imread(image_name), cv2.COLOR_BGR2RGB) 
                        center_angle = float(each_image[3]) 
                        corrected_image.append(center_image)
                        
        	            #using multiple camera image - we add a threshold in left and right image for centering the image 
                        if(i==0):
                            steering_op.append(center_angle)
                        elif(i==1):
                            steering_op.append(center_angle+0.2)
                        elif(i==2):
                            steering_op.append(center_angle-0.2)
                        
                        # data augumentation done by flipping the image and negating the steering output
                        corrected_image.append(cv2.flip(center_image,1))
                        if(i==0):
                            steering_op.append(center_angle*-1)
                        elif(i==1):
                            steering_op.append((center_angle+0.2)*-1)
                        elif(i==2):
                            steering_op.append((center_angle-0.2)*-1)
                        
        
            X_preprocessed = np.array(corrected_image)
            y_preprocessed = np.array(steering_op)
            
            #value returned only when the genrator object is looped, thus saving memory 
            yield sklearn.utils.shuffle(X_preprocessed, y_preprocessed)
           

# compile and train the model using the generator function
X_y_train = preprocess_generator(train_data, batch_size=32)
X_y_valid = preprocess_generator(validation_data, batch_size=32)

#----------------------------------------------------MODEL INSPIRED FROM NVIDIA-------------------------------------------------------

from keras.models import Sequential
from keras.layers.core import Dense, Flatten, Activation, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers import Lambda, Cropping2D

model = Sequential()

# Preprocess incoming data, centered around zero with small standard deviation 
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))

# trim image to only see section with road
model.add(Cropping2D(cropping=((70,25),(0,0))))           

#layer 1- Convolution, no of filters- 24, filter size= 5x5, stride= 2x2
model.add(Convolution2D(24,5,5,subsample=(2,2)))
model.add(Activation('elu'))

#layer 2- Convolution, no of filters- 36, filter size= 5x5, stride= 2x2
model.add(Convolution2D(36,5,5,subsample=(2,2)))
model.add(Activation('elu'))

#layer 3- Convolution, no of filters- 48, filter size= 5x5, stride= 2x2
model.add(Convolution2D(48,5,5,subsample=(2,2)))
model.add(Activation('elu'))

#layer 4- Convolution, no of filters- 64, filter size= 3x3, stride= 1x1
model.add(Convolution2D(64,3,3))
model.add(Activation('elu'))

#layer 5- Convolution, no of filters- 64, filter size= 3x3, stride= 1x1
model.add(Convolution2D(64,3,3))
model.add(Activation('elu'))

#flatten image from 2D to side by side
model.add(Flatten())

#layer 6- fully connected layer 1
model.add(Dense(100))
model.add(Activation('elu'))

#Dropout for avoiding overfit
model.add(Dropout(0.25))

#layer 7- fully connected layer 1
model.add(Dense(50))
model.add(Activation('elu'))


#layer 8- fully connected layer 1
model.add(Dense(10))
model.add(Activation('elu'))

#layer 9- fully connected layer 1
model.add(Dense(1)) 

#we find the steering angle output for a given image - so clearly regression - thus using MSE
model.compile(loss='mse',optimizer='adam')

#fit generator function supported by keras to use data from generator object
model.fit_generator(X_y_train, samples_per_epoch= len(train_data), validation_data=X_y_valid,   nb_val_samples=len(validation_data), nb_epoch=5, verbose=1)

#saving model
model.save('model_updated.h5')


