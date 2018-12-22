import csv
import cv2
import numpy as np
#load the image log from csv
lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_bridge/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images = []
measurements = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_bridge/IMG/'+filename
    image = cv2.imread(current_path)
    measurement = float(line[3])
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements.append(measurement)
    images.append(image)
    measurements.append(measurement_flipped)
    images.append(image_flipped)



#---end bridge------------------------------
lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_lap1/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_lap1 = []
measurements_lap1 = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_lap1/IMG/'+filename
    image = cv2.imread(current_path)
    images_lap1.append(image)
    measurement = float(line[3])
    measurements_lap1.append(measurement)
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_lap1.append(measurement_flipped)
    images_lap1.append(image_flipped)

#---------end lap1------------------------
lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_lap2/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_lap2 = []
measurements_lap2 = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_lap2/IMG/'+filename
    image = cv2.imread(current_path)
    images_lap2.append(image)
    measurement = float(line[3])
    measurements_lap2.append(measurement)    
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_lap2.append(measurement_flipped)
    images_lap2.append(image_flipped)
    

#----------end of lap2-------------

lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_lap1_rev/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_lap_rev = []
measurements_lap_rev = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_lap1_rev/IMG/'+filename
    image = cv2.imread(current_path)
    images_lap_rev.append(image)
    measurement = float(line[3])
    measurements_lap_rev.append(measurement)   
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_lap_rev.append(measurement_flipped)
    images_lap_rev.append(image_flipped)

#----------end of rev-------------

lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_center_lap1/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_cent_lap1 = []
measurements_cent_lap1 = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_center_lap1/IMG/'+filename
    image = cv2.imread(current_path)
    images_cent_lap1.append(image)
    measurement = float(line[3])
    measurements_cent_lap1.append(measurement)   
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_cent_lap1.append(measurement_flipped)
    images_cent_lap1.append(image_flipped)
    
#----------end of cent_lap1-------------

lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_center_lap2/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_cent_lap2 = []
measurements_cent_lap2 = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_center_lap2/IMG/'+filename
    image = cv2.imread(current_path)
    images_cent_lap2.append(image)
    measurement = float(line[3])
    measurements_cent_lap2.append(measurement)   
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_cent_lap2.append(measurement_flipped)
    images_cent_lap2.append(image_flipped)

#----------end of cent_lap2-------------

lines = []
with open ('/home/workspace/CarND-Behavioral-Cloning-P3/training_recovery_lap1/driving_log.csv') as csvfile:
    reader = csv.reader (csvfile)
    for line in reader:
        lines.append(line)

#read images
images_rec_lap1 = []
measurements_rec_lap1 = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = '/home/workspace/CarND-Behavioral-Cloning-P3/training_recovery_lap1/IMG/'+filename
    image = cv2.imread(current_path)
    images_rec_lap1.append(image)
    measurement = float(line[3])
    measurements_rec_lap1.append(measurement)   
    
    image_flipped = np.fliplr(image)
    measurement_flipped = -measurement
    
    measurements_rec_lap1.append(measurement_flipped)
    images_rec_lap1.append(image_flipped)

#----------end of rec_lap1-------------







consolidated_images = images + images_lap1 + images_lap2 + images_lap_rev + images_cent_lap1 + images_cent_lap2 + images_rec_lap1
consolidated_measurements = measurements + measurements_lap1 + measurements_lap2 + measurements_lap_rev + measurements_cent_lap1 + measurements_cent_lap2 + measurements_rec_lap1


X_train = np.array(consolidated_images)
y_train = np.array(consolidated_measurements)
print(X_train.shape)




#-------------------model starts----------------------------------------------
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers import Convolution2D, MaxPooling2D, Cropping2D

model = Sequential()
model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape = (160,320,3)))
model.add(Cropping2D(cropping=((70,20), (0,0))))

model.add(Convolution2D(6,5,5,activation= "relu"))
model.add(MaxPooling2D())

model.add(Convolution2D(10,5,5,activation= "relu"))
model.add(MaxPooling2D())

model.add(Convolution2D(1,5,5,activation= "relu"))
model.add(MaxPooling2D())

model.add(Flatten())
model.add(Dense(120))
model.add(Dense(84))
model.add(Dense(1))

model.compile(loss = 'mse', optimizer = 'adam')
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=15)

model.save('/home/workspace/CarND-Behavioral-Cloning-P3/model.h5')
    
    
exit()
    
    
    
    