#Unzip the training data to data_udacity folder###
import os
from zipfile import ZipFile
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def unzip(dir,name):
         with ZipFile(dir) as zipf:
             zipf.extractall('data_udacity')
 unzip('data.zip','data_udacity')
 print("Unzip done...")
