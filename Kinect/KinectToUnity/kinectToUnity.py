import os
######## Change it according to python script and data location ########
import csv

import pickle
import json
import ast

#import numpy as np

#from base64 import b16decode

import numpy as np
#from scipy import linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import inf

import sys
sys.path.insert(1, './../')


from timeit import repeat
#import numpy as np
import datetime

from matplotlib import animation
import time



if __name__=="__main__":
    data2=np.load(r'Kinect/RecordedVideo/00001.npy',allow_pickle=True)

    total_frames=int(np.max(data2[:,0]))
    print(total_frames)

    print(int(np.max(data2[:,14])))

    all_person_data=np.zeros((int(np.max(data2[:,14])),total_frames,11))
    zero_person_data=np.zeros((800,11)).astype(int)
    for i in range(zero_person_data.shape[0]):
        zero_person_data[i,0]=(i+1001)
    
    print(all_person_data.shape)

    now = datetime.datetime.now()
    dateFolder = now.strftime("%Y%m%d_%H%M")

    if not os.path.exists('Kinect/RecordedVideo/UnityCSVs/'+dateFolder):
        os.makedirs('Kinect/RecordedVideo/UnityCSVs/'+dateFolder)


    for i in range(all_person_data.shape[0]):
        current_person_data=data2[np.where(data2[:,14]==i+1),:]
        current_person_data=current_person_data[0,:,:]
        print(current_person_data.shape)
        #print(current_person_data[0:10,0])
        #print(current_person_data[:,0].shape)
        #print(current_person_data[-1,0])

        all_person_data[i,:,0]=list(range(total_frames+1))[1:]

        # Convert to Unity coordinates and save

        all_person_data[i,current_person_data[:,0].astype(int)-1,0]=current_person_data[:, 0].astype(int)
        all_person_data[i,current_person_data[:,0].astype(int)-1,1]=np.round(current_person_data[:, 1]/1000, 2)-5.1 * (current_person_data[:, 1] > 0)-(current_person_data[:, 1] == 0)*10  # unity x
        all_person_data[i,current_person_data[:,0].astype(int)-1,2]=((current_person_data[:, 3] > 1300).astype(float)*0.5+1)*(current_person_data[:, 3] > 0).astype(int)  # unity y, which is python z
        all_person_data[i,current_person_data[:,0].astype(int)-1,3]=np.round(current_person_data[:, 2]/1000, 2)-10.8*(current_person_data[:, 2] > 0)  # unity z, which is python y
        all_person_data[i,current_person_data[:,0].astype(int)-1,4]=current_person_data[:,11]
        all_person_data[i,current_person_data[:,0].astype(int)-1,5]=current_person_data[:,13]
        all_person_data[i,current_person_data[:,0].astype(int)-1,6]=current_person_data[:,12]
        all_person_data[i,current_person_data[:,0].astype(int)-1,7]=current_person_data[:,15].astype(int) #Activity
        all_person_data[i,current_person_data[:,0].astype(int)-1,8]=(current_person_data[:,3]>1300).astype(int)+1
        all_person_data[i,current_person_data[:,0].astype(int)-1,9]=current_person_data[:,16].astype(int) #sTAtionary
        all_person_data[i,current_person_data[:,0].astype(int)-1,10]=0
        
        txt = r'Kinect/RecordedVideo/UnityCSVs/'+dateFolder+'/person_'+str(i+1)+'_unity.csv'
        with open(txt, 'w', newline='') as file:
            mywriter = csv.writer(file, delimiter=',')
            mywriter.writerow(['frame','head_x','head_y','head_z','pos_x','pos_y','pos_z','activity','pose','bubble','glare'])
            for j in range(all_person_data.shape[1]):
                mywriter.writerow(np.round(all_person_data[i,j,:],2))
