
# coding: utf-8

# In[74]:

'''
After measuring , we write the function that takes the given streering angle of the car and 
return the steering angle in the reality. 
The convertion of measurements in angles  didn't make  so sense much that why we decided to continue with our 
an intuitiv apprixomation observed in the labor  : (the more car turn the  less precise it is!  )
'''


import pandas as pd 
import numpy as np 

def function_approx(angle):
    
    x=[0, 30, 60, 70,  90, 160,180]
    
    # our  intuitiv approximation ( assumed from our intuition )
    y=[0, 32, 65, 78, 100 , 165,200]
    
    # polynomimal approximation  (the set polynome to maximal degree) 
    z = np.polyfit(x, y,6 )
    
    # evaluation of the 
    angle_correction = np.polyval( z, angle)
    
    return angle_correction



# make table 
''' 
here we just make the table.  
'''

given_angle= [0, 30,60, 90,120,150, 180]

real_angle= list( map(function_approx, given_angle))
real_angle=list( map(int,real_angle))
real_angle= np.array(real_angle)

real_angle= pd.DataFrame(real_angle).T

xy=[[0, 1.0, 2.0, 3.0,  4.0,  5.0,6.0],[0, 1.0, 2.0, 3.0,  4.0,  5.0,6.0]]
xy= pd.DataFrame(xy ,index=['given angle ','real'])
xy.drop('given angle ', inplace=True)

table =pd.concat([xy, real_angle], axis=0)
table.rename(columns={1: '30', 2: '60',3: '90',4: '120',5: '150',6: '180'},inplace=True)
table.drop('real', inplace=True)
table.rename(index={0:' real values'},inplace=True)



'main'
function_test= function_approx(35)
print('turn in real world: ',function_test )

print (table)




# In[ ]:



