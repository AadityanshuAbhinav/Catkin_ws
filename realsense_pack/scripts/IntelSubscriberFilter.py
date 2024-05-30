#!/usr/bin/env python3
import rospy
import numpy as np
   
def Filter(Xval,Yval,Zval,ite):
   if ite>6:
      a = np.array([0.5,0.1,0.1,0.1,0.1,0.1])
   else:
      a = np.array([1,0,0,0,0,0])
   x = np.dot(a,Xval)
   y = np.dot(a,Yval)
   Z = np.dot(a,Zval)
   out = np.array([x,y,Z])
   return out
      
if __name__ == '__main__':
   Xval = np.array([1.5,1,1,1,1,1])
   Yval = np.array([1,1,1,1,1,1])
   Zval = np.array([1,1,1,1,1,1])
   Filter(Xval,Yval,Zval)
