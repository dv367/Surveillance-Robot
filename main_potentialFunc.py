
from basics import *
from math import *
import numpy as np


def contourAttractive(x,y,xg,yg):	
	
	zeta = 1.0#0.22/dist(0,0,xg,yg)

	xx = np.linspace(x, 2*xg, 100)
	yy = np.linspace(y, 2*yg, 100)

	X, Y = np.meshgrid(xx, yy)
	Z = zeta*distNP(X, Y, xg,yg)**2 + (Y - X)**2 #only Magnitude
	
	
	
	
	return X,Y,Z
	

def contourRepulsive(x,y,xg,yg,xo,yo):
	
	tol = 1.0
	r = 0.1
	mR = 3.5
	eta = 0.22/(((1/tol) - (1/mR))*(mR)**2)
	

	xx = np.linspace(x, xg*2, 100)
	yy = np.linspace(y, yg*2, 100)
	
			

	X, Y = np.meshgrid(xx, yy)
	Z = distNP(X/100, Y/100, xo/100,yo/100)
	
	
	for i in range(0,len(Z)):
		for j in range(0,len(Z)):
			if Z[i][j] < tol and Z[i][j] > 0:
				Z[i][j] = fabs(eta*((1/tol) - (1/Z[i][j]))*(1/Z[i][j]**2))
				
				if fabs(Z[i][j]) > 0.22:
					Z[i][j] = 0.22
				 			
			else:
				Z[i][j] = 0
			

		
	return X,Y,Z	

def actuate():

	X1,Y1,Z1 = contourRepulsive(0,0,500,500,100,130)
	X2,Y2,Z2 = contourRepulsive(0,0,500,500,330,270)
	X3,Y3,Z3 = contourAttractive(0,0,500,500)
	X,Y,Z = addContour(X1,Y1,Z1,Z2,Z3)
	plotcontour3D(X3,Y3,Z3)
	#plotcontour2D(X,Y,Z)


actuate()






           	

	





