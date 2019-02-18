import math
import cmath

def cod2ang(x,y,l1,l2):

## x is the position along the x-axis
## y is the position along the y-axis
## l1 is the length of the 1st link
## l2 is the length of the 2nd link

	x2 = x*x;
	y2 = y*y;
	l12 = l1*l1;
	l22 = l2*l2;

	r = math.sqrt(x2 + y2);		

# r is the magnitude of the position vector of the final position

##----------------------------------------------------------------------------------------------
##
##						    -1  (y) 	  -1 ( 	    link2 length * sin(angle2) 	      )
## 	  formula of the first angle is, angle1 = tan  (---) - tan  (------------------------------------------)
##							(x)	     (link1 length + link2 length * cos(angle2)
##
##----------------------------------------------------------------------------------------------
##
##						      -1  ( x^2 + y^2 - (link1 length)^2 - (link2 length)^2 )
##	  formula of the second angle is, angle2 = cos   (---------------------------------------------------)
##	 						  (	  2 * link1 length * link2 length	    )
##
##-----------------------------------------------------------------------------------------------

	a2 = cmath.acos((x2 + y2 - l12 - l22)/(2*l1*l2));
	a2d = a2*180 / math.pi;				## convert radian to degree
	a2d = a2d.real;					## to eliminate the imaginary values

	a1 = cmath.atan(y/x) - cmath.atan(l2*cmath.sin(a2)/(l1+l2*cmath.cos(a2)));
	a1d = a1*180 / math.pi;				## convert radian to degree
	a1d = a1d.real;					## to eliminate the imaginary values

	return a1d, a2d
