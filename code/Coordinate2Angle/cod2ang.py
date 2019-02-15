import math
import cmath

def cod2ang(x,y,l1,l2):

	x2 = x*x;
	y2 = y*y;
	l12 = l1*l1;
	l22 = l2*l2;

	r = math.sqrt(x2 + y2);

	a2 = cmath.acos(x2 + y2 - l12 - l22)/(2*l1*l2);
	a2d = a2*180 / math.pi;
	a2d = a2d.real;

	a1 = cmath.atan(y/x) - cmath.atan(l2*cmath.sin(a2)/(l1+l2*cmath.cos(a2)));
	a1d = a1*180 / math.pi;
	a1d = a1d.real;

	return a1d, a2d
