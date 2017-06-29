#include "MVector.h"
#include <cmath>


MVector::MVector():
X(0.0f),
Y(0.0f)
{
}

MVector::MVector(float x, float y):
X(x),
Y(y)
{
}

MVector::MVector(float x1, float y1, float x2, float y2):
X(x2-x1),
Y(y2-y1)
{
}

MVector::~MVector()
{
}

float MVector::GetAngleBetweenTwoVectors(MVector v1, MVector v2)
{
	float dot = v1.X*v2.X + v1.Y*v2.Y;
	float det = v1.X*v2.Y - v1.Y*v2.X;
	return atan2(det, dot);
}

