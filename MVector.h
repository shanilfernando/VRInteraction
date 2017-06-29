#pragma once
#define PI 3.14159265
static class MVector
{
public:
	MVector();
	MVector(float x,float y);
	MVector::MVector(float x1, float y1, float x2, float y2);
	~MVector();
	float X;
	float Y;

	 float static MVector::GetAngleBetweenTwoVectors(MVector v1, MVector v2);

};

