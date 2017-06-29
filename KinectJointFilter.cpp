#include "stdafx.h"



bool KinectJointFilter::JointPositionIsValid(CameraSpacePoint vJointPosition)
{
	return (vJointPosition.X != 0.0f ||
		vJointPosition.Y != 0.0f ||
		vJointPosition.Z != 0.0f);
}



CameraSpacePoint KinectJointFilter::CSVectorSet(float x, float y, float z)
{
	CameraSpacePoint point = CameraSpacePoint();

	point.X = x;
	point.Y = y;
	point.Z = z;

	return point;
}

CameraSpacePoint KinectJointFilter::CSVectorZero()
{
	CameraSpacePoint point = CameraSpacePoint();

	point.X = 0.0f;
	point.Y = 0.0f;
	point.Z = 0.0f;

	return point;
}

CameraSpacePoint KinectJointFilter::CSVectorAdd(CameraSpacePoint p1, CameraSpacePoint p2)
{
	CameraSpacePoint sum = CameraSpacePoint();

	sum.X = p1.X + p2.X;
	sum.Y = p1.Y + p2.Y;
	sum.Z = p1.Z + p2.Z;

	return sum;
}

CameraSpacePoint KinectJointFilter::CSVectorScale(CameraSpacePoint p, float scale)
{
	CameraSpacePoint point = CameraSpacePoint();

	point.X = p.X * scale;
	point.Y = p.Y * scale;
	point.Z = p.Z * scale;

	return point;
}

CameraSpacePoint KinectJointFilter::CSVectorSubtract(CameraSpacePoint p1, CameraSpacePoint p2)
{
	CameraSpacePoint diff = CameraSpacePoint();

	diff.X = p1.X - p2.X;
	diff.Y = p1.Y - p2.Y;
	diff.Z = p1.Z - p2.Z;

	return diff;
}

float KinectJointFilter::CSVectorLength(CameraSpacePoint p)
{
	return fabs(sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z));
}

void KinectJointFilter::UpdateJoint(Joint joint)
{
	TRANSFORM_SMOOTH_PARAMETERS smoothingParams;

	smoothingParams.fSmoothing = m_fSmoothing;
	smoothingParams.fCorrection = m_fCorrection;
	smoothingParams.fPrediction = m_fPrediction;
	smoothingParams.fJitterRadius = m_fJitterRadius;
	smoothingParams.fMaxDeviationRadius = m_fMaxDeviationRadius;

		// If inferred, we smooth a bit more by using a bigger jitter radius
		if (joint.TrackingState == TrackingState::TrackingState_Inferred)
		{
			smoothingParams.fJitterRadius *= 2.0f;
			smoothingParams.fMaxDeviationRadius *= 2.0f;
		}

	CameraSpacePoint vPrevRawPosition;
	CameraSpacePoint vPrevFilteredPosition;
	CameraSpacePoint vPrevTrend;
	CameraSpacePoint vRawPosition;
	CameraSpacePoint vFilteredPosition;
	CameraSpacePoint vPredictedPosition;
	CameraSpacePoint vDiff;
	CameraSpacePoint vTrend;
	float fDiff;
	bool bJointIsValid;

	vRawPosition = joint.Position;
	vPrevFilteredPosition = m_pHistory.m_vFilteredPosition;
	vPrevTrend = m_pHistory.m_vTrend;
	vPrevRawPosition = m_pHistory.m_vRawPosition;
	bJointIsValid = JointPositionIsValid(vRawPosition);

	// If joint is invalid, reset the filter
	if (!bJointIsValid)
	{
		m_pHistory.m_dwFrameCount = 0;
	}

	// Initial start values
	if (m_pHistory.m_dwFrameCount == 0)
	{
		vFilteredPosition = vRawPosition;
		vTrend = CSVectorZero();
		m_pHistory.m_dwFrameCount++;
	}
	else if (m_pHistory.m_dwFrameCount == 1)
	{
		vFilteredPosition = CSVectorScale(CSVectorAdd(vRawPosition, vPrevRawPosition), 0.5f);
		vDiff = CSVectorSubtract(vFilteredPosition, vPrevFilteredPosition);
		vTrend = CSVectorAdd(CSVectorScale(vDiff, smoothingParams.fCorrection), CSVectorScale(vPrevTrend, 1.0f - smoothingParams.fCorrection));
		m_pHistory.m_dwFrameCount++;
	}
	else
	{
		// First apply jitter filter
		vDiff = CSVectorSubtract(vRawPosition, vPrevFilteredPosition);
		fDiff = CSVectorLength(vDiff);

		if (fDiff <= smoothingParams.fJitterRadius)
		{
			vFilteredPosition = CSVectorAdd(CSVectorScale(vRawPosition, fDiff / smoothingParams.fJitterRadius),
				CSVectorScale(vPrevFilteredPosition, 1.0f - fDiff / smoothingParams.fJitterRadius));
		}
		else
		{
			vFilteredPosition = vRawPosition;
		}

		// Now the double exponential smoothing filter
		vFilteredPosition = CSVectorAdd(CSVectorScale(vFilteredPosition, 1.0f - smoothingParams.fSmoothing),
			CSVectorScale(CSVectorAdd(vPrevFilteredPosition, vPrevTrend), smoothingParams.fSmoothing));


		vDiff = CSVectorSubtract(vFilteredPosition, vPrevFilteredPosition);
		vTrend = CSVectorAdd(CSVectorScale(vDiff, smoothingParams.fCorrection), CSVectorScale(vPrevTrend, 1.0f - smoothingParams.fCorrection));
	}

	// Predict into the future to reduce latency
	vPredictedPosition = CSVectorAdd(vFilteredPosition, CSVectorScale(vTrend, smoothingParams.fPrediction));

	// Check that we are not too far away from raw data
	vDiff = CSVectorSubtract(vPredictedPosition, vRawPosition);
	fDiff = CSVectorLength(vDiff);

	if (fDiff > smoothingParams.fMaxDeviationRadius)
	{
		vPredictedPosition = CSVectorAdd(CSVectorScale(vPredictedPosition, smoothingParams.fMaxDeviationRadius / fDiff),
			CSVectorScale(vRawPosition, 1.0f - smoothingParams.fMaxDeviationRadius / fDiff));
	}

	// Save the data from this frame
	m_pHistory.m_vRawPosition = vRawPosition;
	m_pHistory.m_vFilteredPosition = vFilteredPosition;
	m_pHistory.m_vTrend = vTrend;

	// Output the data
	m_pFilteredJoints = vPredictedPosition;
}