
#pragma once


	typedef struct _TRANSFORM_SMOOTH_PARAMETERS
	{
		FLOAT   fSmoothing;             // [0..1], lower values closer to raw data
		FLOAT   fCorrection;            // [0..1], lower values slower to correct towards the raw data
		FLOAT   fPrediction;            // [0..n], the number of frames to predict into the future
		FLOAT   fJitterRadius;          // The radius in meters for jitter reduction
		FLOAT   fMaxDeviationRadius;    // The maximum radius in meters that filtered positions are allowed to deviate from raw data
	} TRANSFORM_SMOOTH_PARAMETERS;

	class FilterDoubleExponentialData
	{
	public:
		CameraSpacePoint m_vRawPosition;
		CameraSpacePoint m_vFilteredPosition;
		CameraSpacePoint m_vTrend;
		int m_dwFrameCount;
	};

class KinectJointFilter
	{
	public:
		/*KinectJointFilter()
		{
			m_pHistory = FilterDoubleExponentialData();

			Init();
		}*/

		KinectJointFilter(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
		{
			m_pHistory = FilterDoubleExponentialData();

			Init(fSmoothing,fCorrection,fPrediction, fJitterRadius,fMaxDeviationRadius);
		}

		~KinectJointFilter()
		{
			Shutdown();
		}

		void KinectJointFilter::UpdateJoint(Joint joint);

		void Init(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
		{
			Reset(fSmoothing, fCorrection, fPrediction, fJitterRadius, fMaxDeviationRadius);
		}

		void Shutdown()
		{
		}

		void Reset(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
		{

			m_fMaxDeviationRadius = fMaxDeviationRadius; // Size of the max prediction radius Can snap back to noisy data when too high
			m_fSmoothing = fSmoothing;                   // How much smothing will occur.  Will lag when too high
			m_fCorrection = fCorrection;                 // How much to correct back from prediction.  Can make things springy
			m_fPrediction = fPrediction;                 // Amount of prediction into the future to use. Can over shoot when too high
			m_fJitterRadius = fJitterRadius;             // Size of the radius where jitter is removed. Can do too much smoothing when too high

				m_pFilteredJoints.X = 0.0f;
				m_pFilteredJoints.Y = 0.0f;
				m_pFilteredJoints.Z = 0.0f;

				m_pHistory.m_vFilteredPosition.X = 0.0f;
				m_pHistory.m_vFilteredPosition.Y = 0.0f;
				m_pHistory.m_vFilteredPosition.Z = 0.0f;

				m_pHistory.m_vRawPosition.X = 0.0f;
				m_pHistory.m_vRawPosition.Y = 0.0f;
				m_pHistory.m_vRawPosition.Z = 0.0f;

				m_pHistory.m_vTrend.X = 0.0f;
				m_pHistory.m_vTrend.Y = 0.0f;
				m_pHistory.m_vTrend.Z = 0.0f;

				m_pHistory.m_dwFrameCount = 0;
		}

		CameraSpacePoint* GetFilteredJoint(){	return &m_pFilteredJoints;}

	private:
		CameraSpacePoint m_pFilteredJoints;
		FilterDoubleExponentialData m_pHistory;
		float m_fSmoothing;
		float m_fCorrection;
		float m_fPrediction;
		float m_fJitterRadius;
		float m_fMaxDeviationRadius;
		bool KinectJointFilter::JointPositionIsValid(CameraSpacePoint vJointPosition);
		CameraSpacePoint KinectJointFilter::CSVectorSet(float x, float y, float z);
		CameraSpacePoint KinectJointFilter::CSVectorZero();
		CameraSpacePoint KinectJointFilter::CSVectorAdd(CameraSpacePoint p1, CameraSpacePoint p2);
		CameraSpacePoint KinectJointFilter::CSVectorScale(CameraSpacePoint p, float scale);
		CameraSpacePoint KinectJointFilter::CSVectorSubtract(CameraSpacePoint p1, CameraSpacePoint p2);
		float KinectJointFilter::CSVectorLength(CameraSpacePoint p);

	};