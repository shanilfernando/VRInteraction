
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"


CDepthBasics::CDepthBasics() :
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
	m_pColorRGBX(NULL)
{
	m_pColorRGBX = new BYTE[cColorWidth * cColorHeight*4];
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
	InitializeDefaultSensor();
	/*myfile.open("param.xml");
	myfile << "<Data>\n";*/
}
  

CDepthBasics::~CDepthBasics()
{
	/*myfile << "</Data>\n";
	myfile.close();*/
    SafeRelease(m_pDepthFrameReader);

    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
	SafeRelease(m_pCoordinateMapper);
}


HRESULT CDepthBasics::Update()
{
	if (!m_pMultiSourceFrameReader)
    {
		return E_FAIL;
    }


	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}

		SafeRelease(pBodyFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}

		SafeRelease(pBodyIndexFrameReference);
	}

    if (SUCCEEDED(hr))
    {
		IFrameDescription* pDepthFrameDescription = NULL;
		UINT nDepthBufferSize = 0;
		

		IFrameDescription* pColorFrameDescription = NULL;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		
		UINT nBodyIndexBufferSize = 0;
		
        

			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);

		}


		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			//hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, &pColorBuffer);
			if (imageFormat == ColorImageFormat_Rgba)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(unsigned char)*4;
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, pColorBuffer, ColorImageFormat_Rgba);
			}
		}

		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr))
		{
			ProcessBody(BODY_COUNT, ppBodies);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}
        

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
    }

	return hr;
}

HRESULT CDepthBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->Open();
		}

        if (SUCCEEDED(hr))
        {
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
        }


    }

    if (!m_pKinectSensor || FAILED(hr))
    {
		std::cout << "No ready Kinect found!" << std::endl;
        return E_FAIL;
	}
	else
	{
		std::cout << "Kinect found!" << std::endl;
	}

    return hr;
}

void CDepthBasics::ProcessBody(int nBodyCount, IBody** ppBodies)
{
	HRESULT hr;
	
	UINT64 CurrentBodyId = NULL;
	bool IsBodyTracked = false;

	for (int i = 0; i < nBodyCount; ++i)
	{
		
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];

				hr = pBody->GetJoints(_countof(joints), joints);
				pBody->get_TrackingId(&CurrentBodyId);
				User* user;
				int count = engagedUsers.count(CurrentBodyId);
				if (count==0 && joints[JointType_HandRight].Position.Y > joints[JointType_Head].Position.Y){
						engagedUsers[CurrentBodyId];
						std::cout <<  CurrentBodyId<<":Enaged" << std::endl;
					}

				count=engagedUsers.count(CurrentBodyId);

				if (count==1)
				{
					HandState leftHandState = HandState_Unknown;
					HandState rightHandState = HandState_Unknown;
					CameraSpacePoint* HandRight;
					CameraSpacePoint* HandLeft;
					CameraSpacePoint* HandRightTip;
					user = &engagedUsers[CurrentBodyId];
					user->isTracked = true;
					pBody->get_HandLeftState(&leftHandState);
					pBody->get_HandRightState(&rightHandState);
					HandLeft = user->getFilteredLeftHand(joints[JointType_HandLeft]);
					HandRight = user->getFilteredRightHand(joints[JointType_HandRight]);
					HandRightTip = (user->getFilteredRightHandTip(joints[JointType_HandTipRight]));
					user->spineMid=joints[JointType_SpineMid].Position;
					user->interactiveState = InteractiveState_Unknown;

					if (user->preRightHandState == HandState_Closed && rightHandState == HandState_Open){
						user->interactionStart = !user->interactionStart;
						std::cout << user->interactionStart << std::endl;
					}


					if (HandLeft->Y > joints[JointType_HipLeft].Position.Y && HandRight->Y > joints[JointType_HipRight].Position.Y && HandLeft->Y < joints[JointType_Head].Position.Y && HandRight->Y < joints[JointType_Head].Position.Y){
						if (leftHandState == HandState_Closed && rightHandState == HandState_Closed)
						{
							user->interactiveState = InteractiveState_Zoom;
						}
						else
						{
							if (user->interactionStart){
								user->interactiveState = InteractiveState_Rotate;
							}
						}
					}else if (HandLeft->Y < joints[JointType_HipLeft].Position.Y && HandRight->Y > joints[JointType_HipRight].Position.Y)
					{
						if (user->isRightHandHorizontal())
						{
							if (user->rightHandTips.size() >= 10)
							{
								user->rightHandTips.pop_front();
								user->interactiveState = InteractiveState_Swipe;
							}
							user->rightHandTips.push_back(*HandRightTip);
						}
						else
						{
							user->interactiveState = InteractiveState_Click;
						}
						

					}
					user->pointerJoint.Position.X = HandRight->X - joints[JointType_SpineMid].Position.X - 0.15;
					user->pointerJoint.Position.Y = joints[JointType_SpineMid].Position.Y + 0.25 - HandRight->Y;
					user->pointerJoint.Position.Z = HandRight->Z - joints[JointType_SpineMid].Position.Z;
					user->getFilteredPointer();
					user->preRightHandState = rightHandState;

					if (HandLeft->Y < joints[JointType_HipLeft].Position.Y && HandRight->Y < joints[JointType_HipRight].Position.Y)
					{
						engagedUsers.erase(CurrentBodyId);
						std::cout << CurrentBodyId<< ":Disenaged" << std::endl;
					}

					
				}
			}


		}
	}
	if (!engagedUsers.empty())
	{
		std::unordered_map<UINT64, User>::iterator it = engagedUsers.begin();
		for (int i = 0; i < engagedUsers.size(); i++)
		{
			if (it->second.isTracked == false){
				std::cout << it->first << ":Disenaged" << std::endl;
				it = engagedUsers.erase(it);
			}
			else
			{
				it->second.isTracked = false;
				if (it != engagedUsers.end())
				{
					it++;
				}
				
			}

		}
	}
	
}

UINT16* CDepthBasics::getDepthBuffer()
{
	return pDepthBuffer;

}

BYTE* CDepthBasics::getColorBuffer()
{
	return pColorBuffer;
}

void CDepthBasics::release(){
	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
	
}

void CDepthBasics::Register(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	std::vector<pcl::PointXYZRGB> vec;
	int size = 0;
	for (int y = 0; y < cDepthHeight; y++){
		for (int x = 0; x < cDepthWidth; x++){
			pcl::PointXYZRGB point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = pDepthBuffer[y * cDepthWidth + x];
			if (depth > 0){
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				m_pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < cColorWidth) && (0 <= colorY) && (colorY < cColorHeight)){
					unsigned char* color = &pColorBuffer[4*(colorY * cColorWidth + colorX)];
					point.r = *color;
					point.g = *(color + 1);
					point.b = *(color + 2);
				}

				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				m_pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < cColorWidth) && (0 <= colorY) && (colorY < cColorHeight)){
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}
				size++;
				vec.push_back(point);
			}

		}
	}
		cloud->is_dense = true;
		cloud->resize(size);
		memcpy(&cloud->points[0], &vec[0], sizeof(pcl::PointXYZRGB)*size);
}