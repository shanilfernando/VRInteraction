

#pragma once
#include "stdafx.h"
#include "resource.h"
#include <iostream>
#include <ctime>
#include <fstream>
#include <unordered_map>
#include "User.h"

#ifndef _InteractiveState_
#define _InteractiveState_
typedef enum _InteractiveState InteractiveState;


enum _InteractiveState
{
	InteractiveState_Unknown = 0,
	InteractiveState_Rotate = 1,
	InteractiveState_Zoom = 2,
	InteractiveState_Tracked = 3,
	InteractiveState_Swipe = 4
};
#endif // _InteractiveState_

class CDepthBasics
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

public:
    
    CDepthBasics();

   
    ~CDepthBasics();

	
	HRESULT Update();

	UINT16* getDepthBuffer();
	BYTE* getColorBuffer();
	BYTE* getBodyIndexBuffer(){ return pBodyIndexBuffer; };
	std::unordered_map<UINT64, User> engagedUsers;
	void release();
	
	void Register(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	

private:
	UINT16 *pDepthBuffer = NULL;
	BYTE *pColorBuffer = NULL;
	BYTE *pBodyIndexBuffer = NULL;
	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	IBodyFrame* pBodyFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	BYTE*                m_pColorRGBX;
	UINT64 engagedBodyId;
	
	/*std::ofstream myfile;*/
	TIMESPAN bodyRelativeTime;
	
    IKinectSensor*          m_pKinectSensor;

    IDepthFrameReader*      m_pDepthFrameReader;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;
	ICoordinateMapper*      m_pCoordinateMapper=NULL;
	DepthSpacePoint*        m_pDepthCoordinates;
	CameraSpacePoint*        m_pCoordinates;


    HRESULT                 InitializeDefaultSensor();

    void                    ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

	void					ProcessBody(int nBodyCount, IBody** ppBodies);

};

