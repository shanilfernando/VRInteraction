

#include <string>
#include <libfreenect2/config.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/boost.h>
#include <pcl\registration\icp.h>
#include <pcl\registration\incremental_icp.h>
#include <pcl\visualization\pcl_visualizer.h>
#include "stdafx.h"
#include "resource.h"
#include "registrationCuda.h"
#include "Dashboard.h"

class Registration
	{
	public:
		Registration::Registration(const std::string param_file);

		void apply(int dx, int dy, float dz, float& cx, float &cy) const;

		void cudaInit(Eigen::Matrix4d depth2world_);
		void freeMem(){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			cudaFree(d_points);
			freeCudaMem();
			cudaDeviceReset();
		}

		
		void Registration::apply5(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int nthreads = 304);
		void Registration::apply6(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard);
		void Registration::apply7(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard, const BYTE *pBodyIndexBuffer);
		void Registration::apply8(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard, const BYTE *pBodyIndexBuffer, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void Registration::apply9(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const BYTE *pBodyIndexBuffer, int* extrapix,int nthreads = 304);

	private:
		void distort(int mx, int my, float& dx, float& dy) const;
		void depth_to_color(float mx, float my, float& rx, float& ry) const;

		libfreenect2::Freenect2Device::IrCameraParams depth;
		libfreenect2::Freenect2Device::ColorCameraParams color;

		int distort_map[512 * 424];
		float depth_to_color_map_x[512 * 424];
		float depth_to_color_map_y[512 * 424];
		int depth_to_color_map_yi[512 * 424];
		std::vector<std::vector<pcl::PointXYZRGB>> partial_clouds_;
		std::vector<pcl::PointXYZRGB> partial_clouds_2;
		const int size_depth = 512 * 424;
		const int size_color = 1920 * 1080;
		int size_filter_map;
		int offset_filter_map;
		float color_cx;

		const int filter_width_half;
		const int filter_height_half;
		const float filter_tolerance;
		
		const int size_x = 512;
		const int size_y = 424;
		Eigen::Matrix4d depth2world;

		int *d_map_dist;
		int *d_map_yi;
		float *d_map_x;
		float *d_color_shift_m;
		float *d_color_fx;
		float *d_color_cx;
		double *d_t;

		unsigned short *d_depth_data;
		unsigned char *d_rgb_data;
		unsigned int *d_filter_map;
		unsigned int *d_count;
		unsigned int *d_p_filter_map = NULL;
		unsigned char *d_body_index_data;
		PointXYZRGBNew *d_points;
	};

