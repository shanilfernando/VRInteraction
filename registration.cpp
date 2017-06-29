#include <math.h>
#include "registration.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

	static const float depth_q = 0.01;
	static const float color_q = 0.002199;

	void Registration::distort(int mx, int my, float& x, float& y) const
	{
		float dx = ((float)mx - depth.cx) / depth.fx;
		float dy = ((float)my - depth.cy) / depth.fy;
		float dx2 = dx * dx;
		float dy2 = dy * dy;
		float r2 = dx2 + dy2;
		float dxdy2 = 2 * dx * dy;
		float kr = 1 + ((depth.k3 * r2 + depth.k2) * r2 + depth.k1) * r2;
		x = depth.fx * (dx * kr + depth.p2 * (r2 + 2 * dx2) + depth.p1 * dxdy2) + depth.cx;
		y = depth.fy * (dy * kr + depth.p1 * (r2 + 2 * dy2) + depth.p2 * dxdy2) + depth.cy;
	}

	void Registration::depth_to_color(float mx, float my, float& rx, float& ry) const
	{
		
		mx = (mx - depth.cx) * depth_q;
		my = (my - depth.cy) * depth_q;

		float wx =
			(mx * mx * mx * color.mx_x3y0) + (my * my * my * color.mx_x0y3) +
			(mx * mx * my * color.mx_x2y1) + (my * my * mx * color.mx_x1y2) +
			(mx * mx * color.mx_x2y0) + (my * my * color.mx_x0y2) + (mx * my * color.mx_x1y1) +
			(mx * color.mx_x1y0) + (my * color.mx_x0y1) + (color.mx_x0y0);

		float wy =
			(mx * mx * mx * color.my_x3y0) + (my * my * my * color.my_x0y3) +
			(mx * mx * my * color.my_x2y1) + (my * my * mx * color.my_x1y2) +
			(mx * mx * color.my_x2y0) + (my * my * color.my_x0y2) + (mx * my * color.my_x1y1) +
			(mx * color.my_x1y0) + (my * color.my_x0y1) + (color.my_x0y0);

		rx = (wx / (color.fx * color_q)) - (color.shift_m / color.shift_d);
		ry = (wy / color_q) + color.cy;
	}

	void Registration::apply(int dx, int dy, float dz, float& cx, float &cy) const
	{
		const int index = dx + dy * 512;
		float rx = depth_to_color_map_x[index];
		cy = depth_to_color_map_y[index];

		rx += (color.shift_m / dz);
		cx = rx * color.fx + color.cx;
	}

	

	void Registration::cudaInit(Eigen::Matrix4d depth2world_){
		depth2world = depth2world_;
		double *h_t = (double *)depth2world_.data();
		cudaDeviceReset();
		bool result = init(distort_map, &(depth_to_color_map_x[0]), &(depth_to_color_map_yi[0]), &(color.shift_m), &(color.fx), &color_cx, h_t);
		if (cudaMalloc((void**)&d_depth_data, sizeof(unsigned short) * size_depth) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc d_depth_data" << std::endl;
		}
		if (cudaMalloc(&d_rgb_data, sizeof(unsigned char) * size_color * 4) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc d_rgb_data" << std::endl;
		}
		if (cudaMalloc(&d_filter_map, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc d_filter_map" << std::endl;
		}

		if (cudaMalloc(&d_count, sizeof(unsigned int)) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc d_filter_map" << std::endl;
		}
		if (cudaMalloc(&d_points, sizeof(PointXYZRGBNew) * size_depth) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc points" <<  std::endl;
		}
		if (cudaMalloc(&d_body_index_data, sizeof(unsigned char) * size_depth) != cudaSuccess){
			freeMem();
			std::cout << "cudaMalloc d_body_index_data" << std::endl;
		}
		d_p_filter_map = d_filter_map + offset_filter_map;
		std::cout <<"init:"<< result << std::endl;
	}

	void Registration::apply5(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int nthreads)
	{
		if (pDepthBuffer == NULL || pColorBuffer == NULL)
			return;

		unsigned short *h_depth_data = (unsigned short*)pDepthBuffer;
		unsigned char *h_rgb_data = (unsigned char*)pColorBuffer;
		unsigned int h_count = 0;
		cudaError error;
		
		if ((error = cudaMemcpy(d_depth_data, h_depth_data, sizeof(unsigned short) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
			std::cout << "cudaMemcpy d_depth_data" << error << std::endl;
			return;
		}

		if ((error = cudaMemcpy(d_rgb_data, h_rgb_data, sizeof(unsigned char) * size_color*4, cudaMemcpyHostToDevice)) != cudaSuccess){
			std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
			return ;
		}

		if (cudaMemset(d_filter_map, 65535, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
			std::cout << "cudaMemset d_filter_map" << error << std::endl;
			return ;
		}
		if (cudaMemset(d_count, 0, sizeof(unsigned int)) != cudaSuccess){
			std::cout << "cudaMemset d_count" << error << std::endl;
			return ;
		}
				

		bool result = applyRegistrationCuda1(d_rgb_data, d_depth_data, d_points, d_p_filter_map, d_count, nthreads);
		if (result==true &&(error = cudaMemcpy(&h_count, (void**)d_count, sizeof(unsigned int), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy h_count" << error << std::endl;
			return;
		}
		cloud->resize(h_count);
		cloud->is_dense = true;

		if ((error = cudaMemcpy((void**)&cloud->points[0], (void**)d_points, h_count * sizeof(PointXYZRGBNew), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy points" << error << std::endl;
			return;
		}
	}

	void Registration::apply9(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const BYTE *pBodyIndexBuffer, int* extrapix, int nthreads)
	{
		if (pDepthBuffer == NULL || pColorBuffer == NULL)
			return;

		unsigned short *h_depth_data = (unsigned short*)pDepthBuffer;
		unsigned char *h_rgb_data = (unsigned char*)pColorBuffer;
		unsigned char *h_body_index_data = (unsigned char*)pBodyIndexBuffer;
		unsigned int h_count = 0;
		cudaError error;

		if ((error = cudaMemcpy(d_depth_data, h_depth_data, sizeof(unsigned short) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
			std::cout << "cudaMemcpy d_depth_data" << error << std::endl;
			return;
		}

		if ((error = cudaMemcpy(d_rgb_data, h_rgb_data, sizeof(unsigned char) * size_color * 4, cudaMemcpyHostToDevice)) != cudaSuccess){
			std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
			return;
		}

		if ((error = cudaMemcpy(d_body_index_data, h_body_index_data, sizeof(unsigned char) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
			std::cout << "cudaMemcpy d_body_index_data" << error << std::endl;
			return;
		}

		if (cudaMemset(d_filter_map, 65535, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
			std::cout << "cudaMemset d_filter_map" << error << std::endl;
			return;
		}
		if (cudaMemset(d_count, 0, sizeof(unsigned int)) != cudaSuccess){
			std::cout << "cudaMemset d_count" << error << std::endl;
			return;
		}


		bool result = applyRegistrationCuda3(d_rgb_data, d_depth_data, d_points, d_p_filter_map, d_count, d_body_index_data, nthreads);
		if (result == true && (error = cudaMemcpy(&h_count, (void**)d_count, sizeof(unsigned int), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy h_count" << error << std::endl;
			return;
		}
		cloud->resize(h_count + *extrapix);
		*extrapix = h_count;
		cloud->is_dense = true;
		if ((error = cudaMemcpy((void**)&cloud->points[0], (void**)d_points, h_count * sizeof(PointXYZRGBNew), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy points" << error << std::endl;
			return;
		}
		
	}

	void Registration::apply6(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard)
	{
		if (pDepthBuffer == NULL || pColorBuffer == NULL)
			return;

		unsigned short *h_depth_data = (unsigned short*)pDepthBuffer;
		unsigned char *h_rgb_data = (unsigned char*)pColorBuffer;
		unsigned short *d_depth_data;
		unsigned char *d_rgb_data;
		unsigned int *d_filter_map;
		unsigned int *d_p_filter_map = NULL;
		unsigned int *d_count;
		unsigned int h_count = 0;

		cudaError error;
		if ((error = cudaMalloc((void**)&d_depth_data, sizeof(unsigned short) * size_depth)) != cudaSuccess){
			std::cout << "cudaMalloc d_depth_data" << error << std::endl;
			cudaDeviceReset();
			cudaInit(depth2world);
			return;
		}
		if ((error = cudaMalloc(&d_rgb_data, sizeof(unsigned char) * size_color * 4)) != cudaSuccess){
			cudaFree(d_depth_data);
			std::cout << "cudaMalloc d_rgb_data" << error << std::endl;
			return;
		}
		if ((error = cudaMalloc(&d_filter_map, sizeof(unsigned int) * 2 * size_filter_map)) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			std::cout << "cudaMalloc d_filter_map" << error << std::endl;
			return;
		}
		if ((error = cudaMalloc(&d_count, sizeof(unsigned int))) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			std::cout << "cudaMalloc d_filter_map" << error << std::endl;
			return;
		}

		if ((error = cudaMemcpy(d_depth_data, h_depth_data, sizeof(unsigned short) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			std::cout << "cudaMemcpy d_depth_data" << error << std::endl;
			return;
		}

		if ((error = cudaMemcpy(d_rgb_data, h_rgb_data, sizeof(unsigned char) * size_color * 4, cudaMemcpyHostToDevice)) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
			return;
		}

		if (cudaMemset(d_filter_map, 65535, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			std::cout << "cudaMemset d_filter_map" << error << std::endl;
			return;
		}
		if (cudaMemset(d_count, 0, sizeof(unsigned int)) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			std::cout << "cudaMemset d_count" << error << std::endl;
			return;
		}

		d_p_filter_map = d_filter_map + offset_filter_map;

		pcl::cuda::PointXYZRGB *d_points;

		if ((error = cudaMalloc(&d_points, sizeof(pcl::cuda::PointXYZRGB) * size_depth)) != cudaSuccess){
			cudaFree(d_depth_data);
			cudaFree(d_rgb_data);
			cudaFree(d_filter_map);
			cudaFree(d_count);
			std::cout << "cudaMalloc points" << error << std::endl;
			return;
		}

		bool result = applyRegistrationCuda(d_rgb_data, d_depth_data, d_points, d_p_filter_map, d_count);
		if ((error = cudaMemcpy(&h_count, (void**)d_count, sizeof(unsigned int), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy h_count" << error << std::endl;
		}
		cloud->resize(h_count+1+dashboard->getNumberPixel());
		cloud->is_dense = true;


		pcl::cuda::PointXYZRGB *h_points = new pcl::cuda::PointXYZRGB[h_count];


		if ((error = cudaMemcpy((void**)h_points, (void**)d_points, h_count * sizeof(pcl::cuda::PointXYZRGB), cudaMemcpyDeviceToHost)) != cudaSuccess){
			std::cout << "cudaMemcpy points" << error << std::endl;
		}
		for (int i = 0; i < h_count; i++)
		{
			cloud->points[i].x = h_points[i].x;
			cloud->points[i].y = -h_points[i].y;
			cloud->points[i].z = -h_points[i].z;
			cloud->points[i].r = h_points[i].rgb.r;
			cloud->points[i].g = h_points[i].rgb.g;
			cloud->points[i].b = h_points[i].rgb.b;
		}
		dashboard->draw(h_count, cloud, HandRight);
		cloud->points[h_count+25].x = HandRight->X;
		cloud->points[h_count+25].y = HandRight->Y;
		cloud->points[h_count+25].z = HandRight->Z;
		cloud->points[h_count+25].r = 255;
		cloud->points[h_count+25].g = 255;
		cloud->points[h_count+25].b = 255;

		if ((error = cudaFree(d_depth_data)) != cudaSuccess){
			std::cout << "cudaFree d_depth_data" << error << std::endl;
		}

		if ((error = cudaFree(d_points)) != cudaSuccess){
			std::cout << "cudaFree points" << error << std::endl;
		}
		if ((error = cudaFree(d_rgb_data)) != cudaSuccess){
			std::cout << "cudaFree d_rgb_data" << error << std::endl;
		}
		if ((error = cudaFree(d_filter_map)) != cudaSuccess){
			std::cout << "cudaFree d_filter_map" << error << std::endl;
		}
		if ((error = cudaFree(d_count)) != cudaSuccess){
			std::cout << "cudaFree d_filter_map" << error << std::endl;
		}
		//std::cout << "success" << std::endl;
		delete h_points;
	}

	//void Registration::apply7(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard, const BYTE *pBodyIndexBuffer)
	//{
	//	if (pDepthBuffer == NULL || pColorBuffer == NULL)
	//		return;

	//	unsigned short *h_depth_data = (unsigned short*)pDepthBuffer;
	//	unsigned char *h_rgb_data = (unsigned char*)pColorBuffer;
	//	unsigned char *h_body_index_data = (unsigned char*)pBodyIndexBuffer;
	//	unsigned short *d_depth_data;
	//	unsigned char *d_rgb_data;
	//	unsigned char *d_body_index_data;
	//	unsigned int *d_filter_map;
	//	unsigned int *d_p_filter_map = NULL;
	//	unsigned int *d_count;
	//	unsigned int h_count = 0;

	//	cudaError error;
	//	if ((error = cudaMalloc((void**)&d_depth_data, sizeof(unsigned short) * size_depth)) != cudaSuccess){
	//		std::cout << "cudaMalloc d_depth_data" << error << std::endl;
	//		cudaDeviceReset();
	//		cudaInit(depth2world);
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_rgb_data, sizeof(unsigned char) * size_color * 4)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		std::cout << "cudaMalloc d_rgb_data" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_filter_map, sizeof(unsigned int) * 2 * size_filter_map)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		std::cout << "cudaMalloc d_filter_map" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_count, sizeof(unsigned int))) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		std::cout << "cudaMalloc d_filter_map" << error << std::endl;
	//		return;
	//	}

	//	if ((error = cudaMemcpy(d_depth_data, h_depth_data, sizeof(unsigned short) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemcpy d_depth_data" << error << std::endl;
	//		return;
	//	}

	//	if ((error = cudaMemcpy(d_rgb_data, h_rgb_data, sizeof(unsigned char) * size_color * 4, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
	//		return;
	//	}

	//	if (cudaMemset(d_filter_map, 65535, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemset d_filter_map" << error << std::endl;
	//		return;
	//	}
	//	if (cudaMemset(d_count, 0, sizeof(unsigned int)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemset d_count" << error << std::endl;
	//		return;
	//	}

	//	d_p_filter_map = d_filter_map + offset_filter_map;

	//	pcl::cuda::PointXYZRGB *d_points;

	//	if ((error = cudaMalloc(&d_points, sizeof(pcl::cuda::PointXYZRGB) * size_depth)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMalloc points" << error << std::endl;
	//		return;
	//	}
	//	///////////////
	//	if ((error = cudaMalloc(&d_body_index_data, sizeof(unsigned char) * size_depth)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		cudaFree(d_points);
	//		std::cout << "cudaMalloc d_body_index_data" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMemcpy(d_body_index_data, h_body_index_data, sizeof(unsigned char) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		cudaFree(d_points);
	//		std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
	//		return;
	//	}
	//	/////////////////////
	//	bool result = applyRegistrationCuda3(d_rgb_data, d_depth_data, d_points, d_p_filter_map, d_count, d_body_index_data);
	//	if ((error = cudaMemcpy(&h_count, (void**)d_count, sizeof(unsigned int), cudaMemcpyDeviceToHost)) != cudaSuccess){
	//		std::cout << "cudaMemcpy h_count" << error << std::endl;
	//	}
	//	cloud->resize(h_count + 1 + dashboard->getNumberPixel());
	//	cloud->is_dense = true;


	//	pcl::cuda::PointXYZRGB *h_points = new pcl::cuda::PointXYZRGB[h_count];


	//	if ((error = cudaMemcpy((void**)h_points, (void**)d_points, h_count * sizeof(pcl::cuda::PointXYZRGB), cudaMemcpyDeviceToHost)) != cudaSuccess){
	//		std::cout << "cudaMemcpy points" << error << std::endl;
	//	}
	//	for (int i = 0; i < h_count; i++)
	//	{
	//		cloud->points[i].x = h_points[i].x;
	//		cloud->points[i].y = -h_points[i].y;
	//		cloud->points[i].z = -h_points[i].z;
	//		cloud->points[i].r = h_points[i].rgb.r;
	//		cloud->points[i].g = h_points[i].rgb.g;
	//		cloud->points[i].b = h_points[i].rgb.b;
	//	}
	//	dashboard->draw(h_count, cloud, HandRight);
	//	cloud->points[h_count + 25].x = HandRight->X;
	//	cloud->points[h_count + 25].y = HandRight->Y;
	//	cloud->points[h_count + 25].z = -HandRight->Z;
	//	cloud->points[h_count + 25].r = 255;
	//	cloud->points[h_count + 25].g = 255;
	//	cloud->points[h_count + 25].b = 255;

	//	if ((error = cudaFree(d_depth_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_depth_data" << error << std::endl;
	//	}

	//	if ((error = cudaFree(d_points)) != cudaSuccess){
	//		std::cout << "cudaFree points" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_rgb_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_rgb_data" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_filter_map)) != cudaSuccess){
	//		std::cout << "cudaFree d_filter_map" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_count)) != cudaSuccess){
	//		std::cout << "cudaFree d_filter_map" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_body_index_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_body_index_data" << error << std::endl;
	//	}
	//	//std::cout << "success" << std::endl;
	//	delete h_points;
	//}

	//void Registration::apply8(const UINT16 *pDepthBuffer, const BYTE *pColorBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* HandRight, Dashboard* dashboard, const BYTE *pBodyIndexBuffer, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
	//{
	//	if (pDepthBuffer == NULL || pColorBuffer == NULL)
	//		return;

	//	unsigned short *h_depth_data = (unsigned short*)pDepthBuffer;
	//	unsigned char *h_rgb_data = (unsigned char*)pColorBuffer;
	//	unsigned char *h_body_index_data = (unsigned char*)pBodyIndexBuffer;
	//	unsigned short *d_depth_data;
	//	unsigned char *d_rgb_data;
	//	unsigned char *d_body_index_data;
	//	unsigned int *d_filter_map;
	//	unsigned int *d_p_filter_map = NULL;
	//	unsigned int *d_count;
	//	unsigned int h_count = 0;

	//	cudaError error;
	//	if ((error = cudaMalloc((void**)&d_depth_data, sizeof(unsigned short) * size_depth)) != cudaSuccess){
	//		std::cout << "cudaMalloc d_depth_data" << error << std::endl;
	//		cudaDeviceReset();
	//		cudaInit(depth2world);
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_rgb_data, sizeof(unsigned char) * size_color * 4)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		std::cout << "cudaMalloc d_rgb_data" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_filter_map, sizeof(unsigned int) * 2 * size_filter_map)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		std::cout << "cudaMalloc d_filter_map" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMalloc(&d_count, sizeof(unsigned int))) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		std::cout << "cudaMalloc d_filter_map" << error << std::endl;
	//		return;
	//	}

	//	if ((error = cudaMemcpy(d_depth_data, h_depth_data, sizeof(unsigned short) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemcpy d_depth_data" << error << std::endl;
	//		return;
	//	}

	//	if ((error = cudaMemcpy(d_rgb_data, h_rgb_data, sizeof(unsigned char) * size_color * 4, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
	//		return;
	//	}

	//	if (cudaMemset(d_filter_map, 65535, sizeof(unsigned int) * 2 * size_filter_map) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemset d_filter_map" << error << std::endl;
	//		return;
	//	}
	//	if (cudaMemset(d_count, 0, sizeof(unsigned int)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMemset d_count" << error << std::endl;
	//		return;
	//	}

	//	d_p_filter_map = d_filter_map + offset_filter_map;

	//	pcl::cuda::PointXYZRGB *d_points;

	//	if ((error = cudaMalloc(&d_points, sizeof(pcl::cuda::PointXYZRGB) * size_depth)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		std::cout << "cudaMalloc points" << error << std::endl;
	//		return;
	//	}
	//	///////////////
	//	if ((error = cudaMalloc(&d_body_index_data, sizeof(unsigned char) * size_depth)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		cudaFree(d_points);
	//		std::cout << "cudaMalloc d_body_index_data" << error << std::endl;
	//		return;
	//	}
	//	if ((error = cudaMemcpy(d_body_index_data, h_body_index_data, sizeof(unsigned char) * size_depth, cudaMemcpyHostToDevice)) != cudaSuccess){
	//		cudaFree(d_depth_data);
	//		cudaFree(d_rgb_data);
	//		cudaFree(d_filter_map);
	//		cudaFree(d_count);
	//		cudaFree(d_points);
	//		std::cout << "cudaMemcpy d_rgb_data" << error << std::endl;
	//		return;
	//	}
	//	/////////////////////
	//	bool result = applyRegistrationCuda3(d_rgb_data, d_depth_data, d_points, d_p_filter_map, d_count, d_body_index_data);
	//	if ((error = cudaMemcpy(&h_count, (void**)d_count, sizeof(unsigned int), cudaMemcpyDeviceToHost)) != cudaSuccess){
	//		std::cout << "cudaMemcpy h_count" << error << std::endl;
	//	}
	//	cloud->resize(h_count + 1 + dashboard->getNumberPixel());
	//	cloud->is_dense = true;


	//	pcl::cuda::PointXYZRGB *h_points = new pcl::cuda::PointXYZRGB[h_count];


	//	if ((error = cudaMemcpy((void**)h_points, (void**)d_points, h_count * sizeof(pcl::cuda::PointXYZRGB), cudaMemcpyDeviceToHost)) != cudaSuccess){
	//		std::cout << "cudaMemcpy points" << error << std::endl;
	//	}
	//	for (int i = 0; i < h_count; i++)
	//	{
	//		cloud->points[i].x = h_points[i].x;
	//		cloud->points[i].y = -h_points[i].y;
	//		cloud->points[i].z = -h_points[i].z;
	//		cloud->points[i].r = h_points[i].rgb.r;
	//		cloud->points[i].g = h_points[i].rgb.g;
	//		cloud->points[i].b = h_points[i].rgb.b;
	//	}
	//	dashboard->draw(h_count, cloud, HandRight);
	//	cloud->points[h_count + dashboard->getNumberPixel()].x = HandRight->X;
	//	cloud->points[h_count + dashboard->getNumberPixel()].y = HandRight->Y;
	//	cloud->points[h_count + dashboard->getNumberPixel()].z = -HandRight->Z;
	//	cloud->points[h_count + dashboard->getNumberPixel()].r = 255;
	//	cloud->points[h_count + dashboard->getNumberPixel()].g = 255;
	//	cloud->points[h_count + dashboard->getNumberPixel()].b = 255;


	//	if ((error = cudaFree(d_depth_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_depth_data" << error << std::endl;
	//	}

	//	if ((error = cudaFree(d_points)) != cudaSuccess){
	//		std::cout << "cudaFree points" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_rgb_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_rgb_data" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_filter_map)) != cudaSuccess){
	//		std::cout << "cudaFree d_filter_map" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_count)) != cudaSuccess){
	//		std::cout << "cudaFree d_filter_map" << error << std::endl;
	//	}
	//	if ((error = cudaFree(d_body_index_data)) != cudaSuccess){
	//		std::cout << "cudaFree d_body_index_data" << error << std::endl;
	//	}
	//	//std::cout << "success" << std::endl;
	//	delete h_points;
	//}


	Registration::Registration(const std::string param_file) :
		 filter_width_half(2), filter_height_half(1), filter_tolerance(0.01f)
	{
		cv::FileStorage fs;
		fs.open(param_file, cv::FileStorage::READ);
		if (fs.isOpened())
		{
			fs["depth_p.cx"] >> depth.cx;
			fs["depth_p.cy"] >> depth.cy;
			fs["depth_p.fx"] >> depth.fx;
			fs["depth_p.fy"] >> depth.fy;
			fs["depth_p.k1"] >> depth.k1;
			fs["depth_p.k2"] >> depth.k2;
			fs["depth_p.k3"] >> depth.k3;
			fs["depth_p.p1"] >> depth.p1;
			fs["depth_p.p2"] >> depth.p2;
			fs["rgb_p.cx"] >> color.cx;
			fs["rgb_p.cy"] >> color.cy;
			fs["rgb_p.fx"] >> color.fx;
			fs["rgb_p.fy"] >> color.fy;
			fs["rgb_p.mx_x0y0"] >> color.mx_x0y0;
			fs["rgb_p.mx_x0y1"] >> color.mx_x0y1;
			fs["rgb_p.mx_x0y2"] >> color.mx_x0y2;
			fs["rgb_p.mx_x0y3"] >> color.mx_x0y3;
			fs["rgb_p.mx_x1y0"] >> color.mx_x1y0;
			fs["rgb_p.mx_x1y1"] >> color.mx_x1y1;
			fs["rgb_p.mx_x1y2"] >> color.mx_x1y2;
			fs["rgb_p.mx_x2y0"] >> color.mx_x2y0;
			fs["rgb_p.mx_x2y1"] >> color.mx_x2y1;
			fs["rgb_p.mx_x3y0"] >> color.mx_x3y0;
			fs["rgb_p.my_x0y0"] >> color.my_x0y0;
			fs["rgb_p.my_x0y1"] >> color.my_x0y1;
			fs["rgb_p.my_x0y2"] >> color.my_x0y2;
			fs["rgb_p.my_x0y3"] >> color.my_x0y3;
			fs["rgb_p.my_x1y0"] >> color.my_x1y0;
			fs["rgb_p.my_x1y1"] >> color.my_x1y1;
			fs["rgb_p.my_x1y2"] >> color.my_x1y2;
			fs["rgb_p.my_x2y0"] >> color.my_x2y0;
			fs["rgb_p.my_x2y1"] >> color.my_x2y1;
			fs["rgb_p.my_x3y0"] >> color.my_x3y0;
			fs["rgb_p.shift_d"] >> color.shift_d;
			fs["rgb_p.shift_m"] >> color.shift_m;
			fs.release();
		}
		else{
			std::cout << "could not find rgb calibration file " << param_file << std::endl;
			exit(-1);
		}
		float mx, my;
		int ix, iy, index;
		float rx, ry;
		int *map_dist = distort_map;
		float *map_x = depth_to_color_map_x;
		float *map_y = depth_to_color_map_y;
		int *map_yi = depth_to_color_map_yi;

		for (int y = 0; y < 424; y++) {
			for (int x = 0; x < 512; x++) {
				distort(x, y, mx, my);
				ix = (int)(mx + 0.5f);
				iy = (int)(my + 0.5f);
				if (ix < 0 || ix >= 512 || iy < 0 || iy >= 424)
					index = -1;
				else
					index = iy * 512 + ix;
				*map_dist++ = index;

				depth_to_color(x, y, rx, ry);
				*map_x++ = rx;
				*map_y++ = ry;
				*map_yi++ = (int)(ry + 0.5f);
			}
		}
	partial_clouds_2.reserve(size_x * size_y);
	size_filter_map = size_color + 1920 * filter_height_half * 2;
	offset_filter_map = 1920 * filter_height_half;
	color_cx = color.cx + 0.5f;
	}
