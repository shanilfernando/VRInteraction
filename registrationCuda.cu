
#include "..\registrationCuda.h"

const int limit = 512 * 424;
//const int size_color = 1920 * 1080;
//const int size_filter_map = size_color + 1920 * 2;
//const int offset_filter_map = 1920;
int *d_map_dist;
int *d_map_yi;
float *d_map_x;
float *d_color_shift_m;
float *d_color_fx;
float *d_color_cx;
double *d_t;
unsigned int *d_rgb_data;
int *d_map_c_off;



__global__ void Apply(int *d_map_dist, float *d_depth_data, float *d_map_x, float *d_color_shift_m, float *d_color_fx, float *d_color_cx, int *d_map_yi, unsigned int *d_rgb_data, double *d_t, float *d_p_filter_map, pcl::cuda::PointXYZRGB *points, unsigned int *d_count){
	int id = blockIdx.x*blockDim.x + threadIdx.x;
	if (id<limit){
		
		// getting index of distorted depth pixel
		const int index = d_map_dist[id];
		// check if distorted depth pixel is outside of the depth image
		if (index > 0){

			// getting depth value for current pixel
			const float z = d_depth_data[index];
			// checking for invalid depth value
			if (z > 0.0f){
				// calculating x offset for rgb image based on depth value
				const float rx = (d_map_x[id]+ ((*d_color_shift_m) / z)) * (*d_color_fx) + (*d_color_cx);
				const int cx = rx; // same as round for positive numbers (0.5f was already added to color_cx)
				// getting y offset for depth image
				const int cy = d_map_yi[id];
				// combining offsets
				const int c_off = cx + cy * 1920;

				// check if c_off is outside of rgb image
				// checking rx/cx is not needed because the color image is much wider then the depth image
				if (c_off >= 0 && c_off < (1920 * 1080)){
					// setting a window around the filter map pixel corresponding to the color pixel with the current z value
					int yi = (cy - 1) * 1920 + cx - 2; // index of first pixel to set
					for (int r = -1; r <= 1; ++r, yi += 1920) // index increased by a full row each iteration
					{
						float *it = d_p_filter_map + yi;
						for (int c = -2; c <= 2; ++c, ++it)
						{
							// only set if the current z is smaller
							if (z < *it)
								*it = z;
						}
					}

					const float min_z = d_p_filter_map[c_off];
					unsigned int color = (z - min_z) / z > 0.01f ? 0 : *(d_rgb_data + c_off);
					const float depth_value = z / 1000.0f;
					int y = id / 512;
					int x = id % 512;
					double p[4] = { x*depth_value, y*depth_value, 1.0*depth_value, 1.0 };
					double cor[4] =	  { d_t[0] * p[0] + d_t[4] * p[1] + d_t[8] * p[2] + d_t[12] * p[3],
										d_t[1] * p[0] + d_t[5] * p[1] + d_t[9] * p[2] + d_t[13] * p[3],
										d_t[2] * p[0] + d_t[6] * p[1] + d_t[10] * p[2] + d_t[14] * p[3],
										d_t[3] * p[0] + d_t[7] * p[1] + d_t[11] * p[2] + d_t[15] * p[3] };
					pcl::cuda::PointXYZRGB itP;
					itP.z = depth_value;
					itP.x = cor[0];
					itP.y = cor[1];
					itP.rgb.r = ((color >> 16) & 0xff);
					itP.rgb.g = ((color >> 8) & 0xff);
					itP.rgb.b = (color & 0xff);
					unsigned int temp=atomicAdd(d_count, 1);
					points[temp] = itP;
				}
				
				
			}

			
		}
		
	}
}



__global__ void Apply1(int *d_map_dist, unsigned short *d_depth_data, float *d_map_x, float *d_color_shift_m, float *d_color_fx, float *d_color_cx, int *d_map_yi, unsigned int *d_p_filter_map, int *d_map_c_off){
	int id = blockIdx.x*blockDim.x + threadIdx.x;
	if (id<limit){
		
		// getting index of distorted depth pixel
		const int index = d_map_dist[id];
		// check if distorted depth pixel is outside of the depth image
		if (index >= 0){

			// getting depth value for current pixel
			const float z = d_depth_data[index];
			// checking for invalid depth value
			if (z > 0.0f){
				// calculating x offset for rgb image based on depth value
				const float rx = (d_map_x[id] + ((*d_color_shift_m) / z)) * (*d_color_fx) + (*d_color_cx);
				const int cx = rx; // same as round for positive numbers (0.5f was already added to color_cx)
				// getting y offset for depth image
				const int cy = d_map_yi[id];
				// combining offsets
				const int c_off = cx + cy * 1920;
				
				// check if c_off is outside of rgb image
				// checking rx/cx is not needed because the color image is much wider then the depth image
				if (c_off >= 0 && c_off < (1920 * 1080)){
					d_map_c_off[id] = c_off;
					// setting a window around the filter map pixel corresponding to the color pixel with the current z value
					int yi = (cy - 1) * 1920 + cx - 2; // index of first pixel to set
					unsigned int zTemp = z * 100;
					for (int r = -1; r <= 1; ++r, yi += 1920) // index increased by a full row each iteration
					{
						unsigned int *it = d_p_filter_map + yi;
						for (int c = -2; c <= 2; ++c, ++it)
						{
							atomicMin(it, zTemp);
						}
					}
				}
				else
				{
					d_map_c_off[id] = -1;
				}


			}
			else
			{
				d_map_c_off[id] = -1;
			}

		}
		else
		{
			d_map_c_off[id] = -1;
		}

	}
}

__global__ void Apply3(int *d_map_dist, unsigned short *d_depth_data, float *d_map_x, float *d_color_shift_m, float *d_color_fx, float *d_color_cx, int *d_map_yi, unsigned int *d_p_filter_map, int *d_map_c_off, unsigned char *d_body_index_data){
	int id = blockIdx.x*blockDim.x + threadIdx.x;
	if (id<limit){

		// getting index of distorted depth pixel
		const int index = d_map_dist[id];
		// check if distorted depth pixel is outside of the depth image
		if (index >= 0){

			// getting depth value for current pixel
			const float z = d_depth_data[index];
			unsigned char pix=d_body_index_data[index];
			// checking for invalid depth value
			if (z > 0.0f && pix != 0xff){
				// calculating x offset for rgb image based on depth value
				const float rx = (d_map_x[id] + ((*d_color_shift_m) / z)) * (*d_color_fx) + (*d_color_cx);
				const int cx = rx; // same as round for positive numbers (0.5f was already added to color_cx)
				// getting y offset for depth image
				const int cy = d_map_yi[id];
				// combining offsets
				const int c_off = cx + cy * 1920;

				// check if c_off is outside of rgb image
				// checking rx/cx is not needed because the color image is much wider then the depth image
				if (c_off >= 0 && c_off < (1920 * 1080)){
					d_map_c_off[id] = c_off;
					// setting a window around the filter map pixel corresponding to the color pixel with the current z value
					int yi = (cy - 1) * 1920 + cx - 2; // index of first pixel to set
					unsigned int zTemp = z * 100;
					for (int r = -1; r <= 1; ++r, yi += 1920) // index increased by a full row each iteration
					{
						unsigned int *it = d_p_filter_map + yi;
						for (int c = -2; c <= 2; ++c, ++it)
						{
							atomicMin(it, zTemp);
						}
					}
				}
				else
				{
					d_map_c_off[id] = -1;
				}


			}
			else
			{
				d_map_c_off[id] = -1;
			}

		}
		else
		{
			d_map_c_off[id] = -1;
		}

	}
}



__global__ void Apply2(int *d_map_dist, unsigned short *d_depth_data, unsigned char *d_rgb_data, double *d_t, unsigned int *d_p_filter_map, pcl::cuda::PointXYZRGB *points, unsigned int *d_count, int *d_map_c_off){
	int id = blockIdx.x*blockDim.x + threadIdx.x;
	if (id < limit){
		int c_off = d_map_c_off[id];
		if (c_off>=0){
			const float min_z = d_p_filter_map[c_off]/100;
			int index=d_map_dist[id];
			unsigned short z = d_depth_data[index];
			if ((z - min_z) / z <= 0.01f){
				unsigned char* color=(d_rgb_data + c_off * 4);
				const float depth_value = z / 1000.0f;
				int y = id / 512;
				int x = id % 512;
				double p[4] = { x*depth_value, y*depth_value, 1.0*depth_value, 1.0 };
				double cor[4] = { d_t[0] * p[0] + d_t[4] * p[1] + d_t[8] * p[2] + d_t[12] * p[3],
					d_t[1] * p[0] + d_t[5] * p[1] + d_t[9] * p[2] + d_t[13] * p[3],
					d_t[2] * p[0] + d_t[6] * p[1] + d_t[10] * p[2] + d_t[14] * p[3],
					d_t[3] * p[0] + d_t[7] * p[1] + d_t[11] * p[2] + d_t[15] * p[3] };
				pcl::cuda::PointXYZRGB itP;
				itP.z = depth_value;
				itP.x = cor[0];
				itP.y = cor[1];
				itP.rgb.r = *color;// *(d_rgb_data + c_off * 4);
				itP.rgb.g = *(color+1);//*(d_rgb_data + c_off*4+1);
				itP.rgb.b = *(color + 2);//*(d_rgb_data + c_off*4+2);
				unsigned int temp = atomicAdd(d_count, 1);
				points[temp] = itP;
			}			
		}
	}
}

__global__ void Apply21(int *d_map_dist, unsigned short *d_depth_data, unsigned char *d_rgb_data, double *d_t, unsigned int *d_p_filter_map, PointXYZRGBNew *points, unsigned int *d_count, int *d_map_c_off){
	int id = blockIdx.x*blockDim.x + threadIdx.x;
	if (id < limit){
		int c_off = d_map_c_off[id];
		if (c_off >= 0){
			const float min_z = d_p_filter_map[c_off] / 100;
			int index = d_map_dist[id];
			unsigned short z = d_depth_data[index];
			if ((z - min_z) / z <= 0.01f){
				unsigned char* color = (d_rgb_data + c_off * 4);
				const float depth_value = z / 1000.0f;
				int y = id / 512;
				int x = id % 512;
				double p[4] = { x*depth_value, y*depth_value, 1.0*depth_value, 1.0 };
				double cor[4] = { d_t[0] * p[0] + d_t[4] * p[1] + d_t[8] * p[2] + d_t[12] * p[3],
					d_t[1] * p[0] + d_t[5] * p[1] + d_t[9] * p[2] + d_t[13] * p[3],
					d_t[2] * p[0] + d_t[6] * p[1] + d_t[10] * p[2] + d_t[14] * p[3],
					d_t[3] * p[0] + d_t[7] * p[1] + d_t[11] * p[2] + d_t[15] * p[3] };
				PointXYZRGBNew itP;
				itP.z = depth_value;
				itP.x = cor[0];
				itP.y = cor[1];
				itP.r = *color;// *(d_rgb_data + c_off * 4);
				itP.g = *(color + 1);//*(d_rgb_data + c_off*4+1);
				itP.b = *(color + 2);//*(d_rgb_data + c_off*4+2);
				unsigned int temp = atomicAdd(d_count, 1);
				points[temp] = itP;
			}
		}
	}
}

bool init(int *h_map_dist, float *h_map_x, int *h_map_yi, float *h_color_shift_m, float *h_color_fx, float *h_color_cx, double *h_t){
	//allocate memory in device
	if (cudaMalloc(&d_map_dist, sizeof(int)*limit) != cudaSuccess){
		return false;
	}
	if (cudaMalloc(&d_map_x, sizeof(float)*limit) != cudaSuccess){
		cudaFree(d_map_dist);
		return false;
	}
	if (cudaMalloc(&d_map_yi, sizeof(int)*limit) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		return false;
	}
	if (cudaMalloc(&d_color_shift_m, sizeof(float)) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		cudaFree(d_map_yi);
		return false;
	}
	if (cudaMalloc(&d_color_fx, sizeof(float)) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		cudaFree(d_map_yi);
		cudaFree(d_color_shift_m);
		return false;
	}
	if (cudaMalloc(&d_color_cx, sizeof(float)) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		cudaFree(d_map_yi);
		cudaFree(d_color_shift_m);
		cudaFree(d_color_fx);
		return false;
	}
	if (cudaMalloc(&d_t, sizeof(double)*16) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		cudaFree(d_map_yi);
		cudaFree(d_color_shift_m);
		cudaFree(d_color_fx);
		cudaFree(d_color_cx);
		return false;
	}
	if (cudaMalloc(&d_map_c_off, sizeof(int) * limit) != cudaSuccess){
		cudaFree(d_map_dist);
		cudaFree(d_map_x);
		cudaFree(d_map_yi);
		cudaFree(d_color_shift_m);
		cudaFree(d_color_fx);
		cudaFree(d_color_cx);
		cudaFree(d_t);
		return false;
	}

	//copy memory to device
	if (cudaMemcpy(d_map_dist, h_map_dist, sizeof(int)*limit,cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_map_x, h_map_x, sizeof(float)*limit, cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_map_yi, h_map_yi, sizeof(int)*limit, cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_color_shift_m, h_color_shift_m, sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_color_fx, h_color_fx, sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_color_cx, h_color_cx, sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	if (cudaMemcpy(d_t,h_t, sizeof(double)*16, cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	return true;
}

//free memory in device
void freeCudaMem(){
	cudaFree(d_map_dist);
	cudaFree(d_map_x);
	cudaFree(d_map_yi);
	cudaFree(d_color_shift_m);
	cudaFree(d_color_fx);
	cudaFree(d_color_cx);
	cudaFree(d_t);
	cudaFree(d_map_c_off);
}
bool applyRegistrationCuda(unsigned char *d_rgb_data, unsigned short *d_depth_data, pcl::cuda::PointXYZRGB *points, unsigned int *d_p_filter_map, unsigned int *d_count){
	//Apply << <(limit / 424) + 1, 424 >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_rgb_data, d_t, d_p_filter_map, points, d_count);
	Apply1 << <(limit / 424) + 1, 424 >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_p_filter_map, d_map_c_off);
	cudaDeviceSynchronize();
	Apply2 << <(limit / 424) + 1, 424 >> >(d_map_dist,d_depth_data, d_rgb_data, d_t, d_p_filter_map, points, d_count, d_map_c_off);
	cudaDeviceSynchronize();
	return true;
}
bool applyRegistrationCuda2(unsigned char *d_rgb_data, unsigned short *d_depth_data, pcl::cuda::PointXYZRGB *points, unsigned int *d_p_filter_map, unsigned int *d_count, double *h_t){
	if (cudaMemcpy(d_t, h_t, sizeof(double) * 16, cudaMemcpyHostToDevice) != cudaSuccess){
		freeCudaMem();
		return false;
	}
	Apply1 << <(limit / 424) + 1, 424 >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_p_filter_map, d_map_c_off);
	cudaDeviceSynchronize();
	Apply2 << <(limit / 424) + 1, 424 >> >(d_map_dist, d_depth_data, d_rgb_data, d_t, d_p_filter_map, points, d_count, d_map_c_off);
	cudaDeviceSynchronize();
	return true;
}

bool applyRegistrationCuda3(unsigned char *d_rgb_data, unsigned short *d_depth_data, PointXYZRGBNew *points, unsigned int *d_p_filter_map, unsigned int *d_count, unsigned char *d_body_index_data, int nthreads){
	Apply3 << <(limit / nthreads), nthreads >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_p_filter_map, d_map_c_off, d_body_index_data);
	cudaDeviceSynchronize();
	Apply21 << <(limit / nthreads), nthreads >> >(d_map_dist, d_depth_data, d_rgb_data, d_t, d_p_filter_map, points, d_count, d_map_c_off);
	cudaDeviceSynchronize();
	return true;
}

bool applyRegistrationCuda1(unsigned char *d_rgb_data, unsigned short *d_depth_data, PointXYZRGBNew *points, unsigned int *d_p_filter_map, unsigned int *d_count, int nthreads){
	//Apply << <(limit / 424) + 1, 424 >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_rgb_data, d_t, d_p_filter_map, points, d_count);
	Apply1 << <(limit / nthreads), nthreads >> >(d_map_dist, d_depth_data, d_map_x, d_color_shift_m, d_color_fx, d_color_cx, d_map_yi, d_p_filter_map, d_map_c_off);
	cudaDeviceSynchronize();
	Apply21 << <(limit / nthreads), nthreads >> >(d_map_dist, d_depth_data, d_rgb_data, d_t, d_p_filter_map, points, d_count, d_map_c_off);
	cudaDeviceSynchronize();
	return true;
}
