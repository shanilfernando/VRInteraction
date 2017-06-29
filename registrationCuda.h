#include "cuda_runtime.h"
#include "thrust\device_vector.h"
#include "pcl/cuda/point_types.h"
#include "pcl\cuda\point_cloud.h"
#include "device_launch_parameters.h"
#include "device_functions.h"

struct __align__(16) PointXYZRGBNew
{
	inline __host__ __device__ PointXYZRGBNew() {}
	inline __host__ __device__ PointXYZRGBNew(float _x, float _y, float _z, int _rgb) :
		x(_x), y(_y), z(_z), rgb(_rgb) {}

	// Declare a union for XYZ
	union
	{
		float data[4]; \
		struct {
			\
				float x; \
				float y; \
				float z; \
		}; \
	};
	union \
	{ \
	union \
	{ \
	struct \
	{ \
	uint8_t b; \
	uint8_t g; \
	uint8_t r; \
	uint8_t a; \
	}; \
	float rgb; \
	}; \
	uint32_t rgba; \
	};
};

bool init(int *h_map_dist, float *h_map_x, int *h_map_yi, float *h_color_shift_m, float *h_color_fx, float *h_color_cx, double *h_t);

bool applyRegistrationCuda(unsigned char *d_rgb_data, unsigned short *d_depth_data, pcl::cuda::PointXYZRGB *points, unsigned int *d_p_filter_map, unsigned int *d_count);
bool applyRegistrationCuda2(unsigned char *d_rgb_data, unsigned short *d_depth_data, pcl::cuda::PointXYZRGB *points, unsigned int *d_p_filter_map, unsigned int *d_count, double *h_t);
bool applyRegistrationCuda3(unsigned char *d_rgb_data, unsigned short *d_depth_data, PointXYZRGBNew *points, unsigned int *d_p_filter_map, unsigned int *d_count, unsigned char *d_body_index_data, int nthreads);
bool applyRegistrationCuda1(unsigned char *d_rgb_data, unsigned short *d_depth_data, PointXYZRGBNew *points, unsigned int *d_p_filter_map, unsigned int *d_count, int nthreads);

void freeCudaMem();
