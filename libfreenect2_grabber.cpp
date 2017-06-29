#include "libfreenect2_grabber.h"



libfreenect2_grabber::libfreenect2_grabber()
{
	loadCalibration("../calibration/rgb_calibration.yaml", "../calibration/depth_calibration.yaml", "../calibration/pose_calibration.yaml");
	initSizeAndData();
	initRotoTranslation();
}
libfreenect2_grabber::libfreenect2_grabber(const std::string rgb_calibration_file, const std::string depth_calibration_file, const std::string pose_calibration_file)
{	
	loadCalibration(rgb_calibration_file, depth_calibration_file, pose_calibration_file);
	initSizeAndData();
	initRotoTranslation();
}


libfreenect2_grabber::~libfreenect2_grabber()
{	
}

void libfreenect2_grabber::initRotoTranslation()
{
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > erotation((double*)(rotation_.data));
	Eigen::Matrix3d rotation = erotation;
	Eigen::Map<Eigen::Vector3d> etranslation((double*)(translation_.data));
	Eigen::Vector3d translation = etranslation;
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > edepth_matrix((double*)(depth_camera_matrix_.data));
	Eigen::Matrix3d depth_matrix = edepth_matrix;
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > ergb_matrix((double*)(rgb_camera_matrix_.data));
	Eigen::Matrix3d rgb_matrix = ergb_matrix;

	Eigen::Matrix4d rototranslation = Eigen::Matrix4d::Zero();
	Eigen::Matrix4d odepth_matrix = Eigen::Matrix4d::Zero();
	Eigen::Matrix4d orgb_matrix = Eigen::Matrix4d::Zero();

	rototranslation.block<3, 3>(0, 0) = rotation;
	rototranslation.block<3, 1>(0, 3) = translation;
	rototranslation(3, 3) = 1;

	odepth_matrix.block<3, 3>(0, 0) = depth_matrix;
	odepth_matrix(3, 3) = 1;

	orgb_matrix.block<3, 3>(0, 0) = rgb_matrix;
	orgb_matrix(3, 3) = 1;

	world2rgb_ = orgb_matrix * rototranslation;
	depth2world_ = odepth_matrix.inverse();
}

void libfreenect2_grabber::initSizeAndData()
{

	const float sx_depth = ((float)size_x / (float)calibsize_depth_.width);
	const float sy_depth = ((float)size_y / (float)calibsize_depth_.height);
	const float sx_rgb = ((float)size_x / (float)calibsize_rgb_.width);
	const float sy_rgb = ((float)size_y / (float)calibsize_rgb_.height);

	depth_camera_matrix_ = calibdepth_camera_matrix_;
	depth_camera_matrix_.at<double>(0, 0) *= sx_depth;
	depth_camera_matrix_.at<double>(1, 1) *= sy_depth;
	depth_camera_matrix_.at<double>(0, 2) *= sx_depth;
	depth_camera_matrix_.at<double>(1, 2) *= sy_depth;

	//need to rescale since rgb image is resized
	rgb_camera_matrix_ = calibrgb_camera_matrix_;
	rgb_camera_matrix_.at<double>(0, 0) *= sx_rgb;
	rgb_camera_matrix_.at<double>(1, 1) *= sy_rgb;
	rgb_camera_matrix_.at<double>(0, 2) *= sx_rgb;
	rgb_camera_matrix_.at<double>(1, 2) *= sy_rgb;

	ir_fx_ = depth_camera_matrix_.at<double>(0, 0);
	ir_fy_ = depth_camera_matrix_.at<double>(1, 1);
	ir_cx_ = depth_camera_matrix_.at<double>(0, 2);
	ir_cy_ = depth_camera_matrix_.at<double>(1, 2);

	rgb_fx_ = rgb_camera_matrix_.at<double>(0, 0);
	rgb_fy_ = rgb_camera_matrix_.at<double>(1, 1);
	rgb_cx_ = rgb_camera_matrix_.at<double>(0, 2);
	rgb_cy_ = rgb_camera_matrix_.at<double>(1, 2);

	cv::initUndistortRectifyMap(depth_camera_matrix_, depth_distortion_, cv::Mat(), depth_camera_matrix_, cv::Size(size_x, size_y), CV_32FC1, map_x_depth_, map_y_depth_);
}

void libfreenect2_grabber::loadCalibration(const std::string rgb_calibration_file, const std::string depth_calibration_file, const std::string pose_calibration_file){

	cv::FileStorage fs;
	fs.open(rgb_calibration_file, cv::FileStorage::READ);
	int x, y;
	if (fs.isOpened())
	{
		fs["image_width"] >> x;
		fs["image_height"] >> y;
		calibsize_rgb_ = cv::Size(x, y);
		fs["camera_matrix"] >> calibrgb_camera_matrix_;
		fs["distortion_coefficients"] >> rgb_distortion_;
		fs.release();
	}
	else{
		std::cout << "could not find rgb calibration file " << rgb_calibration_file << std::endl;
		exit(-1);
	}

	fs.open(depth_calibration_file, cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["image_width"] >> x;
		fs["image_height"] >> y;
		calibsize_depth_ = cv::Size(x, y);
		fs["camera_matrix"] >> calibdepth_camera_matrix_;
		fs["distortion_coefficients"] >> depth_distortion_;
		fs.release();
	}
	else{
		std::cout << "could not find ir calibration file " << depth_calibration_file << std::endl;
		exit(-1);
	}

	fs.open(pose_calibration_file, cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["rotation matrix"] >> rotation_;
		fs["translation matrix"] >> translation_;
		fs["fundamental matrix"] >> fundamental_;
		fs["essential matrix"] >> essential_;
		fs.release();
	}
	else{
		std::cout << "could not find pose calibration file " << pose_calibration_file << std::endl;
		exit(-1);
	}
}