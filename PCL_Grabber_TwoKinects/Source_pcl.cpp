#include <k4a/k4a.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "k4a_grabber.h"

using namespace std;
using namespace boost;
using namespace pcl;
using namespace Eigen;
using namespace glm;

typedef pcl::PointXYZRGB PointType;

template<typename DataType>
Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> load_csv(const std::string & path)
{
	std::ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<DataType> values;
	unsigned int rows = 0;
	while (std::getline(indata, line))
	{
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ','))
		{
			values.push_back(std::stod(cell));
		}
		++rows;
	}
	return Eigen::Map<const Eigen::Matrix<typename Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::Scalar, Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::RowsAtCompileTime, Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
}

int main(int argc, char **argv)
{
	if (argc < 7)
	{
		throw std::runtime_error("Required arguments: \nframe_sub_master.csv frame_marker_sub.csv master_znear master_zfar sub_znear sub_zfar");
	}
	string frame_sub_master_file = argv[1];
	cout << "Reading frame_sub_master.csv" << endl;
	Transform<float, 3, Affine> transformation_sub_master;
	transformation_sub_master.matrix() = load_csv<float>(frame_sub_master_file);

	string frame_marker_sub_file = argv[2];
	cout << "Reading frame_marker_sub.csv" << endl;
	Transform<float, 3, Affine> transformation_marker_sub;
	transformation_marker_sub.matrix() = load_csv<float>(frame_marker_sub_file);

	float master_znear = std::stof(argv[3]);
	float master_zfar = std::stof(argv[4]);
	float sub_znear = std::stof(argv[5]);
	float sub_zfar = std::stof(argv[6]);

	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

	// Point Cloud
	pcl::PointCloud<PointType>::Ptr cloud_master;
	pcl::PointCloud<PointType>::Ptr cloud_sub;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex_master;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function_master =
		[&cloud_master, &mutex_master, &master_znear, &master_zfar](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	{
		boost::mutex::scoped_lock lock(mutex_master);

		/* Point Cloud Processing */

		cloud_master = ptr->makeShared();
		PassThrough<PointType> pt;
		pt.setInputCloud(cloud_master);
		pt.setFilterFieldName("z");
		pt.setFilterLimits(master_znear, master_zfar);
		pt.filter(*cloud_master);
	};

	boost::mutex mutex_sub;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function_sub =
		[&cloud_sub, &mutex_sub, &sub_znear, &sub_zfar](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	{
		boost::mutex::scoped_lock lock(mutex_sub);

		/* Point Cloud Processing */

		cloud_sub = ptr->makeShared();
		PassThrough<PointType> pt;
		pt.setInputCloud(cloud_sub);
		pt.setFilterFieldName("z");
		pt.setFilterLimits(sub_znear, sub_zfar);
		pt.filter(*cloud_sub);
	};

	// KinectAzureDKGrabber
	boost::shared_ptr<pcl::Grabber> grabber_master =
		boost::make_shared<pcl::KinectAzureDKGrabber>(0, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
	boost::shared_ptr<pcl::Grabber> grabber_sub =
		boost::make_shared<pcl::KinectAzureDKGrabber>(1, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);

	boost::shared_ptr<pcl::KinectAzureDKGrabber> grabber_ = boost::dynamic_pointer_cast<pcl::KinectAzureDKGrabber>(grabber_master);

	// Register Callback Function
	boost::signals2::connection connection_sub = grabber_sub->registerCallback(function_sub);
	boost::signals2::connection connection_master = grabber_master->registerCallback(function_master);

	// Start Grabber
	grabber_sub->start();
	grabber_master->start();

	k4a::calibration calibration = grabber_->getCalibration();
	k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
	Eigen::Matrix3f intrinsics_eigen;
	intrinsics_eigen <<
		intrinsics->param.fx, 0.0f, intrinsics->param.cx,
		0.0f, intrinsics->param.fy, intrinsics->param.cy,
		0.0f, 0.0f, 1.0f;
	Eigen::Matrix4f extrinsics_eigen = Eigen::Matrix4f::Identity();
	viewer->setCameraParameters(intrinsics_eigen, extrinsics_eigen);

	pcl::PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_transformed(new PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_axis_master(new PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_axis_sub(new PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_axis_marker(new PointCloud<PointType>);

	float aspect = 0.475f;
	
	transformation_sub_master.matrix()(0, 3) *= aspect;
	transformation_sub_master.matrix()(1, 3) *= aspect;
	transformation_sub_master.matrix()(2, 3) *= aspect;

	
	transformation_marker_sub.matrix()(0, 3) *= aspect;
	transformation_marker_sub.matrix()(1, 3) *= aspect;
	transformation_marker_sub.matrix()(2, 3) *= aspect;

	while (!viewer->wasStopped())
	{
		// Update Viewer
		viewer->spinOnce();
		boost::mutex::scoped_try_lock lock_master(mutex_master);
		boost::mutex::scoped_try_lock lock_sub(mutex_sub);

		if (lock_master.owns_lock() && cloud_master &&
			lock_sub.owns_lock() && cloud_sub)
		{
			transformPointCloud<PointType, float>(*cloud_master, *cloud_transformed, transformation_sub_master, true);

			*cloud = *cloud_sub + *cloud_transformed;

			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud"))
			{
				viewer->addPointCloud(cloud, "cloud");
			}
		}
	}

	// Stop Grabber
	grabber_sub->stop();
	grabber_master->stop();

	// Disconnect Callback Function
	if (connection_master.connected() || connection_sub.connected())
	{
		connection_sub.disconnect();
		connection_master.disconnect();
	}
	return 0;
}