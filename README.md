K4aGrabber
==============

K4aGrabber is a Grabber of PCL (Point Cloud Library) to retrieve the point cloud data from Kinect Azure DK using Kinect Azure DK v1.1.1.  
This Grabber only depends on Kinect Azure DK v 1.1.1.  

K4aGrabber supports the following pcl::PointType.  
* pcl::PointXYZ
* pcl::PointXYZI
* pcl::PointXYZRGB
* pcl::PointXYZRGBA

Inspired By *[Kinect2Grabber](https://github.com/UnaNancyOwen/KinectGrabber)* .

## Projects

### OneKinect

PCL Azure Kinect Grabber for one Kinect.

### TwoKinects

PCL Azure Kinect Grabber for 2 Kinects. 

Using ArUco Library to calibrate the relative pose between master Kinect and sub Kinect.

We just add two point cloud generated from two Kinects into one point cloud, and then we get a real-time point cloud of the scene.



Environment
-----------
* Point Cloud Library v1.9.1
* Kinect v1.1.1
* Visual Studio Community 2017 

License
-------
Copyright &copy; 2019 Haipeng WANG  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php "MIT License | Open Source Initiative").  


Contact
-------
* Haipeng Wang
    * E-mail: <forestsen@vip.qq.com>


References
----------
* Kinect2Grabber

  <https://github.com/UnaNancyOwen/KinectGrabber>

* openni_grabber | Point Cloud Library  
  <https://github.com/PointCloudLibrary>

* NaturalSoftware.Kinect.PCL | Natural Software  
  <https://github.com/NaturalSoftwareJP/NaturalSoftware.Kinect.Cpp>
