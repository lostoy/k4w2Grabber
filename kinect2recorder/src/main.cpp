#include <Kinect2Grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <queue>

//global queque and save information
volatile int frameid = 0;
std::queue<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr,int>> cloud_que;
bool running = false;
std::string path;


std::string int2string(int n)
{
	std::stringstream ss;
	ss << n;
	return ss.str();
}

//kinect callback, add new xyzrgba cloud to saving queue
void addcloud2que(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	frameid++;
	
	cloud_que.push(std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, int>(cloud,frameid));
	std::cout << frameid << std::endl;
}

//save cloud thread, save all remaining cloud in the queue
void saveCloud()
{
	while (running)
	{

		while (!cloud_que.empty())
		{
			std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, int> first_cloud;
			first_cloud = cloud_que.front();

			std::cout << first_cloud.second << std::endl;
			pcl::io::savePCDFileBinary(path+"/"+int2string(first_cloud.second) + ".pcd", *first_cloud.first);
			cloud_que.pop();
		}
	}
}

//argv[1] the maximum frame #, infinite loop if negative 
//argv[2] the path the cloud will be saved to
int main(int argc,char *argv[])
{
	//extract command line arguments
	int limitFrame = atoi(argv[1]);
	std::cout << "max num: " << limitFrame << std::endl;
	path = argv[2];

	//Kinect2Grabber grabber;
	pcl::Grabber *grabber;
	if (std::string(argv[3]) == "kinect1")
		grabber = new pcl::OpenNIGrabber();
	else
		grabber = new Kinect2Grabber;

	//register calback function
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind(&addcloud2que,_1);
	grabber->registerCallback(f);

	grabber->start();
	running = true;
	std::thread savethread(saveCloud);
	
	while (limitFrame<=0||frameid < limitFrame);
	grabber->stop();

	running = false;
	savethread.join();
	std::cout << "ok!\n";

}