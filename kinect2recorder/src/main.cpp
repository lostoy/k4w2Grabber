#include <Kinect2Grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <queue>
#include <conio.h>

class KinectRecorder
{

public:
	KinectRecorder(int limitNum, std::string path, std::string device) :limitNum_(limitNum), path_(path), frameid_(0), running_(false), pause_(false)
	{
		if (device == "kinect1")
			grabber_ = new pcl::OpenNIGrabber();
		else
			grabber_ = new Kinect2Grabber;
		
		//register calback function
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&KinectRecorder::addcloud2que, this, _1);
		grabber_->registerCallback(f);

	}
	~KinectRecorder()
	{
		stop();
		saveThread_.join();
		std::cout << "ok! press anykey to close...\n";
		keyboardThread_.join();
		
	}
	void start()
	{
		grabber_->start();
		running_ = true;
		saveThread_ = std::thread(&KinectRecorder::saveCloud,this);
		keyboardThread_ = std::thread(&KinectRecorder::keyboardCallback, this);
		while (running_&&(limitNum_ <= 0 || frameid_ < limitNum_));
		stop();
		

	}
	void stop()
	{
		if (running_)
		{
			grabber_->stop();
			running_ = false;

			
		}
	}

private:
	//global queque and save information

	pcl::Grabber *grabber_;
	std::thread saveThread_,keyboardThread_;

	volatile int frameid_ ,limitNum_;
	std::queue<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, int>> cloud_que_;
	bool running_ ,pause_;
	std::string path_;


	std::string int2string(int n)
	{
		std::stringstream ss;
		ss << n;
		return ss.str();
	}

	//kinect callback, add new xyzrgba cloud to saving queue
	void addcloud2que(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if (!pause_)
		{

			frameid_++;

			cloud_que_.push(std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, int>(cloud, frameid_));
			std::cout << "read:" << frameid_ << std::endl;
		}
	}

	//save cloud thread, save all remaining cloud in the queue
	void saveCloud()
	{
		while (running_)
		{

			while (!cloud_que_.empty())
			{
				std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, int> first_cloud;
				first_cloud = cloud_que_.front();

				std::cout << "save: " << first_cloud.second << std::endl;
				pcl::io::savePCDFileBinary(path_ + "/" + int2string(first_cloud.second) + ".pcd", *first_cloud.first);
				cloud_que_.pop();
			}
			
		}
	}

	//keyboard thread
	void keyboardCallback()
	{
		while (running_)
		{
			char ch = getch();
			if (ch == 'q')
				stop();
			if (ch == 'p')
				pause_ = !pause_;
		}
	}
};
//argv[1] the maximum frame #, infinite loop if negative 
//argv[2] the path the cloud will be saved to
//argv[3]=="kinect1" if the device is kinect1 
//when recording data, press 'p' to toggle pause, 'q' to quit
int main(int argc,char *argv[])
{
	//extract command line arguments
	int limitNum = atoi(argv[1]);
	std::cout << "max num: " << limitNum << std::endl;
	
	
	KinectRecorder krecorder(limitNum, std::string(argv[2]), std::string(argv[3]));
	
	krecorder.start();

	return 0;
	

}