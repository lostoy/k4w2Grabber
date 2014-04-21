#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <iostream>

int cur_frameid = 0;
pcl::PointCloud<pcl::PointXYZRGBA> cur_cloud;
std::vector<std::string> filenames;
pcl::visualization::CloudViewer viewer("3dplayer");
bool new_frame = false;
int getFrameIDFromPath(std::string path)
{
	boost::filesystem::path s(path);
	std::string filename = s.filename().string();


	int se = filename.find_last_of(".");
	return (atoi(filename.substr(0, se).c_str()));
}


bool isFilenameSmaller(std::string s1, std::string s2)
{
	int i1 = getFrameIDFromPath(s1);
	int i2 = getFrameIDFromPath(s2);
	return i1<i2;
}

std::vector<std::string> getFilenames(std::string dirname, std::string ext)
{
	boost::filesystem::path dir(dirname);
	std::vector<std::string> Files;
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; pos++)
	if (boost::filesystem::is_regular_file(pos->status()))
	if (boost::filesystem::extension(*pos) == ext)
	{
		Files.push_back(pos->path().string());
	}


	std::sort(Files.begin(), Files.end(), isFilenameSmaller);
	return Files;
}
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void*)
{

	if (event.keyDown())
	{
		new_frame = true;
	}
}

//a 3dplayer, press anykey to load the next frame.
int main(int argc, char *argv[])
{


	
	//register keyboard call-back
	viewer.registerKeyboardCallback(keyboardEventOccurred);
	//get filepaths from argv[1]
	
	filenames = getFilenames(argv[1], ".pcd");
	pcl::io::loadPCDFile(filenames[cur_frameid++], cur_cloud);

	viewer.showCloud(cur_cloud.makeShared(), "cloud");

	while (!viewer.wasStopped()&&cur_frameid+1<filenames.size())
	{
		if (new_frame)
		{
			pcl::io::loadPCDFile(filenames[cur_frameid++], cur_cloud);

			viewer.showCloud(cur_cloud.makeShared(), "cloud");
			std::cout << cur_frameid << std::endl;
			new_frame = false;
		}
		//boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	return 0;

}