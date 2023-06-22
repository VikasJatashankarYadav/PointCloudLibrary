#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <pcl/conversions.h>

using namespace std;

int
main()
{

	// Input .xyz to .pcd
	vector<string> result;
	pcl::PointCloud<pcl::PointXYZRGB> cloudxyz;
	pcl::PointXYZRGB p;
	string filename = "data1.xyz";
	// Read XYZ file i.e. one frame
	ifstream ipfile(filename);
	vector<vector<float>> datapoints;
	if (!(ipfile.is_open())) {
		cout << "[x] Cannot open file!" << endl;
	}
	string content = "";
	int i = 0;
	while (getline(ipfile, content))
	{
		boost::split(result, content, [](char c) {return c == ' '; });
		p.x = stof(result[0]);
		p.y = stof(result[1]);
		p.z = stof(result[2]);
		cloudxyz.points.push_back(p);
		i++;
	}
	cloudxyz.width = cloudxyz.points.size();
	cloudxyz.height = 1;
	cloudxyz.points.resize(cloudxyz.width * cloudxyz.height);
	cloudxyz.is_dense = false;
	ipfile.close();
	pcl::io::savePCDFileASCII("data1.pcd", cloudxyz);
	


	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("data1.pcd", *cloud);
	pcl::VoxelGrid<pcl::PointXYZ> sor1;
	sor1.setInputCloud(cloud);
	sor1.setLeafSize(0.05f, 0.05f, 0.05f);
	sor1.filter(*filtered_cloud);

	ofstream output_file1("data2.xyz");
	string opstring = "";
	i = 0;
	for (auto& p_save : *filtered_cloud) {
		opstring = to_string(p_save.x) + " " + to_string(p_save.y) + " " + to_string(p_save.z);
		output_file1 << opstring << endl;
		i++;
	}
	output_file1.close();

	return (0);
}