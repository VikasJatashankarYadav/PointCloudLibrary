#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace std;

int
main()
{
	// Input .xyz to .pcd
	cout << "Poitn 1" << endl;
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
		p.r = stof(result[3]);
		p.g = stof(result[4]);
		p.b = stof(result[5]);
		cloudxyz.points.push_back(p);
		i++;
	}
	cloudxyz.width = cloudxyz.points.size();
	cloudxyz.height = 1;
	cloudxyz.points.resize(cloudxyz.width * cloudxyz.height);
	cloudxyz.is_dense = false;
	ipfile.close();
	pcl::io::savePCDFileASCII("data1.pcd", cloudxyz);
	cout << "Poitn 2" << endl;

	



	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile<pcl::PointXYZ>("data1.pcd", *cloud);
	cout << "Poitn 3" << endl;
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	
	/*
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
	*/
	cout << "Poitn 4" << endl;
	string opstring = "";

	ofstream output_file("data2.xyz");
	opstring = "";
	i = 0;
	for (auto& point_save : *cloud_filtered) {
		opstring = to_string(point_save.x) + " " + to_string(point_save.y) + " " + to_string(point_save.z);
		output_file << opstring << endl;
		i++;
	}
	output_file.close();


	return (0);

}
