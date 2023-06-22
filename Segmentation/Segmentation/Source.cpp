#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <unordered_map>

using namespace std;

int reverse_sort(pair<int, int> a, pair<int, int> b) {
	return a.second > b.second;
}


int
main(int argc, char** argv)
{
	vector<string> result;
	//pcl::PointCloud<pcl::PointXYZRGB> cloudxyz;
	pcl::PointCloud<pcl::PointXYZ> cloudxyz;
	//pcl::PointXYZRGB p;
	pcl::PointXYZ p;
	string filename = "data1.xyz";
	// Read XYZ file i.e. one frame
	ifstream ipfile(filename);
	vector<vector<float>> datapoints;
	if (!(ipfile.is_open())) {
		std::cout << "[x] Cannot open file!" << endl;
	}
	string content = "";
	int i = 0;
	while (getline(ipfile, content))
	{
		vector<float> data;
		boost::split(result, content, [](char c) {return c == ' '; });
		p.x = stof(result[0]);
		p.y = stof(result[1]);
		p.z = stof(result[2]);
		data.push_back(stof(result[0]));
		data.push_back(stof(result[1]));
		data.push_back(stof(result[2]));
		/*p.r = stof(result[3]);
		p.g = stof(result[4]);
		p.b = stof(result[5]);*/
		cloudxyz.points.push_back(p);
		datapoints.push_back(data);
		i++;
	}
	cloudxyz.width = cloudxyz.points.size();
	cloudxyz.height = 1;
	cloudxyz.points.resize(cloudxyz.width * cloudxyz.height);
	cloudxyz.is_dense = false;
	ipfile.close();
	pcl::io::savePCDFileASCII("data1.pcd", cloudxyz);
	std::cout << "Poitn 2" << cloudxyz.width << endl;





	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("data1.pcd", *cloud);
	std::cout << "Poitn 3" << endl;
  
   

   /* pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;

   
    //for (std::size_t i = 0; i < inliers->indices.size(); ++i)
	string opstring = "";

	ofstream output_file("data2.xyz");
	opstring = "";
	i = 0;
	for (const auto& idx : inliers->indices) {
		opstring = to_string(cloud->points[idx].x) + " " +
			       to_string(cloud->points[idx].y) + " " +
				   to_string(cloud->points[idx].z) ;
		output_file << opstring << endl;
	}
	output_file.close();
	*/

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);



	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);


	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);


	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	/*while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}*/
	std::cout << std::endl;
	std::cout << clusters.size();
	counter = 0;
	string opstring = "";
	unordered_map<int, int> umap;

	while (counter < clusters.size()) {
		/*
		opstring = "";
		ofstream output_file("data_op_"+to_string(counter)+".xyz");
		for (const auto& idx : clusters[counter].indices) {
			opstring = to_string(cloud->points[idx].x) + " " +
				to_string(cloud->points[idx].y) + " " +
				to_string(cloud->points[idx].z);
			output_file << opstring << endl;
		}
		output_file.close();
		*/
		umap[counter] = clusters[counter].indices.size();
		// std::cout << clusters[counter].indices.size() << endl;
		counter++;
	}


	vector <pair<int, int>> sorted_pair(umap.begin(), umap.end());
	
	std::sort(sorted_pair.begin(), sorted_pair.end(), reverse_sort);
	int wall_itr = 0;
	int object_itr = 0;
	pcl::PointCloud<pcl::PointXYZ> cloud_walls, cloud_objects;
	std::cout << 4 << endl;
	cloud_walls.height = cloud_objects.height = 1;
	cloud_walls.width = cloud_objects.width = cloudxyz.width;
	cloud_walls.resize(cloud_walls.width * cloud_walls.height);
	cloud_objects.resize(cloud_objects.width * cloud_objects.height);
	vector<vector<float>> datapoints_walls;
	for (int i = 0; i < sorted_pair.size(); i++) {
		std::cout << 4 << endl;
		float xmin = INT_MAX, ymin = INT_MAX, zmin = INT_MAX, xmax = INT_MIN, ymax = INT_MIN, zmax = INT_MIN;
		opstring = "";
		// ofstream output_file("data_op_" + to_string(sorted_pair[i].first) + ".xyz");
		std::cout << 5 << endl;
		for (const auto& idx : clusters[sorted_pair[i].first].indices) {
			/*opstring = to_string(cloud->points[idx].x) + " " +
				to_string(cloud->points[idx].y) + " " +
				to_string(cloud->points[idx].z);
			output_file << opstring << endl;*/
			if (cloud->points[idx].x < xmin)
				xmin = cloud->points[idx].x;
			if (cloud->points[idx].x > xmax)
				xmax = cloud->points[idx].x;
			if (cloud->points[idx].y < ymin)
				ymin = cloud->points[idx].y;
			if (cloud->points[idx].y > ymax)
				ymax = cloud->points[idx].y;
			if (cloud->points[idx].z < zmin)
				zmin = cloud->points[idx].z;
			if (cloud->points[idx].x > xmax)
				zmax = cloud->points[idx].z;
			std::cout << 6 << endl;
		}
		std::cout << 7 << endl;
		if ( (sorted_pair[i].second > (0.05 * cloudxyz.width) ) && ( (xmax - xmin) < 0.5 || (ymax - ymin) < 0.5 || (zmax - zmin) < 0.5) ) {
			std::cout << 7.1 << endl;
			// cloud_walls.width = cloud_walls.width + sorted_pair[i].second;
			std::cout << 7.2 << endl;
			for (const auto& idx : clusters[sorted_pair[i].first].indices) {
				vector<float> data;
				cloud_walls[wall_itr].x = cloud->points[idx].x;
				cloud_walls[wall_itr].y = cloud->points[idx].y;
				cloud_walls[wall_itr].z = cloud->points[idx].z;
				data.push_back(cloud->points[idx].x);
				data.push_back(cloud->points[idx].y);
				data.push_back(cloud->points[idx].z);
				datapoints_walls.push_back(data);
				wall_itr++;
				std::cout << 7.3 << endl;
			}
			std::cout << 7.4 << endl;
		}
		else {
			std::cout << 8 << endl;
			// cloud_objects.width = cloud_objects.width + sorted_pair[i].second;
			std::cout << 8.1 << endl;
			for (const auto& idx : clusters[sorted_pair[i].first].indices) {
				cloud_objects[object_itr].x = cloud->points[idx].x;
				cloud_objects[object_itr].y = cloud->points[idx].y;
				cloud_objects[object_itr].z = cloud->points[idx].z;

				object_itr++;
				std::cout << 8.2 << endl;
			}
			std::cout << 8.3 << endl;
		}
		/*std::cout << sorted_pair[i].first << " " << sorted_pair[i].second << endl;
		std::cout << xmax << " " << xmin << " " << xmax - xmin << endl;
		std::cout << ymax << " " << ymin << " " << ymax - ymin << endl;
		std::cout << zmax << " " << zmin << " " << zmax - zmin << endl<<endl<<endl;
		output_file.close();*/
	}
	/*for (const auto& idx : clusters[0].indices)
		std::cerr << idx << "    " << cloud->points[idx].x << " "
		<< cloud->points[idx].y << " "
		<< cloud->points[idx].z << std::endl;*/
	std::cout << 8.4 << endl;
	ofstream output_file1("walls.xyz");
	opstring = "";
	for (auto& p_save : cloud_walls) {
		opstring = to_string(p_save.x) + " " + to_string(p_save.y) + " " + to_string(p_save.z);
		output_file1 << opstring << endl;
	}
	output_file1.close();
	

	ofstream output_file2("objects.xyz");
	opstring = "";
	for (auto& p_save : cloud_objects) {
		opstring = to_string(p_save.x) + " " + to_string(p_save.y) + " " + to_string(p_save.z);
		output_file2 << opstring << endl;
	}
	output_file2.close();

    return (0);
}