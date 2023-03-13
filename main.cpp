#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include "include/patchworkpp.h"

void patchworkFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
					 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out_ground,
					 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out_nonground,
					 bool verbose
					 )
{
	// Convert pcl point cloud to Eigen::MatrixXf
	Eigen::MatrixXf cloud;
	cloud.resize(cloud_in->size(), 5);
	for (int i = 0; i < cloud_in->size(); ++i)
	{
		cloud.row(i) << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, cloud_in->points[i].intensity, float(i);
	}
	
	// Initialize patchwork 
	patchwork::Params patchwork_parameters;
	patchwork_parameters.verbose = verbose;
	patchwork::PatchWorkpp Patchworkpp(patchwork_parameters);

	// Estimate Ground
	Patchworkpp.estimateGround(cloud);
	
	std::vector<int> groundIndices = Patchworkpp.getGroundIndices();
	std::vector<int> NongroundIndices = Patchworkpp.getNongroundIndices();

	// Print Info
	if (verbose)
	{
        double time_taken = Patchworkpp.getTimeTaken();
		std::cout << "Origianl Points  #: " << cloud_in->size() << std::endl;
		std::cout << "Ground Points    #: " << groundIndices.size() << std::endl;
		std::cout << "Nonground Points #: " << NongroundIndices.size() << std::endl;
		std::cout << "Time Taken : "<< time_taken / 1000000 << "(sec)" << std::endl;
	}

	pcl::copyPointCloud<pcl::PointXYZI>(*cloud_in, groundIndices, *cloud_out_ground);
	pcl::copyPointCloud<pcl::PointXYZI>(*cloud_in, NongroundIndices, *cloud_out_nonground);

}

int main(int argc, char *argv[])
{
    YAML::Node config = YAML::LoadFile("../config.yaml");
    const std::string pcd_path = config["config"]["pcd_path"].as<std::string>();
    const bool verbose = config["config"]["verbose"].as<bool>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ground (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_nonground (new pcl::PointCloud<pcl::PointXYZI>());

	if (pcl::io::loadPCDFile(pcd_path, *cloud_in) < 0)
    {
        PCL_ERROR("This dir doesnot exit %s pcd file.\n", pcd_path);
        return -1;
    }

    patchworkFilter(cloud_in, cloud_out_ground, cloud_out_nonground, verbose);

    pcl::io::savePCDFileASCII<pcl::PointXYZI>("../cloud_in.pcd", *cloud_in);
    pcl::io::savePCDFileASCII<pcl::PointXYZI>("../cloud_out_ground.pcd", *cloud_out_ground);
    pcl::io::savePCDFileASCII<pcl::PointXYZI>("../cloud_out_nonground.pcd", *cloud_out_nonground);

    return 0;

}

