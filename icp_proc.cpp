/*************************************************************************
	> File Name: icp_proc.cpp
	> Author: Fang Hongyu
	> Created Time: 五  1/ 8 21:10:27 2021
 ************************************************************************/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/geometry.h>

#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <cmath>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace std;

/////////////////////////////////////

bool U;
typedef pcl::PointXYZ Point;
typedef Eigen::Matrix4d TF;
typedef Eigen::Isometry3d tf;
typedef pcl::PointCloud<Point> PCD;

#define FRAME2FRAME 0
#define FRAME2GLOBAL 1
int icp_mode = FRAME2FRAME;
string data_dir = "./data/"

void readPointcloud(PCD& cloud, string file_path)
{
	fstream fin;
	fin.open(file_path.data());
	string tmp;
	getline(fin, tmp); getline(fin, tmp);
	float x, y, z;
	while (fin >> x >> y >> z)
	{
		Point p;
		p.x = x; p.y = y; p.z = z;
		cloud.push_back(p);
	}
	return cloud;
}

void savePointcloud(PCD& pcd, string file_name)
{
	pcl::io::savePCDFileASCII((file_name + ".pcd").data(), pcd);
	fstream fout;
	fout.open((file_name + ".asc").data());
	fout << "# Geomagic Studio\n# New Model\n";
	for (int i = 0; i < pcd.size(); i++) 
	{
		fout << pcd[i].x << ' ' << pcd[i].y << ' ' << pcd[i].z << endl;
	}
	fout.close();
}

void preprocessing(PCD::Ptr inputCloud)
{
	cout << "Size of point cloud before downsampling: " << inputCloud->size() << endl;
	// downsample to decrease computation
	// filter
	pcl::VoxelGrid<Point> sampler;
	sampler.setInputCloud(inputCloud);
	// set size of voxel grid
	sampler.setLeafSize(0.001f, 0.001f, 0.001f);
	sampler.filter(*inputCloud);
	
	cout << "Size of point cloud after downsampling: " << inputCloud->size() << endl;
}

void icp_iteration(
	const vector<Point3f>& pts1,
	const vector<Point3f>& pts2,
	TF& T,
)
{
	Point3f p1, p2; // center of mass
	int N = pts1.size();
	for ( int i = 0; i < N; i++) 
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 /= N; p2 /= N;
	vector<Point3f> q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1 * q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d( q1[i].x, a1[i].y, q1[i].z) * Eigen:Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	cout << "W=" << W << endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	cout << "U=" << U << endl;
	cout << "V=" << V << endl;

	R = U*(V.transpose());
	t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);

	T = ( TF() <<
		R(0, 0), R(0, 1), R(0, 2), t(0, 0),
		R(1, 0), R(1, 1), R(1, 2), t(1, 0),
		R(2, 0), R(2, 1), R(2, 2), t(2, 0),
		0      , 0      , 0      , 1
	);
}

TF get_tf(Eigen::Matrix3d R, Eigen::Vector3d t)
{
	TF T = ( TF() <<
		R(0, 0), R(0, 1), R(0, 2), t(0, 0),
		R(1, 0), R(1, 1), R(1, 2), t(1, 0),
		R(2, 0), R(2, 1), R(2, 2), t(2, 0),
		0      , 0      , 0      , 1
	);
	return T;
}

void icp(PCD& global_pcd)
{
	PCD fixed_pcd;
	readPointcloud(fixed_pcd, data_dir+"C1.asc");
	PCD output_pcd = fixed_pcd;  // final output
	preprocessing(fixed_pcd.make_shared()); 	// downsample to boost NN search

	int pcd_idx = 1;
	while (true)
	{
		pcd_idx++;
		fstream fin;
		string file_path = data_dir+"C"+std::to_string(pcd_idx)+".asc"
		fin.open(file_path);
		if (!fin) break;
		cout << "Processing pcd " << pcd_idx << endl;

		PCD new_pcd;
		readPointcloud(new_pcd, file_path);
		PCD origin_new_pcd = new_pcd;
		preprocessing(new_pcd.make_shared());

		vector<Vector3d> closest_new_pts, closest_fix_pts;

		// Eigen::Matrix3d final_R = Eigen::Matrix3d::Identity();
		// Eigen::Vector3d final_t = Eigen::Vector3d::Zero();
		TF final_T = TF::Identity();

		// icp iterations 
		for (int k = 0; ; k++) 
		{
			double sum = 0;
			pcl:KdTreeFLANN<Point> kdtree;
			kdtree.setInputCloud(fixed_pcd.make_shared());

			// find closest points
			for (int p = 0; p < new_pcd.size(); p++) 
			{
				vector<int> pointIdxNKNSearch(1);
				vector<float> pointNKNSquareDistance(1);
				if (kdtree.nearestKSearch(new_pcd.points[p], 1, pointIdxNKNSearch, pointNKNSquareDistance) <= 0)
					cout << "can't find one" << endl;
				if (pointNKNSquareDistance[0] > 0.1) continue // threshold

				Point f = fixed_pcd.points[pointIdxNKNSearch[0]];
				Point n = new_pcd.points[p];
				closest_fix_pts.push_back(Vector3d(f.x, f.y, f.z));
				closest_new_pts.push_back(Vector3d(n.x, n.y, n.z));

				sum += (closest_fix_pts[p] - closest_new_pts[p]).norm();
			}
			if (k % 20 == 0) cout << "Iter: " << k << ", error: " << sum << endl;

			// calculate TF
			TF T;
			icp_iteration(fixed_pcd, new_pcd, T);

			// update new_pcd
			pcl::transformPointCloud(new_pcd, new_pcd, T);

			// update final TF
			final_T = T * final_T;

			if ((T - TF::Identity()).maxCoeff() < 1e-6 && sum < 1) 
			{
				cout << "pcd " << pcd_idx << ": converged after " << k << " iterations." << endl;
				break;
			} 				

			closest_new_pts.clear();
			closest_fix_pts.clear();
		}
		cout << "TF: " << endl << final_T << endl;

		// transform pcd and save
		pcl::transformPointCloud(origin_new_pcd, origin_new_pcd, final_T);
		savePointcloud(origin_new_pcd, data_dir+"C"+std::to_string(pcd_idx)+"_new");

		output_pcd += origin_new_pcd;

		// update fixed_pcd
		if (icp_mode = FRAME2GLOBAL)
		{
			fixed_pcd = output_pcd;
			preprocessing(fixed_pcd.make_shared());
		}
		
	}
	global_pcd = output_pcd;
}

int main()
{
    PCD global_pcd;
	icp(global_pcd);
	savePointcloud(global_pcd, data_dir + "output")
}

