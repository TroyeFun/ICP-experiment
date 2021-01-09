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
string data_dir = "./data/";

bool DEBUG = false;

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
	cout << file_path << " read: " << cloud.size() << " points in total." << endl;
}

void savePointcloud(PCD& pcd, string file_name)
{
	pcl::io::savePCDFileASCII((file_name + ".pcd").data(), pcd);
	cout << file_name + ".pcd saved." << endl;

	fstream fout;
	fout.open((file_name + ".asc").data());
	fout << "# Geomagic Studio\n# New Model\n";
	for (int i = 0; i < pcd.size(); i++) 
	{
		fout << pcd[i].x << ' ' << pcd[i].y << ' ' << pcd[i].z << endl;
	}
	fout.close();
	cout << file_name + ".asc saved." << endl;
}

void preprocessing(PCD& inputCloud, float leafsize = 1)
{
	cout << "Point cloud downsampling: before:" << inputCloud.size();
	// downsample to decrease computation
	// filter
	pcl::VoxelGrid<Point> sampler;
	sampler.setInputCloud(inputCloud.makeShared());
	// set size of voxel grid
	sampler.setLeafSize(leafsize, leafsize, leafsize);
	sampler.filter(inputCloud);
	
	cout << ", after: " << inputCloud.size() << endl;
}

void icp_iteration(
	const vector<Eigen::Vector3d>& pts1,
	const vector<Eigen::Vector3d>& pts2,
	TF& T
)
{
	Eigen::Vector3d p1=Eigen::Vector3d::Zero(), p2=Eigen::Vector3d::Zero(); // center of mass
	int N = pts1.size();
	for ( int i = 0; i < N; i++) 
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 /= N; p2 /= N;
	
	vector<Eigen::Vector3d> q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}
	
	if (DEBUG) cout << "debug 0" << endl;

	// compute q1 * q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += q1[i] * q2[i].transpose();
	}
	if (DEBUG) cout << "W=\n" << W << endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	if (DEBUG) cout << "U=\n" << U << endl;
	if (DEBUG) cout << "V=\n" << V << endl;

	Eigen::Matrix3d R = U * V.transpose();
	Eigen::Vector3d t = p1 - R * p2;

	if (DEBUG) cout << "debug 1" << endl;
	TF tmpT;
	tmpT <<
		R(0, 0), R(0, 1), R(0, 2), t(0, 0),
		R(1, 0), R(1, 1), R(1, 2), t(1, 0),
		R(2, 0), R(2, 1), R(2, 2), t(2, 0),
		0      , 0      , 0      , 1;
	T = tmpT;
	if (DEBUG) cout << "T=\n" << T << endl;
	if (DEBUG) cout << "debug 2" << endl;
}

void icp(PCD& global_pcd)
{
	PCD fixed_pcd;
	readPointcloud(fixed_pcd, data_dir+"C1.asc");
	PCD output_pcd = fixed_pcd;  // final output
	// preprocessing(fixed_pcd, 1); 	// downsample to boost NN search

	int pcd_idx = 1;
	while (true)
	{
		pcd_idx++;
		fstream fin;
		string file_path = data_dir+"C"+std::to_string(pcd_idx)+".asc";
		fin.open(file_path);
		if (!fin) break;

		cout << "Processing pcd " << pcd_idx << "..." << endl;

		PCD new_pcd;
		readPointcloud(new_pcd, file_path);
		PCD origin_new_pcd = new_pcd;
		preprocessing(new_pcd, 10);

		vector<Vector3d> closest_new_pts, closest_fix_pts;

		// Eigen::Matrix3d final_R = Eigen::Matrix3d::Identity();
		// Eigen::Vector3d final_t = Eigen::Vector3d::Zero();
		TF final_T = TF::Identity();

		// icp iterations 
		for (int k = 0; ; k++) 
		{
			double sum = 0;

			pcl:KdTreeFLANN<Point> kdtree;
			kdtree.setInputCloud(fixed_pcd.makeShared());

			if (DEBUG) cout << "debug 3" << endl;

			// find closest points
			for (int p = 0; p < new_pcd.size(); p++) 
			{
				// cout << p << endl;
				vector<int> pointIdxNKNSearch(1);
				vector<float> pointNKNSquareDistance(1);
				if (kdtree.nearestKSearch(new_pcd.points[p], 1, pointIdxNKNSearch, pointNKNSquareDistance) <= 0)
					cout << "can't find one" << endl;
				// if (pointNKNSquareDistance[0] > 0.1) continue; // threshold

				Point fp = fixed_pcd.points[pointIdxNKNSearch[0]];
				Point np = new_pcd.points[p];

				Eigen::Vector3d fv(fp.x, fp.y, fp.z);
				Eigen::Vector3d nv(np.x, np.y, np.z);

				closest_fix_pts.push_back(fv);
				closest_new_pts.push_back(nv);

				sum += (fv - nv).norm();

			}
			if (DEBUG) cout << "debug 4" << endl;

			// calculate TF
			TF T;
			icp_iteration(closest_fix_pts, closest_new_pts, T);

			if (DEBUG) cout << "debug 5" << endl;
			// update new_pcd
			pcl::transformPointCloud(new_pcd, new_pcd, T);

			if (DEBUG) cout << "debug 6" << endl;
			// update final TF
			final_T = T * final_T;

			if (k % 20 == 0) cout << "Iter: " << k << ", error: " << sum/new_pcd.size() << "\nT=\n" << T << endl;

			if ((T - TF::Identity()).maxCoeff() < 1e-6) // && sum/new_pcd.size() < 10) 
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
		if (icp_mode == FRAME2GLOBAL)
		{
			fixed_pcd = output_pcd;
			// preprocessing(fixed_pcd, 1);
		}
		
	}
	global_pcd = output_pcd;
}

int main()
{
    PCD global_pcd;
	icp(global_pcd);
	savePointcloud(global_pcd, data_dir + "output");
}

