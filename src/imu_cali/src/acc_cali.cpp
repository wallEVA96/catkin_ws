/*
 * curve_lm.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2023 sino <sino@sino>
 *
 * Distributed under terms of the MIT license.
 */

#include <Eigen/Core>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/robust_kernel_impl.h"

#include <ros/ros.h>
#include <ros/assert.h>
#include <sensor_msgs/Imu.h>
#include <thread>

using namespace std;

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class VertexCurveLm:public g2o::BaseVertex<9, Vector9d>
//class VertexCurveLm:public g2o::BaseVertex<6, Vector6d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexCurveLm() {}

	virtual void setToOriginImpl() override
	{
		_estimate << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		//_estimate << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
	}

	virtual void oplusImpl(const double *update) override
	{
		Vector9d::ConstMapType v_update(update);
	//	Vector6d::ConstMapType v_update(update);
		_estimate += v_update;
	}

	virtual bool read(istream &in) 
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	virtual bool write(ostream &out) const 
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}
};

class EdgeCurveLm:public g2o::BaseUnaryEdge<1, double, VertexCurveLm>
{
public:
	double _acc_x, _acc_y, _acc_z;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeCurveLm(double acc_x, double acc_y, double acc_z):BaseUnaryEdge(),_acc_x(acc_x), _acc_y(acc_y), _acc_z(acc_z)
	{
	
	}

	virtual bool read(istream& in)
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	virtual bool write(ostream& out) const
	{
		cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
		return false;
	}

	virtual void computeError() override
	{
		const VertexCurveLm* v = static_cast<const VertexCurveLm*>(_vertices[0]);
		const Vector9d &param_9 = v->estimate();
//		const Vector6d &param_6 = v->estimate();
//		std::cout << "param:" << param_9(0,0) << " "<< param_9(1,0) << " "<< param_9(2, 0) << " "
//							  << param_9(3, 0) << " "<< param_9(4, 0) << " "<< param_9(5, 0) << " "
//							  << param_9(6, 0) << " "<< param_9(7, 0) << " "<< param_9(8,0) << std::endl;
		
		Eigen::Vector3d correct_acc(0.0, 0.0, 0.0);
		_acc_x += param_9(6, 0);
		_acc_y += param_9(7, 0);
		_acc_z += param_9(8, 0);
//		_acc_x += param_6(3, 0);
//		_acc_y += param_6(4, 0);
//		_acc_z += param_6(5, 0);

//		correct_acc(0, 0) =  param_6(0, 0)*_acc_x ;
//		correct_acc(1, 0) =  param_6(1, 0)*_acc_y; 
//		correct_acc(2, 0) =	 param_6(2, 0)*_acc_z;

		correct_acc(0, 0) =  param_9(0, 0)*_acc_x + param_9(3, 0)*_acc_y + param_9(4, 0)*_acc_z;
		correct_acc(1, 0) =                         param_9(1, 0)*_acc_y + param_9(5, 0)*_acc_z;
		correct_acc(2, 0) =												   param_9(2, 0)*_acc_z;

//		correct_acc(0, 0) =  param_9(0, 0)*_acc_x + param_9(3, 0)*_acc_y - param_9(4, 0)*_acc_z;
//		correct_acc(1, 0) = -param_9(3, 0)*_acc_x + param_9(1, 0)*_acc_y + param_9(5, 0)*_acc_z;
//		correct_acc(2, 0) =  param_9(4, 0)*_acc_x - param_9(5, 0)*_acc_y + param_9(2, 0)*_acc_z;
		
		double correct_acc_square = correct_acc(0, 0)*correct_acc(0, 0) + correct_acc(1, 0)*correct_acc(1, 0) + correct_acc(2, 0)*correct_acc(2, 0);
		_error(0) = _measurement - correct_acc_square;
		//std::cout << "Error: " << _error(0) << std::endl;
	}

//	G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
};

/**
 * @brief imu_msg data
 *
 * @param msg
 */

std::vector<Eigen::Vector3d>  imu_vec_v3;
Eigen::Vector3d imu_msg;
void readIMUData(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_msg(0, 0) = msg->linear_acceleration.x * 10;
	imu_msg(1, 0) = msg->linear_acceleration.y * 10;
	imu_msg(2, 0) = msg->linear_acceleration.z * 10;
	if(imu_vec_v3.size() < 400)
	{
//	std::cout << "acc x:" << msg->linear_acceleration.x*10 << 
//		         "acc y:" << msg->linear_acceleration.y*10 << 
//				 "acc z:" << msg->linear_acceleration.z*10 << std::endl;
		imu_vec_v3.push_back(Eigen::Vector3d(msg->linear_acceleration.x*10, msg->linear_acceleration.y*10, msg->linear_acceleration.z*10));
	}
}

void t_s()
{
	ros::spin();
}


/**
 * @brief 
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
using namespace std;
std::vector<Eigen::Vector3d>  imu_vec_data;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "acc_cali");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("livox/imu", 1, &readIMUData);
	ros::Publisher  pub_corr_imu = nh.advertise<sensor_msgs::Imu>("imu_correct", 10);

	std::thread t_spin(&t_s);
	double w_sigma = 1;
//	cv::RNG rng;

	Vector9d last_ec_9d_est;
	uint16_t rec_imu_cnt = 0;
	while(ros::ok())
	{
		if(imu_vec_v3.size() >= 100 &&
		   imu_vec_data.size() < 1200)
		{
			typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 1>> Block;
			std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
			std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
		    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
			g2o::SparseOptimizer optimizer;
			optimizer.setAlgorithm(solver);
			// optimizer.setVerbose(true);
		
			VertexCurveLm* v = new VertexCurveLm();
			Vector9d v9_initial;
			v9_initial << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//			Vector6d v6_initial;
//			v6_initial << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
			v->setEstimate(v9_initial);
			v->setId(0);
			optimizer.addVertex(v);
//			std::cout << "Init Estimate Value:" << v->estimate().transpose() << std::endl;

			for(int i = 0; i < imu_vec_v3.size(); i++)
			{
				EdgeCurveLm *edge = new EdgeCurveLm(imu_vec_v3[i](0, 0), imu_vec_v3[i](1, 0),  imu_vec_v3[i](2, 0));
				edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
				edge->setId(i);
				edge->setVertex(0, v);
		#define GRAVITY_2 (9.8*9.8)
				edge->setMeasurement(GRAVITY_2);
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;  // Huber function (kill outlier data)
				edge->setRobustKernel(rk);
				optimizer.addEdge(edge);
			}

			optimizer.initializeOptimization();
			int curr_iter = optimizer.optimize(100);
			Vector9d vec_9d_est = v->estimate();
//			Vector6d vec_9d_est = v->estimate();
			std::cout << "Est Cal Cnt:" << rec_imu_cnt++ << " Estimate Value: " << vec_9d_est.transpose() << std::endl 
				      << "Itera:" << curr_iter << std::endl;
			
			if(curr_iter >=  2 && curr_iter < 100)
			{
				std::cout << "Add IMU data to IMU Vector ++++++" << std::endl;
				imu_vec_data.insert(imu_vec_data.end(), imu_vec_v3.begin(), imu_vec_v3.end());
				sleep(3);
			}

			imu_vec_v3.clear();
		}
		
		if(imu_vec_data.size() >= 1200 &&
		   imu_vec_v3.size() >= 100)
		{
			typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 1>> Block;
			std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
			std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
		    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
			g2o::SparseOptimizer optimizer;
			optimizer.setAlgorithm(solver);
			// optimizer.setVerbose(true);
		
			VertexCurveLm* v = new VertexCurveLm();
			Vector9d v9_initial;
			v9_initial << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//			Vector6d v6_initial;
//			v6_initial << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
			v->setEstimate(v9_initial);
			v->setId(0);
			optimizer.addVertex(v);
//			std::cout << "Init Estimate Value:" << v->estimate().transpose() << std::endl;
			std::cout << "Cal IMU Data, Size is :" << imu_vec_data.size() << std::endl;

			for(int i = 0; i < imu_vec_data.size(); i++)
			{
				EdgeCurveLm *edge = new EdgeCurveLm(imu_vec_data[i](0, 0), imu_vec_data[i](1, 0),  imu_vec_data[i](2, 0));
				edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
				edge->setId(i);
				edge->setVertex(0, v);
		#define GRAVITY_2 (9.8*9.8)
				edge->setMeasurement(GRAVITY_2);
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;  // Huber function (kill outlier data)
				edge->setRobustKernel(rk);
				optimizer.addEdge(edge);
			}

			optimizer.initializeOptimization();
			int curr_iter = optimizer.optimize(100);
			last_ec_9d_est = v->estimate();
//			Vector6d vec_9d_est = v->estimate();
			std::cout << "Last Estimate Value: " << last_ec_9d_est.transpose() << std::endl 
				      << "Itera:" << curr_iter << std::endl;
			break;
		}
	}
	
	std::cout << "Publish Correct IMU Data;" << std::endl;
	sensor_msgs::Imu imu_pub;
	while(ros::ok())
	{
		double _acc_x =  imu_msg(0, 0), _acc_y = imu_msg(1, 0), _acc_z = imu_msg(2, 0);
		const Vector9d &param_9 = last_ec_9d_est;
		Eigen::Vector3d correct_acc(0.0, 0.0, 0.0);
		
		_acc_x += param_9(6, 0);
		_acc_y += param_9(7, 0);
		_acc_z += param_9(8, 0);
		correct_acc(0, 0) =  param_9(0, 0)*_acc_x + param_9(3, 0)*_acc_y + param_9(4, 0)*_acc_z;
		correct_acc(1, 0) =                         param_9(1, 0)*_acc_y + param_9(5, 0)*_acc_z;
		correct_acc(2, 0) =												   param_9(2, 0)*_acc_z;

		imu_pub.header.frame_id = "imu_correct";
		imu_pub.linear_acceleration.x = correct_acc(0.0) /10.0;
		imu_pub.linear_acceleration.y = correct_acc(1.0) /10.0;
		imu_pub.linear_acceleration.z = correct_acc(2.0) /10.0;
		pub_corr_imu.publish(imu_pub);	
		ros::spinOnce();
	}
return 0;
}

