#pragma once

#include <string>
#include <memory>
#include <stdexcept>
#include <algorithm>

#include <Eigen/Core>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>

class ComputedTorqueController
{
public:
	enum class ModelFormat
	{
	AUTO = 0,
	URDF,
	MJCF
	};

public:
	ComputedTorqueController();
	~ComputedTorqueController() = default;

	// 加载模型：支持 URDF / MJCF
	bool loadModel(const std::string &model_file,
					ModelFormat format = ModelFormat::AUTO,
					bool verbose = false);

	// 设置 PID 型加速度反馈增益（关节空间）
	void setGains(const Eigen::VectorXd &kp,
				const Eigen::VectorXd &kd,
				const Eigen::VectorXd &ki = Eigen::VectorXd());

	// 设置力矩限制；若为空则不限制
	void setTorqueLimits(const Eigen::VectorXd &tau_limit_abs);

	// 设置积分误差限制；若为空则不限制
	void setIntegralLimits(const Eigen::VectorXd &int_limit_abs);

	// 清空积分项
	void reset();

	// 计算闭环计算力矩
	// q, dq, q_des, dq_des, ddq_des 维数都应为 n
	Eigen::VectorXd computeTorque(const Eigen::VectorXd &q,
								const Eigen::VectorXd &dq,
								const Eigen::VectorXd &q_des,
								const Eigen::VectorXd &dq_des,
								const Eigen::VectorXd &ddq_des,
								double dt);

	// 仅计算模型前馈逆动力学 tau_ff = M*ddq_des + h
	Eigen::VectorXd computeFeedforwardTorque(const Eigen::VectorXd &q,
											const Eigen::VectorXd &dq,
											const Eigen::VectorXd &ddq_des);

	// 获取维度
	int nq() const;
	int nv() const;
	bool loaded() const;

	const pinocchio::Model &model() const;
	const pinocchio::Data &data() const;

private:
	void checkLoaded() const;
	void checkVectorSize(const Eigen::VectorXd &x, int n, const std::string &name) const;
	void allocateInternalBuffers();
	ModelFormat inferFormatFromFile(const std::string &model_file) const;
	Eigen::VectorXd saturateAbs(const Eigen::VectorXd &x,
								const Eigen::VectorXd &limit_abs) const;

private:
	bool loaded_ = false;
	ModelFormat format_ = ModelFormat::AUTO;

	pinocchio::Model model_;
	std::unique_ptr<pinocchio::Data> data_;

	int n_ = 0;  // 对固定基机械臂，默认 nq == nv == n

	Eigen::VectorXd kp_;
	Eigen::VectorXd kd_;
	Eigen::VectorXd ki_;

	Eigen::VectorXd e_int_;

	Eigen::VectorXd tau_limit_abs_;
	Eigen::VectorXd int_limit_abs_;

	bool use_integral_ = false;
	bool use_tau_limit_ = false;
	bool use_int_limit_ = false;
};