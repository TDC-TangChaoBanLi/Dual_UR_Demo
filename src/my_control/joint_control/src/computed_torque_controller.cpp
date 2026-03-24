#include "joint_control/computed_torque_controller.hpp"

#include <iostream>
#include <cctype>

ComputedTorqueController::ComputedTorqueController()
{
}

bool ComputedTorqueController::loadModel(const std::string &model_file,
										ModelFormat format,
										bool verbose)
{
	if (model_file.empty())
	{
		throw std::runtime_error("model_file is empty.");
	}

	format_ = (format == ModelFormat::AUTO) ? inferFormatFromFile(model_file) : format;

	try
	{
		model_ = pinocchio::Model();

		if (format_ == ModelFormat::URDF)
		{
			// 固定基模型
			pinocchio::urdf::buildModel(model_file, model_, verbose);
		}
		else if (format_ == ModelFormat::MJCF)
		{
			// 固定基 MJCF
			pinocchio::mjcf::buildModel(model_file, model_, verbose);
		}
		else
		{
			throw std::runtime_error("Unsupported model format.");
		}

		data_ = std::make_unique<pinocchio::Data>(model_);

		if (model_.nq != model_.nv)
		{
			throw std::runtime_error(
					"This controller currently assumes a fixed-base arm with nq == nv. "
					"The loaded model does not satisfy this.");
		}

		n_ = static_cast<int>(model_.nv);
		allocateInternalBuffers();
		loaded_ = true;
		return true;
	}
	catch (const std::exception &e)
	{
		loaded_ = false;
		data_.reset();
		throw std::runtime_error(std::string("Failed to load model: ") + e.what());
	}
}

void ComputedTorqueController::setGains(const Eigen::VectorXd &kp,
										const Eigen::VectorXd &kd,
										const Eigen::VectorXd &ki)
{
	checkLoaded();
	checkVectorSize(kp, n_, "kp");
	checkVectorSize(kd, n_, "kd");

	kp_ = kp;
	kd_ = kd;

	if (ki.size() == 0)
	{
		ki_ = Eigen::VectorXd::Zero(n_);
		use_integral_ = false;
	}
	else
	{
		checkVectorSize(ki, n_, "ki");
		ki_ = ki;
		use_integral_ = true;
	}
}

void ComputedTorqueController::setTorqueLimits(const Eigen::VectorXd &tau_limit_abs)
{
	checkLoaded();
	checkVectorSize(tau_limit_abs, n_, "tau_limit_abs");
	tau_limit_abs_ = tau_limit_abs.cwiseAbs();
	use_tau_limit_ = true;
}

void ComputedTorqueController::setIntegralLimits(const Eigen::VectorXd &int_limit_abs)
{
	checkLoaded();
	checkVectorSize(int_limit_abs, n_, "int_limit_abs");
	int_limit_abs_ = int_limit_abs.cwiseAbs();
	use_int_limit_ = true;
}

void ComputedTorqueController::reset()
{
	if (n_ > 0)
		e_int_.setZero();
}

Eigen::VectorXd ComputedTorqueController::computeFeedforwardTorque(
		const Eigen::VectorXd &q,
		const Eigen::VectorXd &dq,
		const Eigen::VectorXd &ddq_des)
{
	checkLoaded();
	checkVectorSize(q, n_, "q");
	checkVectorSize(dq, n_, "dq");
	checkVectorSize(ddq_des, n_, "ddq_des");

	// RNEA 直接给出 tau = M(q)ddq + h(q,dq)
	Eigen::VectorXd tau_ff = pinocchio::rnea(model_, *data_, q, dq, ddq_des);
	return tau_ff;
}

Eigen::VectorXd ComputedTorqueController::computeTorque(
		const Eigen::VectorXd &q,
		const Eigen::VectorXd &dq,
		const Eigen::VectorXd &q_des,
		const Eigen::VectorXd &dq_des,
		const Eigen::VectorXd &ddq_des,
		double dt)
{
	checkLoaded();
	checkVectorSize(q, n_, "q");
	checkVectorSize(dq, n_, "dq");
	checkVectorSize(q_des, n_, "q_des");
	checkVectorSize(dq_des, n_, "dq_des");
	checkVectorSize(ddq_des, n_, "ddq_des");

	if (dt <= 0.0)
	{
		throw std::runtime_error("dt must be > 0.");
	}

	// 对于普通固定基串联机械臂，可直接做关节差
	// 若包含球关节/浮基，建议改成 pinocchio::difference(model_, q, q_des)
	Eigen::VectorXd e = q_des - q;
	Eigen::VectorXd de = dq_des - dq;

	if (use_integral_)
	{
		e_int_ += e * dt;
		if (use_int_limit_)
			e_int_ = saturateAbs(e_int_, int_limit_abs_);
	}
	else
	{
		e_int_.setZero();
	}

	// 参考加速度
	Eigen::VectorXd ddq_cmd = ddq_des + kd_.cwiseProduct(de) + kp_.cwiseProduct(e);

	if (use_integral_)
		ddq_cmd += ki_.cwiseProduct(e_int_);

	// 逆动力学：tau = M(q)*ddq_cmd + h(q,dq)
	Eigen::VectorXd tau = pinocchio::rnea(model_, *data_, q, dq, ddq_cmd);

	if (use_tau_limit_)
		tau = saturateAbs(tau, tau_limit_abs_);

	return tau;
}

int ComputedTorqueController::nq() const
{
	return model_.nq;
}

int ComputedTorqueController::nv() const
{
	return model_.nv;
}

bool ComputedTorqueController::loaded() const
{
	return loaded_;
}

const pinocchio::Model &ComputedTorqueController::model() const
{
	checkLoaded();
	return model_;
}

const pinocchio::Data &ComputedTorqueController::data() const
{
	checkLoaded();
	return *data_;
}

void ComputedTorqueController::checkLoaded() const
{
	if (!loaded_ || !data_)
	{
		throw std::runtime_error("Model is not loaded.");
	}
}

void ComputedTorqueController::checkVectorSize(const Eigen::VectorXd &x,
												int n,
												const std::string &name) const
{
	if (x.size() != n)
	{
		throw std::runtime_error(
				name + " size mismatch: expected " + std::to_string(n) +
				", got " + std::to_string(x.size()));
	}
}

void ComputedTorqueController::allocateInternalBuffers()
{
	kp_ = Eigen::VectorXd::Zero(n_);
	kd_ = Eigen::VectorXd::Zero(n_);
	ki_ = Eigen::VectorXd::Zero(n_);
	e_int_ = Eigen::VectorXd::Zero(n_);
	tau_limit_abs_ = Eigen::VectorXd::Zero(n_);
	int_limit_abs_ = Eigen::VectorXd::Zero(n_);
	use_integral_ = false;
	use_tau_limit_ = false;
	use_int_limit_ = false;
}

ComputedTorqueController::ModelFormat
ComputedTorqueController::inferFormatFromFile(const std::string &model_file) const
{
	auto lower = model_file;
	std::transform(lower.begin(), lower.end(), lower.begin(),
					[](unsigned char c) { return std::tolower(c); });

	if (lower.size() >= 5 &&
			lower.substr(lower.size() - 5) == ".urdf")
	{
		return ModelFormat::URDF;
	}

	if ((lower.size() >= 5 && lower.substr(lower.size() - 5) == ".mjcf") ||
			(lower.size() >= 4 && lower.substr(lower.size() - 4) == ".xml"))
	{
		return ModelFormat::MJCF;
	}

	throw std::runtime_error(
			"Cannot infer model format from file extension. Please specify URDF or MJCF explicitly.");
}

Eigen::VectorXd ComputedTorqueController::saturateAbs(
		const Eigen::VectorXd &x,
		const Eigen::VectorXd &limit_abs) const
{
	Eigen::VectorXd y = x;
	for (int i = 0; i < y.size(); ++i)
	{
		const double lim = std::abs(limit_abs[i]);
		if (lim > 0.0)
		{
			if (y[i] > lim) y[i] = lim;
			if (y[i] < -lim) y[i] = -lim;
		}
	}
	return y;
}