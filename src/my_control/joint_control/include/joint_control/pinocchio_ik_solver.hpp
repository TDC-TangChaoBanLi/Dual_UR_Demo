#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace pin_ik_solver {

struct PinIkSettings {
    int max_iterations = 200;
    double tolerance = 1e-4;
    double damping = 1e-6;   // DLS 阻尼
    double dt = 0.2;         // 迭代步长
    bool verbose = false;
};

struct PinIkResult {
    bool success = false;
    std::vector<double> joint_angles;
};

class PinIkSolver {
public:
    PinIkSolver();

    // 目前这个简单版只加载 URDF
    bool loadModel(const std::string& file_path);

    std::vector<std::string> getJointNames() const;

    // name 可以是 frame 名、joint 名，或常见的 link/body 对应 frame 名
    pinocchio::FrameIndex getFrameId(const std::string& name) const;

    PinIkResult solveIK(const std::vector<std::string>& active_joint_names,
                        const std::vector<double>& initial_joint_angles,
                        const std::string& end_effector_name,
                        const std::string& relative_name,
                        const Eigen::Isometry3d& target_pose,
                        const PinIkSettings& settings = PinIkSettings());

private:
    bool hasJointName(const std::string& name) const;

    pinocchio::SE3 eigenToSE3(const Eigen::Isometry3d& T) const;

private:
    pinocchio::Model model_;
    bool model_loaded_;
};

} // namespace pin_ik_solver