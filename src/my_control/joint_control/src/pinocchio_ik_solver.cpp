#include "joint_control/pinocchio_ik_solver.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/log.hpp>

#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include <limits>
#include <algorithm>

namespace pin_ik_solver {

PinIkSolver::PinIkSolver() : model_loaded_(false) {}

bool PinIkSolver::loadModel(const std::string& file_path) {
    try {
        pinocchio::urdf::buildModel(file_path, model_);
        model_loaded_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[PinIkSolver] Failed to load URDF from " << file_path
                  << ": " << e.what() << std::endl;
        model_loaded_ = false;
        return false;
    }
}

std::vector<std::string> PinIkSolver::getJointNames() const {
    std::vector<std::string> names;
    if (!model_loaded_) return names;

    for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid) {
        names.push_back(model_.names[jid]);
    }
    return names;
}

bool PinIkSolver::hasJointName(const std::string& name) const {
    if (!model_loaded_) return false;
    pinocchio::JointIndex jid = model_.getJointId(name);
    return (jid > 0 && jid < model_.joints.size());
}

pinocchio::SE3 PinIkSolver::eigenToSE3(const Eigen::Isometry3d& T) const {
    return pinocchio::SE3(T.rotation(), T.translation());
}

pinocchio::FrameIndex PinIkSolver::getFrameId(const std::string& name) const {
    if (!model_loaded_) {
        return static_cast<pinocchio::FrameIndex>(-1);
    }

    // 1) 先按 frame 名查
    if (model_.existFrame(name)) {
        return model_.getFrameId(name);
    }

    // 2) 如果是 joint 名，则优先返回与该 joint 关联的 joint frame / body frame
    if (hasJointName(name)) {
        pinocchio::JointIndex jid = model_.getJointId(name);

        // 优先找 parentJoint == jid 的 BODY frame
        for (pinocchio::FrameIndex fid = 0; fid < model_.frames.size(); ++fid) {
            const auto& fr = model_.frames[fid];
            if (fr.parentJoint == jid && fr.type == pinocchio::BODY) {
                return fid;
            }
        }

        // 再退化找任意 parentJoint == jid 的 frame
        for (pinocchio::FrameIndex fid = 0; fid < model_.frames.size(); ++fid) {
            const auto& fr = model_.frames[fid];
            if (fr.parentJoint == jid) {
                return fid;
            }
        }
    }

    // 3) 如果给的是 link/body 名，很多时候它本身就是某个 BODY frame 的名字
    for (pinocchio::FrameIndex fid = 0; fid < model_.frames.size(); ++fid) {
        if (model_.frames[fid].name == name) {
            return fid;
        }
    }

    return static_cast<pinocchio::FrameIndex>(-1);
}

PinIkResult PinIkSolver::solveIK(const std::vector<std::string>& active_joint_names,
                                 const std::vector<double>& initial_joint_angles,
                                 const std::string& end_effector_name,
                                 const std::string& relative_name,
                                 const Eigen::Isometry3d& target_pose,
                                 const PinIkSettings& settings) {
    PinIkResult result;
    result.success = false;
    result.joint_angles.clear();

    if (!model_loaded_) {
        std::cerr << "[PinIkSolver] Model not loaded." << std::endl;
        return result;
    }

    // 1. 输入检查
    if (active_joint_names.size() != initial_joint_angles.size()) {
        std::cerr << "[PinIkSolver] Size mismatch: active_joint_names ("
                  << active_joint_names.size() << ") vs initial_joint_angles ("
                  << initial_joint_angles.size() << ")" << std::endl;
        return result;
    }

    if (active_joint_names.empty()) {
        std::cerr << "[PinIkSolver] active_joint_names is empty." << std::endl;
        return result;
    }

    // 2. 活动关节检查（当前简单版假设每个活动关节 nq=1, nv=1）
    std::vector<pinocchio::JointIndex> joint_indices;
    std::vector<int> joint_q_offsets;
    std::vector<int> joint_v_offsets;
    std::vector<int> joint_nqs;
    std::vector<int> joint_nvs;

    for (size_t i = 0; i < active_joint_names.size(); ++i) {
        const std::string& name = active_joint_names[i];

        if (!hasJointName(name)) {
            std::cerr << "[PinIkSolver] Joint '" << name << "' not found in model." << std::endl;
            return result;
        }

        pinocchio::JointIndex jid = model_.getJointId(name);
        joint_indices.push_back(jid);

        const auto& jmodel = model_.joints[jid];
        joint_q_offsets.push_back(jmodel.idx_q());
        joint_v_offsets.push_back(jmodel.idx_v());
        joint_nqs.push_back(jmodel.nq());
        joint_nvs.push_back(jmodel.nv());

        if (joint_nqs.back() != 1 || joint_nvs.back() != 1) {
            std::cerr << "[PinIkSolver] Joint '" << name
                      << "' has nq=" << joint_nqs.back()
                      << ", nv=" << joint_nvs.back()
                      << ". Current implementation only supports single-DOF joints."
                      << std::endl;
            return result;
        }
    }

    // 3. 末端 frame 与参考 frame
    pinocchio::FrameIndex end_frame_id = getFrameId(end_effector_name);
    if (end_frame_id == static_cast<pinocchio::FrameIndex>(-1)) {
        std::cerr << "[PinIkSolver] End effector '" << end_effector_name
                  << "' not found as frame/joint/body name." << std::endl;
        return result;
    }

    pinocchio::FrameIndex ref_frame_id = 0; // world
    bool use_world_as_ref = true;

    if (relative_name != "world") {
        ref_frame_id = getFrameId(relative_name);
        if (ref_frame_id == static_cast<pinocchio::FrameIndex>(-1)) {
            std::cerr << "[PinIkSolver] Reference frame '" << relative_name
                      << "' not found." << std::endl;
            return result;
        }
        use_world_as_ref = false;
    }

    // 4. 构造完整初值 q
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < joint_indices.size(); ++i) {
        q[joint_q_offsets[i]] = initial_joint_angles[i];
    }

    // 5. target_pose 转为 pinocchio::SE3
    pinocchio::SE3 target_refMend = eigenToSE3(target_pose);

    // 6. 数据对象
    pinocchio::Data data(model_);

    bool success = false;

    for (int iter = 0; iter < settings.max_iterations; ++iter) {
        // 正运动学
        pinocchio::forwardKinematics(model_, data, q);
        pinocchio::updateFramePlacements(model_, data);
        pinocchio::computeJointJacobians(model_, data, q);

        // 当前末端与参考系位姿
        const pinocchio::SE3& oMend = data.oMf[end_frame_id];
        pinocchio::SE3 oMref = pinocchio::SE3::Identity();
        if (!use_world_as_ref) {
            oMref = data.oMf[ref_frame_id];
        }

        // 当前“末端相对参考系”位姿
        pinocchio::SE3 refMend = oMref.actInv(oMend);

        // 误差：target^{-1} * current
        // 如果想朝着 target 收敛，后续更新时用负号
        pinocchio::SE3 err_SE3 = target_refMend.actInv(refMend);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(err_SE3).toVector();

        if (settings.verbose && (iter % 10 == 0)) {
            std::cout << "[PinIkSolver] Iter " << iter
                      << ", error norm = " << err.norm() << std::endl;
        }

        if (err.norm() < settings.tolerance) {
            success = true;
            if (settings.verbose) {
                std::cout << "[PinIkSolver] Converged at iteration " << iter << std::endl;
            }
            break;
        }

        // 末端在 world 对齐坐标系下的 Jacobian
        Eigen::MatrixXd J_full(6, model_.nv);
        J_full.setZero();
        pinocchio::getFrameJacobian(
            model_, data, end_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

        // 转到参考系表达：J_ref = Ad_{ref<-world} * J_world
        if (!use_world_as_ref) {
            pinocchio::SE3 refMo = oMref.inverse();
            J_full = refMo.toActionMatrix() * J_full;
        }

        // 只取活动关节列
        Eigen::MatrixXd J_act(6, static_cast<int>(joint_indices.size()));
        J_act.setZero();
        for (size_t i = 0; i < joint_indices.size(); ++i) {
            J_act.col(static_cast<int>(i)) = J_full.col(joint_v_offsets[i]);
        }

        // 阻尼最小二乘： dq = - J^T (J J^T + λ I)^-1 e
        Eigen::Matrix<double, 6, 6> JJt;
        JJt.noalias() = J_act * J_act.transpose();
        JJt.diagonal().array() += settings.damping;

        Eigen::Matrix<double, 6, 1> x = JJt.ldlt().solve(err);
        Eigen::VectorXd dq_act = -J_act.transpose() * x;

        // 映射回全模型切空间
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
        for (size_t i = 0; i < joint_indices.size(); ++i) {
            v[joint_v_offsets[i]] = dq_act[static_cast<int>(i)];
        }

        // integrate 更新
        q = pinocchio::integrate(model_, q, v * settings.dt);

        // 简单限位
        for (size_t i = 0; i < joint_indices.size(); ++i) {
            int qidx = joint_q_offsets[i];

            const double lower = model_.lowerPositionLimit[qidx];
            const double upper = model_.upperPositionLimit[qidx];

            const bool lower_finite = std::isfinite(lower);
            const bool upper_finite = std::isfinite(upper);

            if (lower_finite && q[qidx] < lower) q[qidx] = lower;
            if (upper_finite && q[qidx] > upper) q[qidx] = upper;
        }
    }

    if (!success) {
        if (settings.verbose) {
            std::cout << "[PinIkSolver] IK did not converge within "
                      << settings.max_iterations << " iterations." << std::endl;
        }
        return result;
    }

    // 提取活动关节结果
    result.success = true;
    result.joint_angles.reserve(joint_indices.size());
    for (size_t i = 0; i < joint_indices.size(); ++i) {
        result.joint_angles.push_back(q[joint_q_offsets[i]]);
    }

    return result;
}

} // namespace pin_ik_solver