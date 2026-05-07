//
// Created by chengfeng on 2026/4/18.
//

#ifndef TEST_FEEDBACK_ARM_H
#define TEST_FEEDBACK_ARM_H

#include "dvc_motor.h"
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

class BaseMotor {
public:
    virtual ~BaseMotor() = default;
    virtual void Set_Target_Angle(float angle) = 0; // 设置目标角度(弧度)
};

class MotorAdapter_C610 : public BaseMotor {
public:
    explicit MotorAdapter_C610(Class_Motor_C610& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_C610& real_motor_;
};

class MotorAdapter_C620 : public BaseMotor {
public:
    explicit MotorAdapter_C620(Class_Motor_C620& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_C620& real_motor_;
};

// ==========================================
// 1. 轨迹规划类 (5次多项式 S 型曲线)
// ==========================================
class QuinticPlanner {
public:
    enum State { UNPLANNING, PLANNING };

    explicit QuinticPlanner(float pos)
        : q0(pos), qf(pos), state(UNPLANNING), current_target(pos) {
    }

    // 规划一段新的轨迹
    void Plan(float start_pos, float target_pos, float time_duration);

    // 获取 dt 时间后的期望位置
    float GetNextPosition(float dt);

    bool IsPlanning() const;

    float GetCurrentTarget() const;

    void ForceSetPosition(float pos);

private:
    float q0, qf;
    float t, T;
    State state;
    float current_target;
};

class RobotJoint {
public:
    // 直接接收 BaseMotor，实现与具体硬件型号彻底解耦
    RobotJoint(BaseMotor &motor_ptr, float min_limit, float max_limit, float zero_pos, float dir, float ratio)
        : motor_(motor_ptr), v_min_(min_limit), v_max_(max_limit),
          pos_at_zero_(zero_pos), direction_(dir), transmission_ratio_(ratio) {}

    bool IsWithinLimits(float target_val) const;
    void Update(float current_val);

private:
    BaseMotor &motor_;
    float v_min_, v_max_;
    float pos_at_zero_;
    float direction_;
    float transmission_ratio_;
};

class PlannedJoint {
public:
    PlannedJoint(BaseMotor &motor, float min_limit, float max_limit, float zero_pos, float dir, float ratio, float dt)
        : joint_(motor, min_limit, max_limit, zero_pos, dir, ratio), planner_(zero_pos), dt_(dt) {}

    void Move(float target, float duration);
    bool IsMoving() const;
    void Update();

private:
    RobotJoint joint_;
    QuinticPlanner planner_;
    float dt_;
};

// 定义单个关节的动作指令
struct JointCmd {
    float target;
    float duration;
};

// 带有模板参数 N 的动作步骤，N 代表关节数
template <size_t N>
struct ArmStep {
    JointCmd cmds[N];
    void (*custom_action)();
};

// 带有模板参数 N 的播放器类
template <size_t N>
class ArmSequencePlayer {
public:
    // 变长参数模板构造函数，可以直接接收任意数量的 PlannedJoint&
    template <typename... Joints>
    ArmSequencePlayer(Joints&... joints)
        : state_(IDLE), current_sequence_(nullptr), joints_{ &joints... }
    {
        // 编译期断言：确保传入的引用数量等于实例化的模板参数 N
        static_assert(sizeof...(Joints) == N, "传入的关节数量与实例化模板的 N 不匹配!");
    }

    void Play(const ArmStep<N>* sequence, uint16_t step_count) {
        current_sequence_ = sequence;
        total_steps_ = step_count;
        current_step_index_ = 0;
        state_ = RUNNING_STEP;
    }

    void Stop() {
        state_ = IDLE;
    }

    bool IsPlaying() const {
        return state_ != IDLE;
    }

    void Update() {
        if (state_ == IDLE) return;

        if (state_ == RUNNING_STEP) {
            const ArmStep<N>& step = current_sequence_[current_step_index_];

            // 优先执行自定义动作
            if (step.custom_action != nullptr) {
                step.custom_action();
            }

            // 遍历并下发 N 个关节的指令
            for (size_t i = 0; i < N; ++i) {
                if (step.cmds[i].duration > 0.0f) {
                    joints_[i]->Move(step.cmds[i].target, step.cmds[i].duration);
                }
            }

            state_ = WAITING_STEP;
        }
        else if (state_ == WAITING_STEP) {
            bool all_finished = true;
            // 检查所有的轴是否都已经运动完毕
            for (size_t i = 0; i < N; ++i) {
                if (joints_[i]->IsMoving()) {
                    all_finished = false;
                    break;
                }
            }

            if (all_finished) {
                current_step_index_++;
                if (current_step_index_ >= total_steps_) {
                    state_ = IDLE; // 完成
                } else {
                    state_ = RUNNING_STEP; // 下一步
                }
            }
        }
    }

private:
    enum State { IDLE, RUNNING_STEP, WAITING_STEP };
    State state_;

    const ArmStep<N>* current_sequence_;
    uint16_t total_steps_;
    uint16_t current_step_index_;

    // 内部存放各个关节引用的指针数组
    PlannedJoint* joints_[N];
};

#endif // TEST_FEEDBACK_ARM_H
