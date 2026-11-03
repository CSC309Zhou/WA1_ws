#ifndef __UPLIMB_CONTROLLER_H_
#define __UPLIMB_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>
#define UPLIMB_ALG_JOINT_MAX_NUM 30
#define FORCE_SENSOR_MAX_NUM 4

typedef struct
{
    uint8_t mode[UPLIMB_ALG_JOINT_MAX_NUM];
    uint16_t statuswd[UPLIMB_ALG_JOINT_MAX_NUM];
    double act_pos[UPLIMB_ALG_JOINT_MAX_NUM];
    double act_vel[UPLIMB_ALG_JOINT_MAX_NUM];
    double act_acc[UPLIMB_ALG_JOINT_MAX_NUM];
    double act_tau[UPLIMB_ALG_JOINT_MAX_NUM];
    unsigned long long int upTime;
} UPLIMB_INPUT_INFO_STRUCT;

typedef struct
{
    double offset_torque[UPLIMB_ALG_JOINT_MAX_NUM];
    double offset_vel[UPLIMB_ALG_JOINT_MAX_NUM];
    uint8_t mode_operation[UPLIMB_ALG_JOINT_MAX_NUM];
    uint16_t ctrlwd[UPLIMB_ALG_JOINT_MAX_NUM];
    double cmd_pos[UPLIMB_ALG_JOINT_MAX_NUM];
    double cmd_vel[UPLIMB_ALG_JOINT_MAX_NUM];
    double cmd_tau[UPLIMB_ALG_JOINT_MAX_NUM];
    unsigned long long int downTime;
} UPLIMB_OUTPUT_INFO_STRUCT;

typedef struct
{
    int var1[UPLIMB_ALG_JOINT_MAX_NUM];
    int var2[UPLIMB_ALG_JOINT_MAX_NUM];
    int var3[UPLIMB_ALG_JOINT_MAX_NUM];
    int var4[UPLIMB_ALG_JOINT_MAX_NUM];
    int var5[UPLIMB_ALG_JOINT_MAX_NUM];
    double var6[UPLIMB_ALG_JOINT_MAX_NUM];
    double var7[UPLIMB_ALG_JOINT_MAX_NUM];
    double var8[UPLIMB_ALG_JOINT_MAX_NUM];
    double var9[UPLIMB_ALG_JOINT_MAX_NUM];
    double var10[UPLIMB_ALG_JOINT_MAX_NUM];
} UPLIMB_VAR_OBSERVE;


typedef struct
{
    unsigned short dataNum; ///< 计数
    float fx;               ///< X轴方向的力
    float fy;               ///< Y轴方向的力
    float fz;               ///< Z轴方向的力
    float mx;               ///< X轴方向的扭矩
    float my;               ///< Y轴方向的扭矩
    float mz;               ///< Z轴方向的扭矩
} FT_SENSOR_DATA_STRUCT;
typedef struct
{
    FT_SENSOR_DATA_STRUCT force[FORCE_SENSOR_MAX_NUM];
} ROBOT_FT_SENSOR_STATE_INFO_STRUCT;

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <iostream>
#include <vector>
#include <variant>

template<typename T>
struct always_false : std::false_type {};
using CartersianPose = std::variant<Eigen::Matrix<double, 6, 1>,  std::vector<Eigen::Matrix<double, 6, 1>>>;
using CartersianVel = CartersianPose;
using q7_Type = std::variant<double, std::vector<double>>;

enum class RobotRunMode { PT = 4, CSP = 8, CSV, CST };
enum class ControllerParasList { JOINT_COLLISION_THRESHOLD=2, KP = 3, KD = 4, K_IMPDC, D_IMPDC};
enum class ControllerMode { POSITION_CSP = 0, POSITION_CST = 1, IMPEDANCE };  

/**
 * @brief: 新臂计算运动学逆解
 * @param {CartersianPose} &pose  目标位姿，输入的数据类型支持下面两种
 *        - Eigen::Matrix<double, 6, 1>              单臂目标位姿，6维，xyz+rpy(旋转顺序为ZYX)
 *        - std::vector<Eigen::Matrix<double, 6, 1>> 双臂目标位姿，2*6维，pose[.]为xyz+rpy(旋转顺序为ZYX)
 * @param {q7_type} q7            支持下面两种类型
 *         - double               单臂第7关节角
 *         - std::vector<double>  双臂第7关节角
 * @param {VectorXd} &q          返回所有可能的关节角，单臂时外层容器大小为1，双臂时为2，内层std::vector的大小表示有几组解，VectorXd的维度是7
 * @param {double} &phi          与逆解对应的臂角
 * @param {int} arm_type         左1 右2 双臂3
 * @return {*}                   返回存在逆解的手臂对应的arm_type，左臂有解返回1，右臂有解返回2，双臂有解返回3
 */
int getInverseKinematics(const CartersianPose& pose,
                          const q7_Type& q7,
                          ::std::vector<::std::vector<Eigen::VectorXd>>& q,
                          ::std::vector<::std::vector<double>>& phi, 
                          int arm_type=3);

/**
 * @brief: 初始化控制器
 * @param {char} *filePath         配置文件路径
 */
void init(std::string config_path = "");

/**
 * @brief: 运动至指定关节角
 * @param {VectorXd} &q           目标关节角，维数同urdf
 * @param {double} speed          最大运行速度
 * @param {double} acceleration   最大运行加速度
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveJ(const Eigen::VectorXd &q, double speed=1.05, double acceleration=1.4, bool asynchronous=false, int arm_type=15);

/**
 * @brief: 依次通过多个指定的关节角位置
 * @note: 该接口隐含当前位置为第一个路径点。
 * @param {std::vector<std::vector<double>>} &path  指定的关节角路径，第一个维度表示路径点个数（要求大于等于2)，此处隐含当前位置为第一个路径点；第二个维度表示每个路径点的信息，包含：pos1， pos2, ..., posN，N是arm_type所对应的自由度。
 * @param {double} &time          到达最后一个点的时间，单位为秒，中间的时间点将依次等差递增
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveJ(const std::vector<std::vector<double>> &path, const double& time, bool asynchronous=false, int arm_type=15);

/**
 * @brief: 依次通过多个指定的关节角位置
 * @note: 该接口隐含当前位置为第一个路径点，前位置对应的时刻为0.0，因此传入的第一个路径点对应的时刻time[0]必须大于0.0。
 * @param {std::vector<std::vector<double>>} &path  指定的关节角路径，第一个维度表示路径点个数（要求大于等于2)，此处隐含当前位置为第一个路径点；第二个维度表示每个路径点的信息，包含：pos1， pos2, ..., posN，N是arm_type所对应的自由度。
 * @param {std::vector<double>} &time   路径点对应的时刻，此处隐含当前位置对应的时刻为0.0，单位为秒。
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveJ(const std::vector<std::vector<double>> &path, const std::vector<double>& time, bool asynchronous=false, int arm_type=15);

/**
 * @brief: 运动至指定关节角
 * @param {VectorXd} &q           目标关节角，维数同urdf
 * @param {double} freq           调用频率
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveJ(const Eigen::VectorXd &q, double freq, bool asynchronous=false, int arm_type=15);

/**
 * @brief: 运动至指定关节角
 * @param {VectorXd} &q           目标关节角，维数同urdf
 * @param {VectorXd} &qd          目标关节角速度，维数同urdf
 * @param {double} freq           调用频率
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveJ(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, double freq, bool asynchronous=false, int arm_type=15);

/**
 * @brief: 运动至指定位姿，不确保末端执行是否为直线
 * @param {CartersianPose} &pose  目标位姿，输入的数据类型支持下面两种
 *        - Eigen::Matrix<double, 6, 1>              单臂目标位姿，6维，xyz+rpy(旋转顺序为ZYX)
 *        - std::vector<Eigen::Matrix<double, 6, 1>> 双臂目标位姿，2*6维，pose[.]为xyz+rpy(旋转顺序为ZYX)
 * @param {q7_type} q7            支持下面两种类型
 *         - double               单臂第7关节角
 *         - std::vector<double>  双臂第7关节角
 * @param {double} speed          最大运行速度
 * @param {double} acceleration   最大运行加速度
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 双臂3
 * @return {*}                    返回存在规划成功的手臂对应的arm_type，左臂规划成功返回1，右臂规划成功返回2，双臂规划成功返回3
 */
int moveJ(const CartersianPose& pose, const q7_Type& q7, double speed=1.05, double acceleration=1.4, bool asynchronous=false, int arm_type=3);

/**
 * @brief: 运动至双臂目标位姿，并且距离参考关节角最近(测试用)
 * @param {std::vector<::Eigen::Matrix<double, 6, 1>>} &pose        左右手臂目标位姿，2个6维向量，xyz+rpy(旋转顺序为ZYX)
 * @param {VectorXd} &ref_q       参考关节角
 * @param {double} speed          最大运行速度
 * @param {double} acceleration   最大运行加速度
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveL(const ::std::vector<::Eigen::Matrix<double, 6, 1>> &pose, const ::Eigen::VectorXd& ref_q, double speed=0.25, double acceleration=1.2, bool asynchronous=false);

/**
 * @brief: 运动至双臂目标位姿
 * @param {CartersianPose} &pose  输入的数据类型支持下面两种
 *        - Eigen::Matrix<double, 6, 1>              单臂目标位姿，6维，xyz+rpy(旋转顺序为ZYX)
 *        - std::vector<Eigen::Matrix<double, 6, 1>> 双臂目标位姿，2*6维，pose[.]为xyz+rpy(旋转顺序为ZYX)
 * @param {double} speed          最大运行速度
 * @param {double} acceleration   最大运行加速度
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveL(const CartersianPose& pose, double speed=0.25, double acceleration=1.2, bool asynchronous=false, int arm_type=3);

/**
 * @brief: 依次通过多个指定的位姿
 * @note: 该接口隐含当前位置为第一个路径点。
 * @param {std::vector<std::vector<double>>} &path  指定的位姿路径，第一个维度表示路径点个数（要求大于等于2）；第二个维度表示每个路径点的信息，至少包含左臂xyz+rpy或右臂xyz+rpy(旋转顺序为ZYX)中的一个。
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveL(const std::vector<std::vector<double>> &path, const double &time, const bool& asynchronous=false, int arm_type=3);

/**
 * @brief: 依次通过多个指定的位姿
 * @note: 该接口隐含当前位置为第一个路径点，前位置对应的时刻为0.0，因此传入的第一个路径点对应的时刻time[0]必须大于0.0。
 * @param {std::vector<std::vector<double>>} &path  指定的位姿路径，第一个维度表示路径点个数（要求大于等于2）；第二个维度表示每个路径点的信息，至少包含左臂xyz+rpy或右臂xyz+rpy(旋转顺序为ZYX)中的一个。
 * @param {bool} asynchronous     是否阻塞，false阻塞，true不阻塞
 * @param {int} arm_type          左1 右2 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool moveL(const std::vector<std::vector<double>> &path, const ::std::vector<double> &time, const bool& asynchronous=false, int arm_type=3);

/**
 * @brief: 高频调用关节空间位置函数
 * @param {VectorXd} &q           关节空间位置
 * @param {double} speed          最大速度，当前无用
 * @param {double} acceleration   最大加速度，当前无用
 * @param {double} time           轨迹时间
 * @param {double} lookahead_time 轨迹lookahead时间，暂时使用默认值
 * @param {double} gain           增益调节，调整轨迹跟踪效果
 * @param {int} arm_type          左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}
 */
bool servoJ(const Eigen::VectorXd &q, double speed, double acceleration, double time, double lookahead_time=0.2, double gain=1000, int arm_type=15);

/**
 * @brief: 高频调用笛卡尔空间位置函数
 * @param {CartersianPose} &pose  输入的数据类型支持下面两种
 *        - Eigen::Matrix<double, 6, 1>              单臂目标位姿，6维，xyz+rpy(旋转顺序为ZYX)
 *        - std::vector<Eigen::Matrix<double, 6, 1>> 双臂目标位姿，2*6维，pose[.]为xyz+rpy(旋转顺序为ZYX)
 * @param {double} speed          最大速度，当前无用
 * @param {double} acceleration   最大加速度，当前无用
 * @param {double} time           轨迹时间
 * @param {double} lookahead_time 轨迹lookahead时间，暂时使用默认值
 * @param {double} gain           增益调节，调整轨迹跟踪效果
 * @param {int} arm_type          左1 右2 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                    运行正确返回1，错误返回0
 */
bool servoL(const CartersianPose& pose, double speed, double acceleration, double time, double lookahead_time=0.2, double gain=1000, int arm_type=3);

/**
 * @brief: 关节空间速度控制
 * @param {VectorXd} &qd        目标关节角速度，维度同arm_type所对应的自由度
 * @param {double} acceleration 最大运行加速度
 * @param {double} time         阻塞时间，大于2倍的控制周期时生效，否则不生效。
 * @param {int} arm_type        左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                  运行正确返回1，错误返回0
 */
bool speedJ(const Eigen::VectorXd &qd, double acceleration=0.5, double time=0.1, int arm_type=15);

/**
 * @brief: 笛卡尔空间速度控制
 * @param {CartersianVel} &xd  输入的数据类型支持下面两种
 *        - Eigen::Matrix<double, 6, 1>              单臂目标位姿速度，6维
 *        - std::vector<Eigen::Matrix<double, 6, 1>> 双臂目标位姿速度，2*6维
 * @param {double} acceleration 最大运行加速度
 * @param {double} time         阻塞时间，大于2倍的控制周期时生效，否则不生效。
 * @param {int} arm_type        左1 右2 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                  运行正确返回1，错误返回0
 */
bool speedL(const CartersianVel& xd, const double& acceleration=0.25, const double& time=0.1, int arm_type=3);

/**
 * @brief: 停止运动
 * @param {double} a             停止加速度
 * @return {*}                   运行正确返回1，错误返回0
 */
bool speedStop(double a=10.0);

/**
 * @brief: 设置伺服运行模式，最高调用频率1000Hz
 * @param {RobotRunMode} run_mode   运行模式，枚举类型，PT、CSP、CSV、CST
 * @param {int} arm_type            左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                      运行正确返回1，错误返回0
 */
bool setJointModeOperation(const RobotRunMode& run_mode, int arm_type);

/**
 * @brief: 开启/关闭碰撞检测
 * @param  {bool} enable         true开启，false关闭
 * @return {*}                   运行正确返回1，错误返回0
 */
bool setCollisionDetection(bool enable);

/**
 * @brief: 获取碰撞检测开关状态
 * @return {*}                   开启状态返回1，关闭状态返回0
 */
bool getCollisionDetection();

/**
 * @brief: 设置笛卡尔空间控制接口使用的关节，最高调用频率100Hz
 * @param {VectorXd} &mask           关节掩码，1启用，0屏蔽，顺序与arm_type对应关节顺序一致
 * @param {int} arm_type             左1 右2,支持8421
 * @return {*}                       运行正确返回1，错误返回0
 */
bool setCartesianPlanningJoints(const Eigen::VectorXd& mask, int arm_type);

/**
 * @brief: 设置力矩控制器参数，最高调用频率100Hz
 * @param {ControllerParasList} para_label  参数标签，枚举类型，JOINT_COLLISION_THRESHOLD、KP、KD、K_IMPDC、D_IMPDC
 * @param {VectorXd} &para           与参数标签对应的参数
 * @param {int} arm_type             左1 右2,支持8421
 * @return {*}                       运行正确返回1，错误返回0
 */
bool setControllerPara(const ControllerParasList& para_label, const Eigen::VectorXd& para, int arm_type);

/**
 * @brief: 获取力矩控制器参数，最高调用频率100Hz
 * @param {ControllerParasList} para_label  参数标签，枚举类型，JOINT_COLLISION_THRESHOLD、KP、KD、K_IMPDC、D_IMPDC
 * @param {int} arm_type             左1 右2,支持8421
 * @return {*}                       返回与参数标签对应的参数
 */
::Eigen::VectorXd getControllerPara(const ControllerParasList& para_label, int arm_type);

/**
 * @brief: 设置双臂控制模式
 * @param {ControllerMode} mode      控制模式，枚举类型，POSITION_CSP(位控位置模式)、 POSITION_CST(力控位置模式)、IMPEDANCE(阻抗模式)
 * @return {*}                       运行正确返回1，错误返回0
 */
bool setControllerMode(const ControllerMode &mode);

/**
 * @brief: 获取安全锁定状态
 * @return {*}                   锁定状态返回1，非锁定状态返回0
 */
bool getSafetyLock();

/**
 * @brief: 释放安全锁定
 * @return {*}                    运行正确返回1，错误返回0
 */
bool releaseSafetyLock();

/**
 * @brief: 设置双臂末端负载
 * @param {std::vector<double>} &load_mass 负载质量，顺序为左臂负载、右臂负载。
 * @param {::std::vector<::Eigen::Matrix<double, 3, 1>>} &load_barycenter_trans 负载质心位置（x,y,z）（末端坐标系为参考），顺序为左臂质心位置、右臂质心位置。
 * @return {*}                    无
 */
bool setLoadParas(::std::vector<double>& load_mass, ::std::vector<::Eigen::Matrix<double, 3, 1>>& load_barycenter_trans);

/**
 * @brief: 直接通过伺服下发关节位置，最高调用频率1000Hz（无插补）
 * @param {VectorXd} &q             期望关节位置
 * @param {int} arm_type            左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                      运行正确返回1，错误返回0
 */
bool setJointPosition(const Eigen::VectorXd &q, int arm_type);

/**
 * @brief: 直接通过伺服下发关节力矩，最高调用频率1000Hz
 * @param {VectorXd} &tau           期望关节力矩
 * @param {int} arm_type            左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                      运行正确返回1，错误返回0
 */
bool setJointTorque(const Eigen::VectorXd &tau, int arm_type);

/**
 * @brief: 运动至home位置
 * @param {int} arm_type         左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                   运行正确返回1，错误返回0
 */
bool goHome(int arm_type=15);

/**
 * @brief: 获取关节信息
 * @param {int} arm_type         左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                   返回当前的关节角信息
 */
Eigen::VectorXd getJointActualPositions(int arm_type=15);

/**
 * @brief: 获取关节角速度信息
 * @param {int} arm_type         左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @return {*}                   返回当前的关节角速度信息
 */
Eigen::VectorXd getJointActualVelocities(int arm_type=15);

/**
 * @brief: 计算当前关节角位置前向运动学
 * @return {*}                   返回对应arm_type的当前末端位姿，N个6维向量，N是arm_type对应的部位数量，按照部位对应的arm_type从小到大存放，xyz+rpy(旋转顺序和欧拉角存放顺序均为ZYX)
 */
std::vector<::Eigen::Matrix<double, 6, 1>> getForwardKinematics(int arm_type=3);

/**
 * @brief: 计算指定关节角位置前向运动学
 * @param {VectorXd} &q          指定关节角(所有关节角)
 * @param {int} arm_type         左1 右2
 * @return {*}                   返回对应arm_type和q的当前末端位姿，N个6维向量，N是arm_type对应的部位数量，按照部位对应的arm_type从小到大存放，xyz+rpy(旋转顺序和欧拉角存放顺序均为ZYX)
 */
std::vector<::Eigen::Matrix<double, 6, 1>> getForwardKinematics(const Eigen::VectorXd &q, int arm_type=3);

/**
 * @brief: 计算当前末端的速度
 * @return {*}                   返回双臂末端速度，左右
 */
std::vector<::Eigen::Matrix<double, 6, 1>> getEEVel();

/**
 * @brief: 计算当前末端的速度
 * @param {int} arm_type         左1 右2
 * @return {*}                   返回指定手臂末端速度
 */
Eigen::VectorXd getEEVel(int arm_type);

/**
 * @brief: 获取当前关节位置的重力
 * @param  {VectorXd} tau_g       重力矩
 * @return {*}                    运行正确返回1，错误返回0
 */
bool getGravityTorque(Eigen::VectorXd &tau_g);

/**
 * @brief: 计算当前双臂雅可比矩阵
 * @param {int} arm_type         左1 右2 腰(轮臂腰)8 轮臂腰上下平移16，支持8421，默认0为全部
 * @return {*}                   返回双臂笛卡尔规划组对应的雅可比矩阵，索引为左0、右1，矩阵内部顺序为从根部到末端
 */
::std::vector<Eigen::MatrixXd> getJacobian(int arm_type);

/**
 * @brief: 清除离线关节轨迹
 * @param  {string} traj_name     轨迹名称
 * @return {*}             
 */
void clearTrajectory(const std::string& traj_name = "");

/**
 * @brief: 获取离线关节轨迹名称列表
 * @return {vector<std::string>} 返回预加载轨迹名称列表            
 */
::std::vector<std::string> getLoadedTrajectoryNames();

/**
 * @brief: 加载离线关节轨迹(1kHz频率)，不运行
 * @param  {vector<std::vector<double>>} data 轨迹数据, 外层容器大小代表轨迹长度，内层容器大小表示arm_type所对应的自由度，传入数据使用std::move(data)传入
 * @param  {int} arm_type         左1 右2 颈4 腰(轮臂腰上下平移)8 轮臂腰16，支持8421
 * @param  {string} traj_name     轨迹名称
 * @return {*}                    运行正确返回1，错误返回0
 */
bool loadTrajData(std::vector<std::vector<double>>&& data, int arm_type, const std::string& traj_name);

/**
 * @brief: 执行预加载关节轨迹
 * @param  {string} traj_name     轨迹名称
 * @param  {bool} asynchronous    是否阻塞，false阻塞，true不阻塞
 * @return {*}                    运行正确返回1，错误返回0
 */
bool preloadedTrajMove(const std::string& traj_name, bool asynchronous);

/**
 * @brief: 获取正在执行的预加载关节轨迹名称和当前执行步长
 * @param  {string} traj_name     返回值为1时，为当前正在执行的轨迹名称，
 *                                返回值为0时，为上一次执行的轨迹，未执行过预加载轨迹时为空字符串
 * @param  {int} trajectory_current_step    返回值为1时，为当前执行步长
 *                                          返回值为0时，为最后执行的步长（若等于对应轨迹长度则为执行完成，否则没有执行完成）
 * @return {*}                    正在执行预加载轨迹返回1，否则返回0
 */
bool getTrajectoryMoving(std::string& traj_name, int& trajectory_current_step);

/**
 * @brief: 示教模式
 * @return {*}                    运行正确返回1，错误返回0
 */
bool teachMode(int arm_type);

/**
 * @brief: 结束示教模式
 * @return {*}                    运行正确返回1，错误返回0
 */
bool endTeachMode(int arm_type);

/**
 * @brief: 获取机器人的自由度
 * @return 机器人的自由度
 */
::std::size_t getDof();

/**
 * @brief: 获取驱动器信息
 * @param {UPLIMB_INPUT_INFO_STRUCT} *recv_info  接收到的驱动器信息
 * @return {*}
 */
void getDriverInfo(UPLIMB_INPUT_INFO_STRUCT *recv_info);

void getArmState(UPLIMB_INPUT_INFO_STRUCT *recv_info);


/**
 * @brief: 发送控制器变量
 * @param {UPLIMB_VAR_OBSERVE} *sendVar  待观测的变量
 * @return {*}
 */
void sendVarInfo(UPLIMB_VAR_OBSERVE *sendVar);

void getArmVar(UPLIMB_VAR_OBSERVE *sendVar);

/**
 * @brief: 发送驱动器信息, 全部
 * @param {UPLIMB_OUTPUT_INFO_STRUCT} *sendInfo  发送给驱动器的信息
 * @return {*}
 */
void sendDriverInfo(UPLIMB_OUTPUT_INFO_STRUCT *sendInfo);

void getArmCMD(UPLIMB_OUTPUT_INFO_STRUCT *sendInfo);

/**
 * @brief: 获取末端六维力传感器信息
 * @param {ROBOT_FT_SENSOR_STATE_INFO_STRUCT} *forceInfo  末端六维力信息
 * @return {*}
 */
void getForceSensorInfo(ROBOT_FT_SENSOR_STATE_INFO_STRUCT *forceInfo);

/**
 * @brief: 获取机器人的运动状态
 * @return 返回机器人执行指令的代码，
 *        -0 停止状态，1 MOVEJ，2 MOVEJ_PATH，
 *        -3 MOVEL_NULLSPACE，4 MOVEL， 
 *        -5 MOVEL_PATH，6 SPEEDJ，7 SPEEDL，
 *        -8 SPEED_STOP，9 PRELOADED_TRAJ，
 *        -10 MOVEFOURIER，11 MOVEJ_SPLINE，
 *        -12 TEACH，13 SERVOJ，14 SERVOL，15 PROTECTED。
 */
int getRobotCommand();

/**
 * @brief: 获取指定关节角下双臂是否处于奇异位姿
 * @param  {VectorXd} q           指定关节角
 * @param {int} arm_type          左1 右2 腰(轮臂腰)8 轮臂腰上下平移16，支持8421
 * @return {*}                    若双臂处于奇异状态，则返回双臂对应的arm_type，非奇异返回0，输入有误返回-1
 */
int isSingular(const Eigen::VectorXd &q, int arm_type);

/**
 * @brief: 获取双臂当前是否处于奇异位姿
 * @param {int} arm_type          左1 右2 腰(轮臂腰)8 轮臂腰上下平移16，支持8421
 * @return {*}                    若双臂处于奇异状态，则返回双臂对应的arm_type，非奇异返回0，输入有误返回-1
 */
int isSingular(int arm_type=3);

/**
 * @brief: 测试用接口
 * @return {*}
 */
bool loadControllerParas(std::string file_name);

/**
 * @brief 获取库版本号
 * @return 版本字符串，格式为"MAJOR.MINOR.PATCH.TWEAK"
 */
const char* getVersion();

/**
 * @brief 获取详细版本信息
 * @return 包含版本号、构建时间、Git提交哈希的字符串
 */
const char* getDetailedVersionInfo();
 
// // To do
// bool servoStop(double a=10.0, int arm_type=0);


#endif