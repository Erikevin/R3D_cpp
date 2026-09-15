#pragma once
// startRecord 前置的相机曝光/增益快速整定 —— C++ 移植自 adjust_cam_param.py /
// post_proc_tool/v4l2_tune.py 的 policy_step 闭环：抓帧 → 统计平均亮度/高光占比 →
// 按策略改 exposure/gain → 重复直到收敛。
//
// 与桌面端工具的差异只在档位：本版受 2 s 总预算 / ≤5 轮约束（startRecord 到拉起
// 建图进程链之间只留这么久），故用 640x480@30fps 小流抓帧压缩每轮耗时；策略本体
// （目标 120±15、高光≤8%、先曝光后增益、单轮乘性 ×0.4~×2.5、顶边界判定）与
// adjust_cam_param.py 逐行一致。
//
// 亮度统计与分辨率无关（同一传感器同一曝光），小流统计的整定值对录制用的
// 2592x1944 全分辨率流同样成立 —— 桌面端工具同样以预览小流统计、整定结果供
// 高分辨率抓图使用。整定结束 release 设备，参数留存于驱动；随后
// image_odom_recorder 以 auto_exposure:=false 打开即沿用整定值。

#include <string>

namespace r3d_ws {

struct CamTuneConfig {
  std::string device;        // 如 "/dev/video0"
  std::string side;          // "左" / "右"（仅日志标签）
  double target = 120.0;     // 目标平均亮度（后处理提亮目标 128，抓图略低留余量）
  double tol = 15.0;         // 死区半宽：|mean-target| ≤ tol 即收敛
  double hl_max = 0.08;      // 高光(≥250)像素占比上限
  double max_exp_ms = 10.0;  // 曝光上限 ms（再大运动拖影明显）
  int max_gain = 64;         // 增益上限（再大噪点明显）
  int warmup = 3;            // 每轮丢弃的旧参数残留帧数（Python 版 4，快速档 3）
  int frames = 3;            // 每轮统计帧数（Python 版 5，快速档 3）
  int settle_ms = 80;        // 改参后等相机稳定时长（Python 版 350ms）
  int max_iter = 5;          // 最大迭代轮数（预算要求；Python 版 10）
  int budget_ms = 2000;      // 单相机总时长预算（硬上限，含打开相机）
  int width = 640;           // 整定用小分辨率（30fps 下每轮 6 帧仅 ~0.2s）
  int height = 480;
  int fps = 30;
};

struct CamTuneResult {
  bool ok = false;         // 整定流程完成且相机已切手动（详见 cam_tuner.cpp 语义注释）
  bool converged = false;  // 进入目标区间且高光达标
  double mean = 0.0;       // 末轮统计平均亮度
  double hl = 0.0;         // 末轮统计高光占比 [0,1]
  double exp_ms = 0.0;     // 末次曝光
  int gain = 0;            // 末次增益
  int iters = 0;           // 实际完成轮数
};

// 单相机闭环整定。相机独占打开（须先于 image_odom_recorder 启动），结束时释放。
CamTuneResult cam_tune_one(const CamTuneConfig &cfg);

// startRecord 用：左右双相机（config::TUNE_DEV0/2）并行整定。
// 返回是否两台均 ok —— 全部 ok 时调用方应让 image_odom_recorder 以手动曝光
// 启动以保留整定值；任一失败则回退配置文件的 auto_exposure（AE 兜底）。
bool cam_tune_pair();

}  // namespace r3d_ws
