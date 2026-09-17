#pragma once
// startRecord 前置的相机曝光/增益快速整定 —— C++ 移植自 adjust_cam_param.py /
// post_proc_tool/v4l2_tune.py 的 policy_step 闭环：抓帧 → 统计平均亮度/高光占比 →
// 按策略改 exposure/gain → 重复直到收敛。
//
// 与桌面端工具（adjust_cam_param.py）的三处板端演进（桌面版未同步）：
//   1) 档位：受 3 s 总预算 / ≤3 轮约束（startRecord 到拉起建图进程链之间只留这么久）。
//      open ~1s + 3×(6 帧采集 ~0.2s + settle 60ms) ≈ 1.8s，典型场景 2~3 轮收敛；
//   2) 目标亮度 70±15（桌面版 120±15）：略欠曝换高光余量，录制后由 post_proc_tool
//      动态提亮到 128（factor≈1.8，会放大 8-bit 量化噪声——用噪声换高光的取舍）；
//   3) 过曝步长削顶感知：mean 被削顶钳在 ~250 时 deadbeat 因子 target/mean 低估
//      所需降幅（120/255≈×0.47，0.4 下限本就够不着），按高光占比放大等效 mean
//      （×(1+1.5·hl)）并放宽单轮下限到 ×0.25 —— 大面积过曝 3 轮内可跳达目标。
//
// 策略参数（target/tol/hl_max/max_iter 等）外部化到 config/cam_tune.json（可用
// $R3D_CAM_TUNE_FILE 覆盖路径，同 mapping_commands.json 约定），改文件后下一次
// startRecord 即生效，无需重编译/重启桥；任何加载失败逐项回退到本文件编译期默认。
// 整定起点固定为默认参数 kDefaultExpMs/kDefaultGain（7.8ms/gain=16），不用切
// Manual 后驱动的残留值。
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
  double target = 70.0;      // 目标平均亮度（ETTL：略欠曝保高光，后处理提亮到 128）
  double tol = 15.0;         // 死区半宽：|mean-target| ≤ tol 即收敛（死区 [55,85]）
  double hl_max = 0.08;      // 高光(≥250)像素占比上限
  double max_exp_ms = 10.0;  // 曝光上限 ms（再大运动拖影明显）
  int max_gain = 64;         // 增益上限（再大噪点明显）
  int warmup = 3;            // 每轮丢弃的旧参数残留帧数（Python 版 4，快速档 3）
  int frames = 3;            // 每轮统计帧数（Python 版 5，快速档 3）
  int settle_ms = 60;        // 改参后等相机稳定时长（Python 版 350ms；板上实测
                             // open 已耗 ~1s，压缩 settle 给整定轮留预算）
  int max_iter = 3;          // 最大迭代轮数（Python 版 10、旧板端档 5）。3 轮耗时
                             // ≈ 1.8s < 3s 预算；大面积过曝靠削顶感知步长 3 轮压达
  int budget_ms = 5000;      // 单相机总时长预算（硬上限，含打开相机；板上实测
                             // open 即耗 ~1s，2s 下有效轮次不足，放宽到 3s）
  int width = 640;           // 整定用小分辨率（30fps 下每轮 6 帧仅 ~0.2s）
  int height = 480;
  int fps = 30;

  // 从 other 拷贝全部策略字段（device/side 除外）——cam_tune_pair 双机共用一套
  // JSON 参数：读一次文件覆盖到 base，再分发到左右两份配置。
  void take_policy_from(const CamTuneConfig &o) {
    target = o.target; tol = o.tol; hl_max = o.hl_max; max_exp_ms = o.max_exp_ms;
    max_gain = o.max_gain; warmup = o.warmup; frames = o.frames;
    settle_ms = o.settle_ms; max_iter = o.max_iter; budget_ms = o.budget_ms;
    width = o.width; height = o.height; fps = o.fps;
  }
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
