// startRecord 前置的相机曝光/增益快速整定 —— 实现见 cam_tuner.hpp 头部说明。
// 策略函数 policy_step 与 adjust_cam_param.py 逐行一致；仅两处工程性简化：
//   1) 控制 ioctl 按 V4L2 CID 直达（等价 v4l2-ctl 的 exposure_time_absolute/gain），
//      无需 Python 端那套跨驱动名字模糊解析；
//   2) 写参数每次直写（Python 跳过未变化项是为省 SSH 往返 ~20ms+，本地 ioctl
//      仅微秒级，重复写无害）。

#include "r3d_ws/cam_tuner.hpp"

#include "r3d_camodom/v4l2_camera.hpp"
#include "r3d_ws/config.hpp"
#include "r3d_ws/logger.hpp"

#include <linux/videodev2.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

// 老内核头文件可能缺该宏（本机/板上 5.10+ 均有，稳妥起见兜底定义）
#ifndef V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE
#define V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE (V4L2_CID_CAMERA_CLASS_BASE + 3)
#endif

namespace r3d_ws {
namespace {

// 与 adjust_cam_param.py 逐项对应的策略常量
constexpr double kStepDownMax = 0.4;    // 单轮乘性调整下限（防过冲震荡）
constexpr double kStepUpMax = 2.5;      // 单轮乘性调整上限
constexpr int kExpUnitPerMs = 10;       // UVC exposure_absolute 单位 = 100µs → 1ms = 10
constexpr double kLightTarget = 128.0;  // post_proc_tool 动态提亮的目标亮度

// 控制策略纯函数 —— 与 adjust_cam_param.py policy_step 逐行一致。
struct StepOut {
  double exp_ms;
  double gain;  // 内部按浮点计算（Python 的 gain*f 也是浮点中间量），写入时才取整
  bool done;    // 已进入死区且高光达标
  bool stuck;   // 需要继续调但已顶到上下限，无法改善
  std::string reason;
};

StepOut policy_step(double mean, double hl, double exp_ms, double gain,
                    const CamTuneConfig &cfg, double exp_min_ms, double gain_min) {
  const double emin = exp_min_ms, emax = cfg.max_exp_ms;
  const double gmin = gain_min, gmax = cfg.max_gain;
  exp_ms = std::min(std::max(exp_ms, emin), emax);
  gain = std::min(std::max(gain, gmin), gmax);

  const bool over = mean > cfg.target + cfg.tol || hl > cfg.hl_max;
  const bool under = mean < cfg.target - cfg.tol;

  char fstr[16];
  if (!over && !under) {
    return {exp_ms, gain, true, false, "进入目标区间"};
  }

  if (over) {
    // 过曝：先乘性压曝光；mean 已达标但仅高光超标时温和压 15%
    const double f = (mean > cfg.target + cfg.tol)
                         ? std::min(std::max(cfg.target / std::max(mean, 1.0), kStepDownMax), 1.0)
                         : 0.85;
    const double new_exp = std::max(exp_ms * f, emin);  // 截到下限
    const double new_gain =
        (new_exp > emin) ? gain : std::max(gain * f, gmin);  // 曝光到底才动增益
    std::snprintf(fstr, sizeof(fstr), "×%.2f", f);
    if (std::fabs(new_exp - exp_ms) > 1e-9 || std::fabs(new_gain - gain) > 1e-9) {
      return {new_exp, new_gain, false, false,
              std::string(new_exp > emin ? "压曝光 " : "曝光到底，压增益 ") + fstr};
    }
    return {new_exp, new_gain, false, true, "曝光/增益均已到底仍偏曝"};
  }

  // 昏暗：先拉曝光到上限（防拖影），再拉增益（防噪点）
  const double f = std::min(cfg.target / std::max(mean, 1.0), kStepUpMax);
  const double new_exp = std::min(exp_ms * f, emax);  // 截到上限
  const double new_gain =
      (new_exp < emax) ? gain : std::min(gain * f, gmax);  // 曝光到顶才动增益
  std::snprintf(fstr, sizeof(fstr), "×%.2f", f);
  if (std::fabs(new_exp - exp_ms) > 1e-9 || std::fabs(new_gain - gain) > 1e-9) {
    return {new_exp, new_gain, false, false,
            std::string(new_exp < emax ? "升曝光 " : "曝光到上限，升增益 ") + fstr};
  }
  return {new_exp, new_gain, false, true, "曝光/增益均已到上限仍偏暗"};
}

// 解码一帧 MJPG 并统计 (mean, ≥250 高光占比)。灰度转换与 PIL convert("L") 同为
// ITU-R 601 权重，统计口径与桌面端工具一致。解码失败返回 false。
bool frame_stats(const std::vector<uint8_t> &mjpg, double &mean, double &hl) {
  try {
    cv::Mat raw(1, static_cast<int>(mjpg.size()), CV_8U,
                const_cast<uint8_t *>(mjpg.data()));
    cv::Mat img = cv::imdecode(raw, cv::IMREAD_COLOR);
    if (img.empty()) return false;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    long sum = 0, hl_cnt = 0;
    const int n = gray.rows * gray.cols;
    const uchar *p = gray.ptr<uchar>();
    for (int i = 0; i < n; ++i) {
      sum += p[i];
      if (p[i] >= 250) ++hl_cnt;
    }
    mean = static_cast<double>(sum) / n;
    hl = static_cast<double>(hl_cnt) / n;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

// 一轮统计：抓 warmup+frames 帧，丢前 warmup 帧旧参数残留，取后 frames 帧平均
// （与 CameraWorker.collect_stats 同口径；grab() 排空到最新帧）。deadline 到或
// 流卡死（连续抓帧失败超限）返回 false。
bool collect_stats(r3d_camodom::V4L2Camera &cam, const CamTuneConfig &cfg,
                   std::chrono::steady_clock::time_point deadline, double &mean_out,
                   double &hl_out) {
  double sum_mean = 0, sum_hl = 0;
  int warm_left = cfg.warmup, stat_left = cfg.frames;
  // 帧级失败预算：坏帧/超时帧跳过重抓，连续超限视作流卡死（grab 内部 poll 1s 超时）
  int fail_budget = cfg.warmup + cfg.frames + 4;
  while (warm_left > 0 || stat_left > 0) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::vector<uint8_t> buf;
    if (!cam.grab(buf)) {
      if (--fail_budget < 0) return false;
      continue;
    }
    if (warm_left > 0) {
      --warm_left;
      continue;
    }
    double m, h;
    if (!frame_stats(buf, m, h)) {  // 坏帧不计入统计，重抓
      if (--fail_budget < 0) return false;
      continue;
    }
    sum_mean += m;
    sum_hl += h;
    --stat_left;
  }
  mean_out = sum_mean / cfg.frames;
  hl_out = sum_hl / cfg.frames;
  return true;
}

std::string fmt1(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", v);
  return std::string(buf);
}

}  // namespace

CamTuneResult cam_tune_one(const CamTuneConfig &cfg) {
  CamTuneResult res;
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + std::chrono::milliseconds(cfg.budget_ms);

  // 640x480 MJPG 小流：30fps 下每轮 6 帧仅 ~0.2s，2s 预算内可跑满 5 轮。
  // auto_exposure=false → open() 即切 Manual（驱动保留 AE 最后值作整定起点）；
  // auto_wb=true 不动白平衡；jpeg_quality=0 跳过质量设置（亮度统计不敏感）。
  r3d_camodom::CamConfig cc;
  cc.device = cfg.device;
  cc.width = cfg.width;
  cc.height = cfg.height;
  cc.fps = cfg.fps;
  cc.auto_exposure = false;
  cc.auto_wb = true;
  cc.jpeg_quality = 0;
  r3d_camodom::V4L2Camera cam(cc);
  if (!cam.open()) {
    log::warn("📷 [" + cfg.side + "] 整定失败：相机打开失败 " + cfg.device +
              "（被占用/不存在？），回退自动曝光");
    return res;  // ok=false —— 析构 release，设备不留占用
  }

  // 控制项范围（等价 parse_controls 的 min/max；ioctl 按 id 直达）。
  // 无曝光控制项 = 不是 UVC 相机（CSI/元数据节点），整定无意义。
  int32_t exp_min = 1, exp_max = 1, gain_min = 0, gain_max = 0;
  if (!cam.query_ctrl_range(V4L2_CID_EXPOSURE_ABSOLUTE, exp_min, exp_max)) {
    log::warn("📷 [" + cfg.side + "] 整定失败：" + cfg.device +
              " 无曝光控制项（CSI 相机？），回退自动曝光");
    return res;
  }
  const bool has_gain = cam.query_ctrl_range(V4L2_CID_GAIN, gain_min, gain_max);
  const double exp_min_ms = std::max(exp_min, 1) / kExpUnitPerMs;

  // to_manual 补充：关 dynamic_framerate（防曝光拉长降帧率拖慢统计）。尽力而为。
  cam.set_ctrl(V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE, 0);

  // 起点 = 切 Manual 后驱动保留的当前值（AE 的最后值），与 Python 流程一致。
  int32_t cur = 0;
  double exp_ms = 5.0, gain = 0.0;
  if (cam.get_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, cur)) exp_ms = cur / kExpUnitPerMs;
  if (has_gain && cam.get_ctrl(V4L2_CID_GAIN, cur)) gain = cur;

  log::info("📁 " + cfg.side + "相机 " + cfg.device + " | 曝光范围 " +
            fmt1(exp_min_ms) + "~" + fmt1(exp_max / kExpUnitPerMs) + "ms | 增益范围 " +
            (has_gain ? (std::to_string(gain_min) + "~" + std::to_string(gain_max))
                      : std::string("无")));
  log::info("初始: exp=" + fmt1(exp_ms) + "ms gain=" + std::to_string((int)std::lround(gain)) +
            " | 目标 mean=" + fmt1(cfg.target) + "±" + fmt1(cfg.tol) + " 高光≤" +
            fmt1(cfg.hl_max * 100) + "%");

  // 写入值夹到控制项范围（与 make_param_setter.set_params 的 clip 一致）
  auto apply_params = [&](double e_ms, double g) {
    int32_t raw = static_cast<int32_t>(std::lround(e_ms * kExpUnitPerMs));
    raw = std::min(std::max(raw, exp_min), exp_max);
    cam.set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, raw);
    if (has_gain) {
      int32_t rg = static_cast<int32_t>(std::lround(g));
      rg = std::min(std::max(rg, gain_min), gain_max);
      cam.set_ctrl(V4L2_CID_GAIN, rg);
    }
  };

  bool converged = false;
  bool budget_out = false;  // 轮数/预算用尽而停（末次参数未经统计复核）
  double mean = 0, hl = 0;
  int rounds_done = 0;  // 完成了统计的轮数（done/stuck 轮计入；未开始的预算轮不计）
  for (int it = 1; it <= cfg.max_iter; ++it) {
    if (std::chrono::steady_clock::now() >= deadline) {
      budget_out = true;
      break;
    }
    if (!collect_stats(cam, cfg, deadline, mean, hl)) {
      log::warn("📷 [" + cfg.side + "] 整定失败：取帧超时/流卡死（轮次 " +
                std::to_string(it) + "），回退自动曝光");
      return res;  // ok=false —— 已切手动但参数未经复核，交给 AE 兜底更稳
    }
    ++rounds_done;
    StepOut st = policy_step(mean, hl, exp_ms, gain, cfg, exp_min_ms, gain_min);
    char line[160];
    std::snprintf(line, sizeof(line), "轮次 %2d: mean=%6.1f 高光=%4.1f%% | exp=%.2fms gain=%-4d → %s",
                  it, mean, hl * 100, exp_ms, (int)std::lround(gain), st.reason.c_str());
    log::info(std::string("📷 [") + cfg.side + "] " + line);
    if (st.done) {
      converged = true;
      break;
    }
    if (st.stuck) break;  // 顶到参数边界，保持尽力参数
    apply_params(st.exp_ms, st.gain);
    exp_ms = st.exp_ms;
    gain = st.gain;
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg.settle_ms));
    if (it == cfg.max_iter) budget_out = true;  // 末轮参数已写入但无下一轮复核
  }

  res.ok = true;  // 相机已切手动且流程跑完：保持整定值（即便未收敛，参数也已朝目标调整）
  res.converged = converged;
  res.mean = mean;
  res.hl = hl;
  res.exp_ms = exp_ms;
  res.gain = static_cast<int>(std::lround(gain));
  res.iters = rounds_done;

  const double factor =
      std::min(std::max(kLightTarget / std::max(mean, 1.0), 1.0), 2.0);
  std::string verdict = converged ? "✅ 收敛" : budget_out ? "⚠️ 轮次/预算用尽，末次参数未复核"
                        : (mean < cfg.target - cfg.tol) ? "⚠️ 仍偏暗（已达参数边界）"
                                                        : "⚠️ 仍偏曝（已达参数边界）";
  const int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count());
  log::info("📷 [" + cfg.side + "] 结果: mean=" + fmt1(mean) + " 高光=" + fmt1(hl * 100) +
            "% | exp=" + fmt1(exp_ms) + "ms gain=" + std::to_string(res.gain) + " " + verdict +
            "（" + std::to_string(ms) + "ms）");
  log::info("📷 [" + cfg.side + "] → 此亮度下 post_proc_tool 动态提亮 factor≈" +
            fmt1(factor) + (factor <= 1.10 ? "（基本无需提亮）" : "（仍会明显提亮）"));
  return res;  // cam 析构 release，设备交还给后续 image_odom_recorder
}

bool cam_tune_pair() {
  CamTuneConfig lc, rc;
  lc.device = config::TUNE_DEV0;
  lc.side = "左";
  rc.device = config::TUNE_DEV2;
  rc.side = "右";

  CamTuneResult lr, rr;
  log::info("📷 相机参数整定开始（≤" + std::to_string(lc.budget_ms) + "ms，双机并行）...");
  std::thread t([&lr, &lc] { lr = cam_tune_one(lc); });
  rr = cam_tune_one(rc);
  t.join();

  const bool both_ok = lr.ok && rr.ok;
  log::info(std::string("📷 相机参数整定结束：") + (both_ok ? "✅ 双机已手动曝光" : "⚠️ 有失败，回退自动曝光") +
            " | 左 " + (lr.ok ? "ok" : "失败") + (lr.converged ? "(收敛)" : "") +
            " 右 " + (rr.ok ? "ok" : "失败") + (rr.converged ? "(收敛)" : ""));
  return both_ok;
}

}  // namespace r3d_ws
