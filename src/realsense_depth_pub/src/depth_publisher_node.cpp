#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <librealsense2/rs.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <cstring>
#include <algorithm>
#include <string>

static std::mutex g_mutex;
static rs2::frameset g_latest_fs;
static bool g_has_frame = false;

static void rs_cb(const rs2::frame& f) {
  if (auto fs = f.as<rs2::frameset>()) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_latest_fs = fs;
    g_has_frame = true;
  }
}

static inline void copyClamp16U(const uint16_t* src, int w, int h, int sstride_px,
                                uint16_t* dst, uint16_t min_u, uint16_t max_u)
{
  for (int y = 0; y < h; ++y) {
    const uint16_t* srow = src + y * sstride_px;
    uint16_t* drow = dst + y * w;
    for (int x = 0; x < w; ++x) {
      uint16_t z = srow[x];
      if (z != 0) {
        if (z < min_u) z = min_u;
        else if (z > max_u) z = max_u;
      }
      drow[x] = z;
    }
  }
}

static inline void chooseCenterCrop(int in_w,int in_h,int out_w,int out_h,
                                    int& x0,int& y0,int& cw,int& ch) {
  const double r_in = double(in_w)/double(in_h);
  const double r_out= double(out_w)/double(out_h);
  if (r_in > r_out) {
    ch = in_h;
    cw = std::max(1, int(std::round(in_h * r_out)));
    x0 = (in_w - cw)/2; y0 = 0;
  } else {
    cw = in_w;
    ch = std::max(1, int(std::round(in_w / r_out)));
    x0 = 0; y0 = (in_h - ch)/2;
  }
  if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
  if (x0+cw > in_w) cw = in_w - x0;
  if (y0+ch > in_h) ch = in_h - y0;
}

static inline void resizeNN16U(const uint16_t* src,int sw,int sh,int sstride_px,
                               uint16_t* dst,int dw,int dh) {
  const double sx = double(sw)/double(dw);
  const double sy = double(sh)/double(dh);
  for (int y=0;y<dh;++y) {
    int yy = std::min(int(std::floor(y*sy)), sh-1);
    const uint16_t* srow = src + yy*sstride_px;
    uint16_t* drow = dst + y*dw;
    for (int x=0;x<dw;++x) {
      int xx = std::min(int(std::floor(x*sx)), sw-1);
      drow[x] = srow[xx];
    }
  }
}

static inline void fillCameraInfo(sensor_msgs::msg::CameraInfo& ci,
                                  int w,int h,double fx,double fy,double cx,double cy) {
  ci.width=w; ci.height=h;
  ci.d.resize(5, 0.0);
  for(double &v:ci.k) v=0.0; ci.k[0]=fx; ci.k[2]=cx; ci.k[4]=fy; ci.k[5]=cy; ci.k[8]=1.0;
  for(double &v:ci.r) v=0.0; ci.r[0]=ci.r[4]=ci.r[8]=1.0;
  for(double &v:ci.p) v=0.0; ci.p[0]=fx; ci.p[2]=cx; ci.p[5]=fy; ci.p[6]=cy; ci.p[10]=1.0;
}

static const char* distortionToStr(rs2_distortion d) {
  switch (d) {
    case RS2_DISTORTION_NONE:                   return "NONE";
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY: return "MODIFIED_BROWN_CONRADY";
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:  return "INVERSE_BROWN_CONRADY";
    case RS2_DISTORTION_FTHETA:                 return "FTHETA";
    case RS2_DISTORTION_BROWN_CONRADY:          return "BROWN_CONRADY";
    default: return "UNKNOWN";
  }
}

static void printIntrinsics(const rs2_intrinsics& K, const char* tag, const rclcpp::Logger& logger) {
  const double HFOV = 2.0 * std::atan( double(K.width)  / (2.0 * K.fx) ) * 180.0 / M_PI;
  const double VFOV = 2.0 * std::atan( double(K.height) / (2.0 * K.fy) ) * 180.0 / M_PI;

  RCLCPP_INFO(logger,
              "[Intrinsics:%s] size=%dx%d  fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f  model=%s  coeffs=[%.6f, %.6f, %.6f, %.6f, %.6f]  ->  HFOV=%.2f deg, VFOV=%.2f deg",
              tag, K.width, K.height, K.fx, K.fy, K.ppx, K.ppy, distortionToStr(K.model),
              K.coeffs[0], K.coeffs[1], K.coeffs[2], K.coeffs[3], K.coeffs[4],
              HFOV, VFOV);
}

template<typename T>
static void declare_and_get(const rclcpp::Node::SharedPtr& node, const std::string& name, T& value) {
  node->declare_parameter<T>(name, value);
  node->get_parameter(name, value);
}

int main(int argc,char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("realsense_depth_publisher");

  std::string serial="";
  int depth_w=848, depth_h=480, depth_fps=30;
  int color_w=848, color_h=480, color_fps=30;
  bool enable_color=true, enable_ir=false, enable_imu=true;
  int publish_rate_hz=30;

  int out_w=87, out_h=58;
  bool publish_full=true;
  float hfov=55.2f, vfov=55.2f;

  int decimation=1; float min_dist=0.25f, max_dist=5.0f;
  float spat_alpha=0.6f, spat_delta=20.0f; int spat_holes=2;
  int temp_holes=4;
  float temp_alpha=0.45f, temp_delta=30.0f; int hole1=1, hole2=2;

  std::string frame_depth="camera_depth_optical_frame";
  std::string frame_color="camera_color_optical_frame";
  std::string topic_depth="/camera/depth/image_raw";
  std::string topic_depth_info="/camera/depth/camera_info";
  std::string topic_small="/camera/depth/image_87x58";
  std::string topic_small_info="/camera/depth/camera_info_87x58";
  std::string topic_color="/camera/color/image_raw";
  std::string topic_ir1="/camera/infra1/image_raw";
  std::string topic_ir2="/camera/infra2/image_raw";
  std::string topic_imu="/camera/imu";

  declare_and_get(node, "serial", serial);
  declare_and_get(node, "depth_w", depth_w);
  declare_and_get(node, "depth_h", depth_h);
  declare_and_get(node, "depth_fps", depth_fps);
  declare_and_get(node, "color_w", color_w);
  declare_and_get(node, "color_h", color_h);
  declare_and_get(node, "color_fps", color_fps);
  declare_and_get(node, "enable_color", enable_color);
  declare_and_get(node, "enable_ir", enable_ir);
  declare_and_get(node, "enable_imu", enable_imu);
  declare_and_get(node, "publish_rate_hz", publish_rate_hz);

  declare_and_get(node, "out_w", out_w);
  declare_and_get(node, "out_h", out_h);
  declare_and_get(node, "hfov", hfov);
  declare_and_get(node, "vfov", vfov);
  declare_and_get(node, "publish_full", publish_full);

  declare_and_get(node, "decimation", decimation);
  declare_and_get(node, "min_dist", min_dist);
  declare_and_get(node, "max_dist", max_dist);
  declare_and_get(node, "spat_alpha", spat_alpha);
  declare_and_get(node, "spat_delta", spat_delta);
  declare_and_get(node, "spat_holes", spat_holes);
  declare_and_get(node, "temp_holes", temp_holes);
  declare_and_get(node, "temp_alpha", temp_alpha);
  declare_and_get(node, "temp_delta", temp_delta);
  declare_and_get(node, "hole1", hole1);
  declare_and_get(node, "hole2", hole2);

  declare_and_get(node, "frame_depth", frame_depth);
  declare_and_get(node, "frame_color", frame_color);
  declare_and_get(node, "topic_depth", topic_depth);
  declare_and_get(node, "topic_depth_info", topic_depth_info);
  declare_and_get(node, "topic_small", topic_small);
  declare_and_get(node, "topic_small_info", topic_small_info);
  declare_and_get(node, "topic_color", topic_color);
  declare_and_get(node, "topic_ir1", topic_ir1);
  declare_and_get(node, "topic_ir2", topic_ir2);
  declare_and_get(node, "topic_imu", topic_imu);

  (void)temp_holes;

  auto pub_depth      = node->create_publisher<sensor_msgs::msg::Image>(topic_depth, 1);
  auto pub_depth_info = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_depth_info, 1);
  auto pub_small      = node->create_publisher<sensor_msgs::msg::Image>(topic_small, 1);
  auto pub_small_info = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_small_info, 1);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir1;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir2;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;

  if (enable_color) pub_color = node->create_publisher<sensor_msgs::msg::Image>(topic_color, 1);
  if (enable_ir) {
    pub_ir1 = node->create_publisher<sensor_msgs::msg::Image>(topic_ir1, 1);
    pub_ir2 = node->create_publisher<sensor_msgs::msg::Image>(topic_ir2, 1);
  }
  if (enable_imu) pub_imu = node->create_publisher<sensor_msgs::msg::Imu>(topic_imu, 100);

  rs2::config cfg;
  if (!serial.empty()) cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_DEPTH, depth_w, depth_h, RS2_FORMAT_Z16, depth_fps);
  if (enable_color)
    cfg.enable_stream(RS2_STREAM_COLOR, color_w, color_h, RS2_FORMAT_BGR8, color_fps);
  if (enable_ir) {
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, depth_w, depth_h, RS2_FORMAT_Y8, depth_fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, depth_w, depth_h, RS2_FORMAT_Y8, depth_fps);
  }
  if (enable_imu) {
    cfg.enable_stream(RS2_STREAM_GYRO , RS2_FORMAT_MOTION_XYZ32F, 400);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
  }

  rs2::pipeline pipe;
  rs2::pipeline_profile profile;
  try {
    profile = pipe.start(cfg, rs_cb);
  } catch (const rs2::error& e) {
    RCLCPP_ERROR(node->get_logger(), "RealSense start failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
  float depth_scale = depth_sensor.get_depth_scale();

  uint16_t min_u = static_cast<uint16_t>(std::lround(min_dist / depth_scale));
  uint16_t max_u = static_cast<uint16_t>(std::lround(max_dist / depth_scale));
  if (max_u == 0) max_u = 1;
  if (max_u < min_u) std::swap(max_u, min_u);

  try {
    for (auto& s : profile.get_device().query_sensors()) {
      if (s.supports(RS2_OPTION_EMITTER_ENABLED)) s.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
      if (s.supports(RS2_OPTION_LASER_POWER)) {
        auto r = s.get_option_range(RS2_OPTION_LASER_POWER);
        s.set_option(RS2_OPTION_LASER_POWER, r.max);
      }
      if (s.supports(RS2_OPTION_VISUAL_PRESET))
        s.set_option(RS2_OPTION_VISUAL_PRESET,
                     float(RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY));
    }
  } catch (...) {}

  rs2::video_stream_profile dprof = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  rs2_intrinsics intr0 = dprof.get_intrinsics();
  printIntrinsics(intr0, "depth", node->get_logger());

  rs2::decimation_filter   f_dec; f_dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, float(decimation));
  rs2::threshold_filter    f_thr; f_thr.set_option(RS2_OPTION_MIN_DISTANCE, min_dist);
                                 f_thr.set_option(RS2_OPTION_MAX_DISTANCE, max_dist);
  rs2::spatial_filter      f_spa; f_spa.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spat_alpha);
                                 f_spa.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spat_delta);
                                 f_spa.set_option(RS2_OPTION_HOLES_FILL, float(spat_holes));
  rs2::temporal_filter     f_tmp;f_tmp.set_option(RS2_OPTION_HOLES_FILL,  6.0);
                                 f_tmp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temp_alpha);
                                 f_tmp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,  temp_delta);
  rs2::hole_filling_filter f_h1;  f_h1.set_option(RS2_OPTION_HOLES_FILL, float(hole1));
  rs2::hole_filling_filter f_h2;  f_h2.set_option(RS2_OPTION_HOLES_FILL, float(hole2));

  rclcpp::Rate rate(publish_rate_hz);

  while (rclcpp::ok()) {
    rs2::frameset fs_local;
    { std::lock_guard<std::mutex> lk(g_mutex);
      if (g_has_frame) { fs_local = g_latest_fs; g_has_frame=false; } }

    if (fs_local) {
      const auto now = node->get_clock()->now();
      const int64_t ns = now.nanoseconds();
      builtin_interfaces::msg::Time stamp;
      stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

      if (auto df0 = fs_local.get_depth_frame()) {
        rs2::frame f = df0;
        if (decimation>1) f = f_dec.process(f);
        // f = f_thr.process(f);
        f = f_spa.process(f);
        f = f_tmp.process(f);
        f = f_h1.process(f);
        f = f_h2.process(f);

        rs2::video_frame vf = f.as<rs2::video_frame>();
        const int w0=vf.get_width(), h0=vf.get_height();
        const int step0 = vf.get_stride_in_bytes();
        const uint8_t* src = (const uint8_t*)vf.get_data();

        if (publish_full) {
          std::vector<uint16_t> full_buf(size_t(w0) * h0);
          const uint16_t* in16 = reinterpret_cast<const uint16_t*>(src);
          const int in_stride_px = step0 / 2;
          copyClamp16U(in16, w0, h0, in_stride_px, full_buf.data(), min_u, max_u);

          sensor_msgs::msg::Image img;
          img.header.stamp=stamp; img.header.frame_id=frame_depth;
          img.width=w0; img.height=h0;
          img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
          img.is_bigendian=false; img.step=w0*2;
          img.data.resize(size_t(img.step)*h0);
          std::memcpy(img.data.data(), full_buf.data(), img.data.size());
          pub_depth->publish(img);

          const double sx0=double(w0)/double(intr0.width);
          const double sy0=double(h0)/double(intr0.height);
          double fx0=intr0.fx*sx0, fy0=intr0.fy*sy0,
                 cx0=intr0.ppx*sx0, cy0=intr0.ppy*sy0;

          double hfov_full = 2.0 * std::atan(double(w0) / (2.0 * fx0)) * 180.0 / M_PI;
          double vfov_full = 2.0 * std::atan(double(h0) / (2.0 * fy0)) * 180.0 / M_PI;

          // RCLCPP_INFO(node->get_logger(),
          //             "FOV(full):  w=%d h=%d  fx=%.2f fy=%.2f  ->  HFOV=%.2f deg, VFOV=%.2f deg",
          //             w0, h0, fx0, fy0, hfov_full, vfov_full);

          sensor_msgs::msg::CameraInfo ci;
          ci.header = img.header;
          fillCameraInfo(ci, w0, h0, fx0, fy0, cx0, cy0);
          pub_depth_info->publish(ci);
        }

        const double HFOV_T_deg = hfov;
        const double VFOV_T_deg = vfov;
        const double HFOV_T = HFOV_T_deg * M_PI / 180.0;
        const double VFOV_T = VFOV_T_deg * M_PI / 180.0;
        const double r_out = double(out_w) / double(out_h);

        const uint16_t* in_ptr = reinterpret_cast<const uint16_t*>(src);
        const int in_stride_px = step0 / 2;

        const double fxF = intr0.fx * (double(w0) / intr0.width);
        const double fyF = intr0.fy * (double(h0) / intr0.height);
        const double cxF = intr0.ppx * (double(w0) / intr0.width);
        const double cyF = intr0.ppy * (double(h0) / intr0.height);

        int cw0 = (int)std::round(2.0 * fxF * std::tan(HFOV_T / 2.0));
        int ch0 = (int)std::round(2.0 * fyF * std::tan(VFOV_T / 2.0));

        auto clamp_roi = [&](int& cw, int& ch){
          cw = std::max(1, std::min(cw, w0));
          ch = std::max(1, std::min(ch, h0));
        };
        auto fov_err = [&](int cw, int ch){
          double hf = 2.0 * std::atan(double(cw) / (2.0 * fxF));
          double vf = 2.0 * std::atan(double(ch) / (2.0 * fyF));
          return std::abs(hf - HFOV_T) + std::abs(vf - VFOV_T);
        };

        int cwA = (int)std::round(ch0 * r_out), chA = ch0; clamp_roi(cwA, chA);
        int cwB = cw0, chB = (int)std::round(cw0 / r_out); clamp_roi(cwB, chB);

        double errA = fov_err(cwA, chA);
        double errB = fov_err(cwB, chB);
        int cw = (errA <= errB ? cwA : cwB);
        int ch = (errA <= errB ? chA : chB);

        int x0 = (w0 - cw) / 2;
        int y0 = (h0 - ch) / 2;

        double HFOV_act = 2.0 * std::atan(double(cw) / (2.0 * fxF)) * 180.0 / M_PI;
        double VFOV_act = 2.0 * std::atan(double(ch) / (2.0 * fyF)) * 180.0 / M_PI;
        // RCLCPP_INFO(node->get_logger(),
        //             "ROI=%dx%d @(%d,%d) -> HFOV=%.2f deg, VFOV=%.2f deg",
        //             cw, ch, x0, y0, HFOV_act, VFOV_act);

        const uint16_t* roi = in_ptr + y0 * in_stride_px + x0;
        std::vector<uint16_t> small_buf(size_t(out_w) * out_h);
        resizeNN16U(roi, cw, ch, in_stride_px, small_buf.data(), out_w, out_h);

        for (size_t i = 0, n = small_buf.size(); i < n; ++i) {
          uint16_t z = small_buf[i];
          if (z != 0) {
            if (z < min_u) z = min_u;
            else if (z > max_u) z = max_u;
            small_buf[i] = z;
          }
        }

        sensor_msgs::msg::Image simg;
        simg.header.stamp=stamp; simg.header.frame_id=frame_depth;
        simg.width=out_w; simg.height=out_h;
        simg.encoding=sensor_msgs::image_encodings::TYPE_16UC1;
        simg.is_bigendian=false; simg.step=out_w*2;
        simg.data.resize(size_t(simg.step)*out_h);
        std::memcpy(simg.data.data(), small_buf.data(), simg.data.size());
        pub_small->publish(simg);

        double fx=intr0.fx*double(w0)/intr0.width;
        double fy=intr0.fy*double(h0)/intr0.height;
        double cx=cxF;
        double cy=cyF;

        cx -= x0; cy -= y0;

        fx *= double(out_w)/double(cw);
        fy *= double(out_h)/double(ch);
        cx *= double(out_w)/double(cw);
        cy *= double(out_h)/double(ch);

        sensor_msgs::msg::CameraInfo cismall;
        cismall.header = simg.header;
        fillCameraInfo(cismall, out_w, out_h, fx, fy, cx, cy);
        pub_small_info->publish(cismall);
        double hfov_small = 2.0 * std::atan(double(out_w) / (2.0 * fx)) * 180.0 / M_PI;
        double vfov_small = 2.0 * std::atan(double(out_h) / (2.0 * fy)) * 180.0 / M_PI;

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                             "FOV(small): w=%d h=%d  fx=%.2f fy=%.2f  ROI=%dx%d@(%d,%d)  ->  HFOV=%.2f deg, VFOV=%.2f deg",
                             out_w, out_h, fx, fy, cw, ch, x0, y0, hfov_small, vfov_small);
      }

      if (enable_color) {
        if (auto cf = fs_local.get_color_frame()) {
          const int w=cf.get_width(), h=cf.get_height();
          const int step = cf.get_stride_in_bytes();
          const uint8_t* src = (const uint8_t*)cf.get_data();
          sensor_msgs::msg::Image img;
          img.header.stamp=stamp; img.header.frame_id=frame_color;
          img.width=w; img.height=h;
          img.encoding=sensor_msgs::image_encodings::BGR8;
          img.is_bigendian=false; img.step=w*3;
          img.data.resize(size_t(img.step)*h);
          for (int r=0;r<h;++r)
            std::memcpy(img.data.data()+size_t(r)*img.step, src+size_t(r)*step, w*3);
          pub_color->publish(img);
        }
      }
      if (enable_ir) {
        if (auto ir1 = fs_local.get_infrared_frame(1)) {
          const int w=ir1.get_width(), h=ir1.get_height();
          const int step=ir1.get_stride_in_bytes();
          const uint8_t* src=(const uint8_t*)ir1.get_data();
          sensor_msgs::msg::Image img;
          img.header.stamp=stamp; img.header.frame_id=frame_depth;
          img.width=w; img.height=h;
          img.encoding=sensor_msgs::image_encodings::MONO8;
          img.is_bigendian=false; img.step=w;
          img.data.resize(size_t(img.step)*h);
          for (int r=0;r<h;++r)
            std::memcpy(img.data.data()+size_t(r)*img.step, src+size_t(r)*step, w);
          pub_ir1->publish(img);
        }
        if (auto ir2 = fs_local.get_infrared_frame(2)) {
          const int w=ir2.get_width(), h=ir2.get_height();
          const int step=ir2.get_stride_in_bytes();
          const uint8_t* src=(const uint8_t*)ir2.get_data();
          sensor_msgs::msg::Image img;
          img.header.stamp=stamp; img.header.frame_id=frame_depth;
          img.width=w; img.height=h;
          img.encoding=sensor_msgs::image_encodings::MONO8;
          img.is_bigendian=false; img.step=w;
          img.data.resize(size_t(img.step)*h);
          for (int r=0;r<h;++r)
            std::memcpy(img.data.data()+size_t(r)*img.step, src+size_t(r)*step, w);
          pub_ir2->publish(img);
        }
      }

      if (enable_imu) {
        for (auto&& f : fs_local) {
          if (auto mf = f.as<rs2::motion_frame>()) {
            sensor_msgs::msg::Imu imu; imu.header.stamp=stamp; imu.header.frame_id=frame_depth;
            auto v = mf.get_motion_data();
            if (mf.get_profile().stream_type()==RS2_STREAM_GYRO) {
              imu.angular_velocity.x=v.x; imu.angular_velocity.y=v.y; imu.angular_velocity.z=v.z;
            } else if (mf.get_profile().stream_type()==RS2_STREAM_ACCEL) {
              imu.linear_acceleration.x=v.x; imu.linear_acceleration.y=v.y; imu.linear_acceleration.z=v.z;
            }
            pub_imu->publish(imu);
          }
        }
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  pipe.stop();
  rclcpp::shutdown();
  return 0;
}
