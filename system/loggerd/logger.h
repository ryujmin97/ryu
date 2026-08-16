#pragma once

#include <cassert>
#include <memory>
#include <string>

#include "cereal/messaging/messaging.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "system/loggerd/zstd_writer.h"

constexpr int LOG_COMPRESSION_LEVEL = 10;

typedef cereal::Sentinel::SentinelType SentinelType;

class LoggerState {
public:
  LoggerState(const std::string& log_root = Path::log_root());
  ~LoggerState();
  bool next();
  void write(uint8_t* data, size_t size, bool in_qlog);
  inline int segment() const { return part; }
  // 현재 라우트 안에서 몇 번째 세그먼트인지 (라우트가 바뀌면 0으로 리셋).
  // segment()(=part)는 encoderd 동기화(loggerd.cc의 offset_segment_num
  // 비교)에 쓰이므로 라우트 회전과 무관하게 계속 증가해야 하고, 폴더명/
  // START_OF_ROUTE 판단에는 이 값을 대신 쓴다.
  inline int routeSegment() const { return route_part; }
  inline const std::string& segmentPath() const { return segment_path; }
  inline const std::string& routeName() const { return route_name; }
  inline void write(kj::ArrayPtr<kj::byte> bytes, bool in_qlog) { write(bytes.begin(), bytes.size(), in_qlog); }
  inline void setExitSignal(int signal) { exit_signal = signal; }

protected:
  int part = -1, route_part = -1, exit_signal = 0;
  std::string log_root_dir, route_path, route_name, segment_path, lock_file;
  kj::Array<capnp::word> init_data;
  std::unique_ptr<ZstdFileWriter> rlog, qlog;
};

kj::Array<capnp::word> logger_build_init_data();
std::string logger_get_identifier(std::string key);
std::string zstd_decompress(const std::string &in);
