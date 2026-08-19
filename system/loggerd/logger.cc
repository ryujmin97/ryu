#include "system/loggerd/logger.h"

#include <fstream>
#include <map>
#include <vector>
#include <iostream>
#include <sstream>
#include <random>

#include "common/params.h"
#include "common/swaglog.h"
#include "common/version.h"

// ***** log metadata *****
kj::Array<capnp::word> logger_build_init_data() {
  uint64_t wall_time = nanos_since_epoch();

  MessageBuilder msg;
  auto init = msg.initEvent().initInitData();

  init.setWallTimeNanos(wall_time);
  init.setVersion(COMMA_VERSION);
  init.setDirty(!getenv("CLEAN"));
  init.setDeviceType(Hardware::get_device_type());

  // log kernel args
  std::ifstream cmdline_stream("/proc/cmdline");
  std::vector<std::string> kernel_args;
  std::string buf;
  while (cmdline_stream >> buf) {
    kernel_args.push_back(buf);
  }

  auto lkernel_args = init.initKernelArgs(kernel_args.size());
  for (int i=0; i<kernel_args.size(); i++) {
    lkernel_args.set(i, kernel_args[i]);
  }

  init.setKernelVersion(util::read_file("/proc/version"));
  init.setOsVersion(util::read_file("/VERSION"));

  // log params
  Params params(util::getenv("PARAMS_COPY_PATH", ""));
  std::map<std::string, std::string> params_map = params.readAll();

  init.setGitCommit(params_map["GitCommit"]);
  init.setGitCommitDate(params_map["GitCommitDate"]);
  init.setGitBranch(params_map["GitBranch"]);
  init.setGitRemote(params_map["GitRemote"]);
  init.setPassive(false);
  init.setDongleId(params_map["DongleId"]);

  // for prebuilt branches
  init.setGitSrcCommit(util::read_file("../../git_src_commit"));
  init.setGitSrcCommitDate(util::read_file("../../git_src_commit_date"));

  auto lparams = init.initParams().initEntries(params_map.size());
  int j = 0;
  for (auto& [key, value] : params_map) {
    auto lentry = lparams[j];
    lentry.setKey(key);
    if ( !(params.getKeyType(key) & DONT_LOG) ) {
      lentry.setValue(capnp::Data::Reader((const kj::byte*)value.data(), value.size()));
    }
    j++;
  }

  // log commands
  std::vector<std::string> log_commands = {
    "df -h",  // usage for all filesystems
  };

  auto hw_logs = Hardware::get_init_logs();

  auto commands = init.initCommands().initEntries(log_commands.size() + hw_logs.size());
  for (int i = 0; i < log_commands.size(); i++) {
    auto lentry = commands[i];

    lentry.setKey(log_commands[i]);

    const std::string result = util::check_output(log_commands[i]);
    lentry.setValue(capnp::Data::Reader((const kj::byte*)result.data(), result.size()));
  }

  int i = log_commands.size();
  for (auto &[key, value] : hw_logs) {
    auto lentry = commands[i];
    lentry.setKey(key);
    lentry.setValue(capnp::Data::Reader((const kj::byte*)value.data(), value.size()));
    i++;
  }

  return capnp::messageToFlatArray(msg);
}

std::string logger_get_identifier(std::string key) {
  // a log identifier is a 32 bit counter, plus a 10 character unique ID.
  // e.g. 000001a3--c20ba54385

  Params params;
  uint32_t cnt;
  try {
    cnt = std::stoul(params.get(key));
  } catch (std::exception &e) {
    cnt = 0;
  }
  params.put(key, std::to_string(cnt + 1));

  std::stringstream ss;
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> dist(0, 15);
  for (int i = 0; i < 10; ++i) {
    ss << std::hex << dist(mt);
  }

  return util::string_format("%08x--%s", cnt, ss.str().c_str());
}

std::string zstd_decompress(const std::string &in) {
  ZSTD_DCtx *dctx = ZSTD_createDCtx();
  assert(dctx != nullptr);

  // Initialize input and output buffers
  ZSTD_inBuffer input = {in.data(), in.size(), 0};

  // Estimate and reserve memory for decompressed data
  size_t estimatedDecompressedSize = ZSTD_getFrameContentSize(in.data(), in.size());
  if (estimatedDecompressedSize == ZSTD_CONTENTSIZE_ERROR || estimatedDecompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
    estimatedDecompressedSize = in.size() * 2;  // Use a fallback size
  }

  std::string decompressedData;
  decompressedData.reserve(estimatedDecompressedSize);

  const size_t bufferSize = ZSTD_DStreamOutSize();  // Recommended output buffer size
  std::string outputBuffer(bufferSize, '\0');

  while (input.pos < input.size) {
    ZSTD_outBuffer output = {outputBuffer.data(), bufferSize, 0};

    size_t result = ZSTD_decompressStream(dctx, &output, &input);
    if (ZSTD_isError(result)) {
      break;
    }

    decompressedData.append(outputBuffer.data(), output.pos);
  }

  ZSTD_freeDCtx(dctx);
  decompressedData.shrink_to_fit();
  return decompressedData;
}


static void log_sentinel(LoggerState *log, SentinelType type, int exit_signal = 0) {
  MessageBuilder msg;
  auto sen = msg.initEvent().initSentinel();
  sen.setType(type);
  sen.setSignal(exit_signal);
  log->write(msg.toBytes(), true);
}

// 라우트당 최대 세그먼트 개수. 이 개수를 채우면(21번째 세그먼트부터)
// 새 라우트를 만들어 이어서 기록한다.
// (2026-08-20: carrotweb 로그탭에서 라우트 하나가 너무 길어지는 것을
// 방지하기 위해 40 -> 20으로 축소. 세그먼트 1개는 1분이므로 라우트당
// 최대 길이가 약 40분 -> 20분으로 줄어든다.)
constexpr int MAX_SEGMENTS_PER_ROUTE = 20;

LoggerState::LoggerState(const std::string &log_root) {
  log_root_dir = log_root;
  route_name = logger_get_identifier("RouteCount");
  route_path = log_root_dir + "/" + route_name;
  init_data = logger_build_init_data();
}

LoggerState::~LoggerState() {
  if (rlog) {
    log_sentinel(this, SentinelType::END_OF_ROUTE, exit_signal);
    std::remove(lock_file.c_str());
  }
}

bool LoggerState::next() {
  // route_part(현재 라우트 내 세그먼트 인덱스)가 MAX_SEGMENTS_PER_ROUTE에
  // 도달하면 이번에 닫는 세그먼트가 라우트의 마지막 세그먼트다. 그 경우
  // END_OF_SEGMENT가 아니라 END_OF_ROUTE로 마무리해야 라우트 하나가
  // 온전한 rlog/qlog 시퀀스(START_OF_ROUTE ~ END_OF_ROUTE)를 갖는다.
  bool route_rotating = (route_part + 1) >= MAX_SEGMENTS_PER_ROUTE;

  if (rlog) {
    log_sentinel(this, route_rotating ? SentinelType::END_OF_ROUTE : SentinelType::END_OF_SEGMENT);
    std::remove(lock_file.c_str());
  }

  // part(전역 세그먼트 카운터, segment())는 loggerd.cc에서 encoderd가
  // 보내는 절대 세그먼트 번호와의 동기화에 쓰이므로 라우트가 바뀌어도
  // 리셋하지 않고 계속 증가시킨다. 폴더명과 START_OF_ROUTE 판단에는
  // route_part(라우트 내 세그먼트 인덱스)만 쓴다.
  ++part;
  ++route_part;
  if (route_rotating) {
    route_name = logger_get_identifier("RouteCount");
    route_path = log_root_dir + "/" + route_name;
    route_part = 0;
    Params().put("CurrentRoute", route_name);
  }

  segment_path = route_path + "--" + std::to_string(route_part);
  bool ret = util::create_directories(segment_path, 0775);
  assert(ret == true);

  lock_file = segment_path + "/rlog.lock";
  std::ofstream{lock_file};

  rlog.reset(new ZstdFileWriter(segment_path + "/rlog.zst", LOG_COMPRESSION_LEVEL));
  qlog.reset(new ZstdFileWriter(segment_path + "/qlog.zst", LOG_COMPRESSION_LEVEL));

  // log init data & sentinel type.
  write(init_data.asBytes(), true);
  log_sentinel(this, route_part > 0 ? SentinelType::START_OF_SEGMENT : SentinelType::START_OF_ROUTE);
  return true;
}

void LoggerState::write(uint8_t* data, size_t size, bool in_qlog) {
  rlog->write(data, size);
  if (in_qlog) qlog->write(data, size);
}
