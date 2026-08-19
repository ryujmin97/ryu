#include "catch2/catch.hpp"
#include "system/loggerd/logger.h"

typedef cereal::Sentinel::SentinelType SentinelType;

void verify_segment(const std::string &route_path, int segment, int max_segment, int required_event_cnt, int end_of_route_signal = 0) {
  const std::string segment_path = route_path + "--" + std::to_string(segment);
  SentinelType begin_sentinel = segment == 0 ? SentinelType::START_OF_ROUTE : SentinelType::START_OF_SEGMENT;
  SentinelType end_sentinel = segment == max_segment - 1 ? SentinelType::END_OF_ROUTE : SentinelType::END_OF_SEGMENT;

  REQUIRE(!util::file_exists(segment_path + "/rlog.lock"));
  for (const char *fn : {"/rlog.zst", "/qlog.zst"}) {
    const std::string log_file = segment_path + fn;
    std::string log = util::read_file(log_file);
    REQUIRE(!log.empty());
    std::string decompressed_log = zstd_decompress(log);
    int event_cnt = 0, i = 0;
    kj::ArrayPtr<const capnp::word> words((capnp::word *)decompressed_log.data(), decompressed_log.size() / sizeof(capnp::word));
    while (words.size() > 0) {
      try {
        capnp::FlatArrayMessageReader reader(words);
        auto event = reader.getRoot<cereal::Event>();
        words = kj::arrayPtr(reader.getEnd(), words.end());
        if (i == 0) {
          REQUIRE(event.which() == cereal::Event::INIT_DATA);
        } else if (i == 1) {
          REQUIRE(event.which() == cereal::Event::SENTINEL);
          REQUIRE(event.getSentinel().getType() == begin_sentinel);
          REQUIRE(event.getSentinel().getSignal() == 0);
        } else if (words.size() > 0) {
          REQUIRE(event.which() == cereal::Event::CLOCKS);
          ++event_cnt;
        } else {
          // the last event must be SENTINEL
          REQUIRE(event.which() == cereal::Event::SENTINEL);
          REQUIRE(event.getSentinel().getType() == end_sentinel);
          // END_OF_ROUTE는 두 가지 경우에 발생한다: (1) 실제 프로세스 종료
          // (exit_signal, 보통 1) (2) MAX_SEGMENTS_PER_ROUTE 도달로 인한
          // 중간 라우트 회전(진짜 종료가 아니므로 signal=0). 어느 쪽인지는
          // 호출자가 end_of_route_signal로 알려준다.
          REQUIRE(event.getSentinel().getSignal() == (end_sentinel == SentinelType::END_OF_ROUTE ? end_of_route_signal : 0));
        }
        ++i;
      } catch (const kj::Exception &ex) {
        INFO("failed parse " << i << " exception :" << ex.getDescription());
        REQUIRE(0);
        break;
      }
    }
    REQUIRE(event_cnt == required_event_cnt);
  }
}

void write_msg(LoggerState *logger) {
  MessageBuilder msg;
  msg.initEvent().initClocks();
  logger->write(msg.toBytes(), true);
}

TEST_CASE("logger") {
  const int segment_cnt = 100;
  const std::string log_root = "/tmp/test_logger";
  system(("rm " + log_root + " -rf").c_str());

  // MAX_SEGMENTS_PER_ROUTE(20)를 넘기면 새 라우트로 넘어가므로, 100개
  // 세그먼트를 계속 next()하면 라우트가 여러 개(20+20+20+20+20) 생긴다.
  // (아래 로직은 실제 라우트 경계를 동적으로 추적하므로 상수 값이 바뀌어도
  // 그대로 통과한다 -- 이 숫자는 설명용 주석일 뿐 하드코딩된 검증이 아님.)
  std::vector<std::string> route_names;
  std::vector<int> route_segment_counts;
  {
    LoggerState logger(log_root);
    std::string current_route = logger.routeName();
    route_names.push_back(current_route);
    int in_route = 0;
    for (int i = 0; i < segment_cnt; ++i) {
      REQUIRE(logger.next());
      REQUIRE(util::file_exists(logger.segmentPath() + "/rlog.lock"));
      // segment()(전역 카운터)는 라우트 회전과 무관하게 계속 증가해야
      // encoderd와의 동기화가 깨지지 않는다.
      REQUIRE(logger.segment() == i);
      if (logger.routeName() != current_route) {
        route_segment_counts.push_back(in_route);
        current_route = logger.routeName();
        route_names.push_back(current_route);
        in_route = 0;
      }
      REQUIRE(logger.routeSegment() == in_route);
      ++in_route;
      write_msg(&logger);
    }
    route_segment_counts.push_back(in_route);
    logger.setExitSignal(1);
  }

  int total_verified = 0;
  for (size_t r = 0; r < route_names.size(); ++r) {
    int cnt = route_segment_counts[r];
    // 마지막 라우트만 실제 프로세스 종료(exit_signal=1)로 끝난다.
    // 그 앞의 라우트들은 세그먼트 개수 제한으로 인한 회전이라 signal=0.
    int end_signal = (r == route_names.size() - 1) ? 1 : 0;
    for (int i = 0; i < cnt; ++i) {
      verify_segment(log_root + "/" + route_names[r], i, cnt, 1, end_signal);
    }
    total_verified += cnt;
  }
  REQUIRE(total_verified == segment_cnt);
}
