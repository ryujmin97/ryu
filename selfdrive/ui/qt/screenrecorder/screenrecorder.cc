#include <CL/cl.h>
#include <algorithm>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <atomic>
#include <mutex>
#include <string>

#include <QProcess>

#include "libyuv.h"
#include "common/clutil.h"

#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"
#include "system/hardware/hw.h"

static long long milliseconds(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (((long long)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

ScreenRecoder::ScreenRecoder(QWidget* parent) : QPushButton(parent), image_queue(30) {
  recording.store(false);
  started = 0;
  frame = 0;

  const int size = 190;
  setFixedSize(size, size);
  setFocusPolicy(Qt::NoFocus);

  QObject::connect(this, &QPushButton::clicked, [=]() { toggle(); });
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    if (offroad) {
      stop();
    }
    });

  std::string path = "/data/media/0/videos";
  src_width = 2160;
  src_height = 1080;

  // 720p(2Mbps)는 화질 대비 용량이 큰 편이라는 피드백 -> 540p/1.2Mbps로
  // 낮춰 clip 용량을 줄인다(30초 clip 기준 대략 화소수 56%(720p->540p)
  // x 비트레이트 60%(2Mbps->1.2Mbps) ≈ 1/3 수준으로 예상). 화면 텍스트
  // 가독성이 실사용에서 부족하면 dst_height/bitrate만 다시 올리면 됨.
  dst_height = 540;
  dst_width = src_width * dst_height / src_height;
  if (dst_width % 2 != 0)
    dst_width += 1;

  rgb_buffer = std::make_unique<uint8_t[]>(src_width * src_height * 4);
  rgb_scale_buffer = std::make_unique<uint8_t[]>(dst_width * dst_height * 4);
  encoder = std::make_unique<OmxEncoder>(path.c_str(), dst_width, dst_height, UI_FREQ, 6 * 1024 * 1024 / 5, false, false);
}

ScreenRecoder::~ScreenRecoder() {
  stop();
}

void ScreenRecoder::applyColor() {
  if (frame % (UI_FREQ / 2) == 0) {
    if (frame % UI_FREQ < (UI_FREQ / 2))
      recording_color = QColor::fromRgbF(1, 0, 0, 0.6);
    else
      recording_color = QColor::fromRgbF(0, 0, 0, 0.3);

    update();
  }
}

void ScreenRecoder::paintEvent(QPaintEvent* event) {
  QRect r = QRect(0, 0, width(), height());
  r -= QMargins(5, 5, 5, 5);

  QPainter p(this);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setPen(QPen(QColor::fromRgbF(1, 1, 1, 0.4), 10, Qt::SolidLine, Qt::FlatCap));
  p.setBrush(QBrush(QColor::fromRgbF(0, 0, 0, 0)));

  r -= QMargins(40, 40, 40, 40);
  p.setPen(Qt::NoPen);

  QColor bg = recording.load() ? recording_color : QColor::fromRgbF(0, 0, 0, 0.3);
  p.setBrush(QBrush(bg));
  p.drawEllipse(r);
}

void ScreenRecoder::openEncoder(const char* filename) {
  if (encoder) {
    encoder->encoder_open(filename);
  }
}

void ScreenRecoder::closeEncoder() {
  if (encoder) {
    encoder->encoder_close();
  }
}

void ScreenRecoder::toggle() {
  std::lock_guard<std::mutex> lk(record_lock);
  if (!recording.load()) {
    start_locked();
  }
  else {
    stop_locked();
  }
}

void ScreenRecoder::start() {
  std::lock_guard<std::mutex> lk(record_lock);
  start_locked();
}

void ScreenRecoder::stop() {
  std::lock_guard<std::mutex> lk(record_lock);
  stop_locked();
}

void ScreenRecoder::start_locked() {
  if (recording.load())
    return;

  if (encoding_thread.joinable()) {
    encoding_thread.join();
  }

  char filename[64];
  time_t t = time(NULL);
  struct tm tm_buf;
  localtime_r(&t, &tm_buf);

  snprintf(filename, sizeof(filename), "%04d%02d%02d-%02d%02d%02d.mp4",
    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

  frame = 0;

  QWidget* widget = this;
  while (widget->parentWidget() != nullptr)
    widget = widget->parentWidget();

  rootWidget = widget;

  image_queue.clear();
  openEncoder(filename);
  recording.store(true);

  encoding_thread = std::thread([this] { encoding_thread_func(); });

  started = milliseconds();
  update();
}

void ScreenRecoder::stop_locked(bool auto_rollover) {
  if (!recording.load()) {
    if (encoding_thread.joinable()) {
      encoding_thread.join();
    }
    return;
  }

  recording.store(false);
  update();

  if (encoding_thread.joinable()) {
    encoding_thread.join();
  }

  image_queue.clear();

  std::string finished_path;
  if (encoder) {
    finished_path = encoder->get_last_video_path();
  }
  closeEncoder();

  // 메인 파일은 이미 finalize됐으므로, 정지 시점을 기준으로 마지막 30초를
  // 별도 clip으로 잘라낸다. 녹화 길이가 30초 미만이어도 (전체 길이만큼)
  // 항상 생성 — clip 파일을 목록에서 구분하기 쉽게 하기 위함.
  // 단, 20분 자동 세그먼트 롤오버(auto_rollover=true)로 인한 정지는
  // 사용자가 정지 버튼을 누른 게 아니므로 clip을 만들지 않는다 —
  // 그렇지 않으면 화면녹화를 계속 켜둔 채 장시간 주행 시 20분마다
  // clip이 무한히 쌓이게 된다.
  // extract_trailing_clip() 내부의 QProcess::startDetached("ffmpeg", ...)는
  // posix_spawn/vfork 기반이라 exec()가 완료될 때까지 "호출한 스레드"를
  // 블로킹한다. stop_locked()는 정지 버튼 클릭 시 UI(Qt) 메인 스레드에서
  // 동기 실행되므로, 여기서 직접 호출하면 ffmpeg 바이너리 exec가 느릴 때
  // (특히 방금 큰 mp4를 다 쓴 직후 스토리지가 바쁜 상태) UI 메인 스레드가
  // 수 초간 멈춰 manager의 ui watchdog(5s, watchdog_kick()이 UI 메인
  // 스레드의 QTimer에서만 호출됨)를 넘겨 ui가 SIGKILL당하고 재시작되는
  // 사고로 이어진다(실차 swaglog로 확인: "Watchdog timeout for ui
  // (exitcode None) restarting" — SIGSEGV가 아니라 순수 응답지연).
  // 별도 스레드로 완전히 분리해 UI 메인 스레드는 즉시 반환하도록 한다.
  if (!auto_rollover && !finished_path.empty()) {
    std::thread([this, finished_path]() {
      extract_trailing_clip(finished_path);
    }).detach();
  }
}

void ScreenRecoder::extract_trailing_clip(const std::string& source_path) {
  time_t t = time(NULL);
  struct tm tm_buf;
  localtime_r(&t, &tm_buf);

  // 요청 포맷: YYMMDD_HHMMSS (예: 2026-08-20 13:23:03 -> 260820_132303)
  char ts[16];
  snprintf(ts, sizeof(ts), "%02d%02d%02d_%02d%02d%02d",
    (tm_buf.tm_year + 1900) % 100, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

  // 메인 녹화와 같은 폴더에 저장 -> carrotweb 로그탭 화면녹화 목록에
  // 자동으로 같이 뜸. 파일명은 타임스탬프 + "_clip" 접미사.
  //
  // 타임스탬프 해상도는 의도적으로 초 단위 유지(분 단위로 낮추면 버킷이
  // 60배 커져 충돌 확률이 오히려 늘고, clip의 목적 자체가 "이벤트 발생
  // 시각과 가장 가까운 세그먼트를 초 단위로 찾기"라 정밀도가 필요함).
  // 대신 같은 초에 정지가 두 번 겹치는 드문 경우(토글 연타 등)에 앞
  // clip을 -y로 소리 없이 덮어쓰지 않도록, 대상 경로가 이미 있으면
  // "_clip_2", "_clip_3", ... 접미사를 붙인다. 정상 케이스(충돌 없음)는
  // 지금까지와 동일하게 "_clip.mp4" 그대로 나간다.
  size_t last_slash = source_path.find_last_of('/');
  std::string dir = (last_slash == std::string::npos) ? "." : source_path.substr(0, last_slash);
  std::string clip_path = dir + "/" + std::string(ts) + "_clip.mp4";

  struct stat st;
  int suffix = 2;
  while (stat(clip_path.c_str(), &st) == 0) {
    clip_path = dir + "/" + std::string(ts) + "_clip_" + std::to_string(suffix++) + ".mp4";
  }

  // stream copy(-c copy, 재인코딩 없음)라 빠르고 화질 손실 없음. -sseof를
  // -i보다 앞에 둬서(입력 시크) 가장 가까운 키프레임 기준으로 빠르게
  // seek -> 실제 클립 길이는 약 30.0~30.8초(키프레임 간격만큼 오차).
  // 원본 길이가 30초 미만이면 ffmpeg가 처음부터 잘라줌(요청대로 항상 생성).
  QStringList args;
  args << "-y"
       << "-sseof" << "-30"
       << "-i" << QString::fromStdString(source_path)
       << "-c" << "copy"
       << QString::fromStdString(clip_path);

  // UI 스레드를 막지 않도록 완전히 분리된 프로세스로 실행(fire-and-forget).
  // 실패해도(예: ffmpeg 없음, 원본 파일 문제) 메인 녹화 파일에는 영향 없음.
  QProcess::startDetached("ffmpeg", args);
}

void ScreenRecoder::encoding_thread_func() {
  uint64_t start_time = nanos_since_boot() - 1;

  while (recording.load()) {
    QImage popImage;
    if (!image_queue.pop_wait_for(popImage, std::chrono::milliseconds(10))) {
      continue;
    }

    if (!recording.load()) {
      break;
    }

    QImage image = popImage.convertToFormat(QImage::Format_RGBA8888);

    try {
      libyuv::ARGBScale(image.bits(), image.width() * 4,
        image.width(), image.height(),
        rgb_scale_buffer.get(), dst_width * 4,
        dst_width, dst_height,
        libyuv::kFilterLinear);

      if (recording.load() && encoder) {
        encoder->encode_frame_rgba(
          rgb_scale_buffer.get(),
          dst_width,
          dst_height,
          ((uint64_t)nanos_since_boot() - start_time)
        );
      }
    }
    catch (...) {
      printf("Encoding failed, skipping frame\n");
      continue;
    }
  }
}

void ScreenRecoder::update_screen() {
  bool need_restart = false;

  if (recording.load()) {
    if (milliseconds() - started > 1000 * 60 * 20) {
      need_restart = true;
    }
    else {
      applyColor();

      if (rootWidget != nullptr && recording.load()) {
        QPixmap pixmap = rootWidget->grab();
        if (recording.load()) {
          image_queue.push(pixmap.toImage());
        }
      }
    }
  }

  if (need_restart) {
    std::lock_guard<std::mutex> lk(record_lock);
    stop_locked(/*auto_rollover=*/true);
    start_locked();
    return;
  }

  frame++;
}
