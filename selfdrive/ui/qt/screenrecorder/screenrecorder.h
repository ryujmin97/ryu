#pragma once

#include <memory>
#include <cstdint>
#include <QPainter>
#include <QPushButton>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

#ifdef WSL2

class ScreenRecoder : public QPushButton {
public:
  ScreenRecoder(QWidget* parent = nullptr) {}
  virtual ~ScreenRecoder() {}

  void update_screen() {}
  void toggle() {}
  void start() {}
  void stop() {}
};

#else

#include "omx_encoder.h"
#include "blocking_queue.h"
#include "selfdrive/ui/ui.h"

class ScreenRecoder : public QPushButton {
  Q_OBJECT

public:
  ScreenRecoder(QWidget* parent = nullptr);
  virtual ~ScreenRecoder();

  void start();
  void stop();
  void toggle();
  void update_screen();

protected:
  void paintEvent(QPaintEvent*) override;

private:
  void applyColor();
  void encoding_thread_func();
  void openEncoder(const char* filename);
  void closeEncoder();
  void start_locked();
  // auto_rollover: 20분 자동 세그먼트 롤오버(update_screen())에서 호출된
  // 경우 true, 사용자의 명시적 정지(toggle/stop)인 경우 false(기본값).
  // clip 추출은 사용자가 실제로 정지 버튼을 눌렀을 때만 실행하기 위한
  // 구분 — 롤오버에서도 clip이 계속 쌓이던 버그 수정 (devnotes
  // FINDINGS.md "[RISK_IDENTIFIED] screenrecord clip ... 20분 자동
  // 세그먼트 롤오버에서도 clip이 반복 생성됨" 대응).
  void stop_locked(bool auto_rollover = false);
  // 정지 시점 기준 마지막 30초를 별도 clip mp4로 추출 (ffmpeg 백그라운드,
  // 메인 파일은 이미 finalize된 상태라 non-blocking, 실패해도 메인 녹화에
  // 영향 없음). 같은 초에 두 번 호출되어 파일명이 충돌하면 stat()으로
  // 감지해 접미사를 붙임(앞 clip을 소리 없이 덮어쓰지 않도록).
  void extract_trailing_clip(const std::string& source_path);

  long long started = 0;
  int src_width = 0;
  int src_height = 0;
  int dst_width = 0;
  int dst_height = 0;

  QColor recording_color;
  int frame = 0;

  std::unique_ptr<OmxEncoder> encoder;
  std::unique_ptr<uint8_t[]> rgb_buffer;
  std::unique_ptr<uint8_t[]> rgb_scale_buffer;

  std::thread encoding_thread;
  BlockingQueue<QImage> image_queue;
  QWidget* rootWidget = nullptr;

  std::atomic<bool> recording{ false };
  std::mutex record_lock;
};

#endif
