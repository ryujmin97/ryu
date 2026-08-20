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
  void stop_locked();
  // 정지 시점 기준 마지막 1분을 별도 clip mp4로 추출 (ffmpeg 백그라운드,
  // 메인 파일은 이미 finalize된 상태라 non-blocking, 실패해도 메인 녹화에
  // 영향 없음).
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
