#include "camera_frame_stream.h"

#include <string>
#include <unistd.h>
#include <vector>
#include <string.h>

#include <czmq.h>
#include <libyuv.h>
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "buffering.h"

extern "C" {
#include <libavcodec/avcodec.h>
}

extern volatile int do_exit;

#define FRAME_WIDTH 1164
#define FRAME_HEIGHT 874

namespace {
void camera_open(CameraState *s, VisionBuf *camera_bufs, bool rear) {
  assert(camera_bufs);
  s->camera_bufs = camera_bufs;
}

void camera_close(CameraState *s) {
  tbuffer_stop(&s->camera_tb);
}

void camera_release_buffer(void *cookie, int buf_idx) {
  CameraState *s = static_cast<CameraState *>(cookie);
}

void camera_init(CameraState *s, int camera_id, unsigned int fps) {
  assert(camera_id < ARRAYSIZE(cameras_supported));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);

  s->frame_size = s->ci.frame_height * s->ci.frame_stride;
  s->fps = fps;

  tbuffer_init2(&s->camera_tb, FRAME_BUF_COUNT, "frame", camera_release_buffer, s);
}

void run_frame_stream(DualCameraState *s) {
  int err;
  zsock_t *recorder_sock = zsock_new_sub(">tcp://127.0.0.1:8002", "");
  assert(recorder_sock);
  void *recorder_sock_raw = zsock_resolve(recorder_sock);

  CameraState *const rear_camera = &s->rear;
  auto *tb = &rear_camera->camera_tb;

  while (!do_exit) {
    zmq_msg_t msg;
    err = zmq_msg_init(&msg);
    assert(err == 0);

    err = zmq_msg_recv(&msg, recorder_sock_raw, 0);
    if(err == -1)
	    break;

    // make copy due to alignment issues, will be freed on out of scope
    size_t len = zmq_msg_size(&msg);
    auto amsg = kj::heapArray<capnp::word>((len / sizeof(capnp::word)) + 1);
    memcpy(amsg.begin(), (const uint8_t*)zmq_msg_data(&msg), len);

    capnp::FlatArrayMessageReader cmsg(amsg);
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
    auto frame = event.getFrame();

    const int buf_idx = tbuffer_select(tb);
    rear_camera->camera_bufs_metadata[buf_idx] = {
      .frame_id = frame.getFrameId(),
      .timestamp_eof = frame.getTimestampEof(),
      .frame_length = static_cast<unsigned>(frame.getFrameLength()),
      .integ_lines = static_cast<unsigned>(frame.getIntegLines()),
      .global_gain = static_cast<unsigned>(frame.getGlobalGain()),
    };

    cl_command_queue q = rear_camera->camera_bufs[buf_idx].copy_q;
    cl_mem yuv_cl = rear_camera->camera_bufs[buf_idx].buf_cl;
    cl_event map_event;
    void *yuv_buf = (void *)clEnqueueMapBuffer(q, yuv_cl, CL_TRUE,
                                                CL_MAP_WRITE, 0, frame.getImage().size(),
                                                0, NULL, &map_event, &err);
    assert(err == 0);
    clWaitForEvents(1, &map_event);
    clReleaseEvent(map_event);
    memcpy(yuv_buf, frame.getImage().begin(), frame.getImage().size());

    clEnqueueUnmapMemObject(q, yuv_cl, yuv_buf, 0, NULL, &map_event);
    clWaitForEvents(1, &map_event);
    clReleaseEvent(map_event);
    tbuffer_dispatch(tb, buf_idx);

    err = zmq_msg_close(&msg);
    assert(err == 0);
  }
  zsock_destroy(&recorder_sock);
}

}  // namespace

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  [CAMERA_ID_IMX298] = {
      .frame_width = FRAME_WIDTH,
      .frame_height = FRAME_HEIGHT,
      .frame_stride = FRAME_WIDTH*3,
      .bayer = false,
      .bayer_flip = false,
  },
  [CAMERA_ID_OV8865] = {
    .frame_width = 1632,
    .frame_height = 1224,
    .frame_stride = 2040, // seems right
    .bayer = false,
    .bayer_flip = 3,
    .hdr = false
  },
};

void cameras_init(DualCameraState *s) {
  memset(s, 0, sizeof(*s));

  camera_init(&s->rear, CAMERA_ID_IMX298, 20);
  s->rear.transform = (mat3){{
    1.0,  0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0,  0.0, 1.0,
  }};

  camera_init(&s->front, CAMERA_ID_OV8865, 10);
  s->front.transform = (mat3){{
    1.0,  0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0,  0.0, 1.0,
  }};
}

void camera_autoexposure(CameraState *s, float grey_frac) {}

void cameras_open(DualCameraState *s, VisionBuf *camera_bufs_rear,
                  VisionBuf *camera_bufs_focus, VisionBuf *camera_bufs_stats,
                  VisionBuf *camera_bufs_front) {
  assert(camera_bufs_rear);
  assert(camera_bufs_front);
  int err;

  // LOG("*** open front ***");
  camera_open(&s->front, camera_bufs_front, false);

  // LOG("*** open rear ***");
  camera_open(&s->rear, camera_bufs_rear, true);
}

void cameras_close(DualCameraState *s) {
  camera_close(&s->rear);
}

void cameras_run(DualCameraState *s) {
  set_thread_name("frame_streaming");
  run_frame_stream(s);
  cameras_close(s);
}
