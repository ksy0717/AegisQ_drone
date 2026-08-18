// 실행 설정. 전부 환경변수로 주입되며 기본값은 systemd 유닛에 있다.
#pragma once

#include <string>
#include <vector>

namespace dc {

struct Config {
    // 카메라 / 파이프라인
    int src_w = 1280;
    int src_h = 720;
    int net = 640;                  // 모델 입력 한 변
    int infer_w = 640;              // ISP 2nd 스트림 크기 (원본과 같은 종횡비)
    int infer_h = 360;
    int disp_w = 800;               // DSI 패널
    int disp_h = 480;
    bool dual_stream = true;        // ISP 2nd 스트림 사용 (실패 시 tee 로 폴백)

    // 추론
    std::string hef = "/home/sky/models/y5s_0223_6.hef";
    int num_classes = 2;
    float score_th = 0.35f;
    std::vector<std::string> class_names{"class0", "class1"};
    bool hailo_requested = true;
    double hailo_wait_secs = 15.0;  // 부팅 시 /dev/hailo* 대기 상한

    // 출력
    bool enable_display = true;
    bool enable_rtsp = true;
    std::string rtsp_url = "rtsp://127.0.0.1:8554/cam0";
    int rtsp_bitrate = 3000;        // kbps
    int rtsp_fps = 30;              // GOP 길이 산정용
    bool enable_whip = false;
    std::string whip_url;
    std::string whip_token;
    int whip_retry_secs = 15;

    double run_secs = 0.0;          // 0 = 무한 (systemd 용)
    double report_secs = 10.0;

    static Config from_env();
    std::string validation_error() const;
    void print() const;
};

}  // namespace dc
