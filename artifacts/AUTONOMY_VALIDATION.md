# Kiểm chứng Nav2 tự động + YOLO — 20/10/2026

Sinh viên: **Tô Văn Chúc — 23020727**. Ubuntu 22.04.5, ROS 2 Humble,
Gazebo Fortress 6, TurtleBot 4 Lite, maze/map tuần trước. ROS domain 43 và Cyclone DDS;
kiểm thử action riêng ở domain 44, không can thiệp simulator.

## Build và kiểm thử

- Build ament_cmake: đạt (`autonomy_build.txt`).
- Python/YAML/XML kiểm tra cú pháp: đạt.
- ROS action integration: 5/5 đạt (`autonomy_action_tests.txt`). Kiểm tra tọa độ
  không hợp lệ, thứ tự ba goal thật trên ROS action server kiểm thử, dừng khi abort,
  hủy goal khi timeout, không gửi goal khi thiếu localization.
- Đây là action server giả lập cho kiểm thử luồng điều khiển; bằng chứng simulation
  thật nằm ở phần tiếp theo, không dùng các unit/integration test này làm demo robot.

## Simulation thật, hai node chạy song song

Dừng SLAM/simulation cũ, restart maze, load map YAML/PGM cũ bằng map_server, chạy AMCL
và Nav2 chính thức, undock bằng action. Seed initial pose được nhập cho vị trí/hướng
đã biết của robot, kiểm tra với LiDAR/map; ground truth chỉ được observer đọc.
Thêm Chair v3 Open Robotics vào world (-2.5,0.5,0), có mesh và collision thật.

`autonomy.launch.py` chạy waypoint_navigator và YOLO process riêng. Không teleop,
không đặt Nav2 Goal bằng chuột, không publish cmd_vel trực tiếp trong node sinh viên.

| Goal (x,y,yaw) map | Status | Thời gian thực |
| --- | --- | --- |
| (-2,-3,0) | 4 SUCCEEDED | 46.783 s |
| (2,-3,0) | 4 SUCCEEDED | 45.262 s |
| (-2,4,π/2) | 4 SUCCEEDED | 92.212 s |

`autonomy_parallel_results.json` có timestamp ROS bắt đầu/kết thúc từng goal.
`autonomy_demo/telemetry.json` ghi 3.717 frame RGB, 711 frame annotation, AMCL,
plan, ground truth và hazards. Đối chiếu timestamp xác nhận 117 frame có detection
trong khoảng thực thi navigation. Confidence lớn nhất 0.9425; ảnh thật là
`autonomy_demo/yolo_camera.png` cùng header/box trong `yolo_camera.json`.

Không có BUMP/CLIFF/STALL/WHEEL_DROP trong khoảng thực thi các goal; có 118 message
BACKUP_LIMIT (type 0), không tắt safety_override. Đây là kiểm tra message được lấy mẫu,
không phải chứng minh hình học liên tục về collision. Kết quả script kiểm chứng trong
`autonomy_demo/validation.json`. Không khẳng định mọi nhãn YOLO đúng: ngoài chair có
các false positive như traffic light/person/snowboard/stop sign trong cảnh này.

## Môi trường YOLO thực tế

Python 3.10.12; torch 2.5.1+cpu; torchvision 0.20.1+cpu; ultralytics 8.3.235;
NumPy 1.26.4; cv_bridge Humble; OpenCV apt 4.5.4. Venv dùng system-site-packages.
Do tải wheel OpenCV chậm, lượt kiểm thử tận dụng OpenCV apt hiện có và cài các wheel
ML/dependency cần cho inference. Không coi đây là kết quả `pip check` toàn bộ extras.
README cung cấp quy trình cài đầy đủ cho máy mới qua requirements-yolo.txt
(OpenCV >= 4.6, < 4.12, NumPy < 2).

Weights chính thức: yolov8n.pt từ Ultralytics assets v8.3.0.
SHA256: f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36.
Inference thử độc lập ~42 ms/ảnh; không coi đó là cam kết FPS khi toàn hệ thống chạy.
Giới hạn 2 luồng CPU được áp lại sau khi Ultralytics khởi tạo backend, OMP_WAIT_POLICY
PASSIVE, annotation Best Effort/depth 1, max_fps 3 và imgsz 416.

## Lỗi đã giữ lại và giới hạn nộp bài

Lần thử song song trước gặp TF cũ trong Nav2; đã cancel, giữ trace tại
`artifacts/autonomy_tf_delay/` trong workspace gốc, không tính đạt. View RViz từng ngừng
cập nhật; đã mở lại RViz, không restart/đặt lại pose hoặc điều khiển robot trong goal.
Các video ATTEMPT_* là lần thử, không dùng để nộp. Video chính DEMO_NAV_YOLO.webm chỉ
quay một cửa sổ RViz thật; không ghép ảnh tổng hợp hoặc gửi goal bằng công cụ quay.

Lượt camera 10 Hz đã kiểm chứng riêng tại autonomy_recorded/: ba goal lần lượt
SUCCEEDED trong 41.512 / 42.674 / 82.762 s. Thu 1.242 frame RGB, 763 frame annotation,
116 frame có detection trong khoảng navigation; max confidence 0.9427. Không có
BUMP/CLIFF/STALL/WHEEL_DROP; có 110 message BACKUP_LIMIT trong các goal. Không gộp
các mẫu này với lượt camera 30 Hz phía trên.

## Kết luận về video và thử RMW

Chưa có video hoàn chỉnh được nghiệm thu. XWayland từng ghi thiếu/stale panel ảnh;
Xephyr :99 ghi được màn hình RViz thật nhưng Nav2 vẫn gặp TF cũ trong các lượt lặp.
Thử Fast DDS/domain 45 cũng chưa giải quyết: một lượt kẹt gần Goal 1, lượt khác
không vượt qua readiness. Đã cancel/dừng các lượt này và giữ riêng trace trong
artifacts/autonomy_tf_delay/; video có tên ATTEMPT_*, không dùng để nộp.
Không kết luận DDS hoặc tải camera là nguyên nhân duy nhất.

Các số liệu SUCCEEDED phía trên thuộc đúng lượt đã ghi JSON, không suy ra rằng
video đạt yêu cầu hoặc mọi lần chạy đều ổn định. Drive Public/GitHub Public chưa có
link, lớp chưa cung cấp. REPORT_DRAFT.pdf là bản nháp cho tới khi hoàn thiện video
và các mục này.

Đối chiếu triệu chứng với issue upstream Nav2 #3352 (chỉ là tham khảo chẩn đoán,
không xác nhận cùng nguyên nhân): https://github.com/ros-navigation/navigation2/issues/3352

## Video mới đã kiểm chứng — 20/10/2026

Lượt CLEAN restart maze, gọi load_map thành công, seed initial pose (0,0,0) tại
dock, thêm ghế và undock SUCCEEDED; AMCL/planner/controller/bt_navigator active.
Ba goal status 4 trong 64.581 / 66.982 / 132.916 s thực, YOLO chạy song song.
Thu 1.030 RGB, 777 annotation, 154 detection frame trong navigation; confidence
tối đa 0.9365. Hazard type 1..4 bằng 0; type 0 BACKUP_LIMIT có 111 message.

Đây là dữ liệu artifacts/autonomy_video_clean/, được validate riêng theo timestamp
ROS của từng action. Không trộn lượt quay trước. RViz đã được mở mới với đồng hồ
hiển thị wall time; RobotModel bật, đường đi/vị trí robot cập nhật trên bản ghi.
Các node điều khiển vẫn sim time; không đổi timestamp, tăng TF tolerance hoặc tắt safety.

Bản ghi liên tục DEMO_NAV_YOLO_CLEAN.webm dài 354.229 s, 1600×900/8 FPS. MP4
TURTLEBOT4_NAV2_YOLO_DEMO.mp4 chỉ đổi codec/container toàn bộ bản ghi, không cắt/ghép.
Hai cửa sổ camera và trạng thái đều đọc nguồn ROS/log thật; không tái tạo chuyển động.
Các nhận xét “chưa có video” ở phần lịch sử phía trên mô tả trạng thái trước lượt này.
Drive/GitHub Public và lớp vẫn chưa có thông tin để điền.
