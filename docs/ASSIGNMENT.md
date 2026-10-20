# Bài thực hành: Nav2 tự động và nhận diện YOLO

Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Simulation. Tái sử dụng map YAML/PGM của tuần
trước, cùng world (package này: maze).

- Load map_server + AMCL + Nav2. Đặt initial pose và kiểm tra laser trùng tường.
- Khảo sát ít nhất ba tọa độ x/y/yaw trong frame map, ghi lại trước khi chạy code.
- Viết node Python/C++ gửi tuần tự ít nhất ba goal bằng nav2_simple_commander hoặc
  action NavigateToPose. Sau khi chạy node không điều khiển tay/đặt goal RViz.
- Subscribe camera RGB, cv_bridge chuyển Image sang OpenCV, YOLO nhận diện; hiển thị
  bounding box/class/confidence và log terminal khi navigation đang chạy.
- Nộp video demo đầy đủ, link Google Drive công khai, repository GitHub Public có
  code/launch/config/map/README, PDF ngắn giải thích pipeline/node/topic/action,
  localization/Nav2, ảnh thực tế và cả hai link.

Không dùng bằng chứng bấm goal RViz từ bài SLAM trước để chứng minh bài này. Sinh viên
phải giải thích được code, các topic/action, quaternion yaw, AMCL/costmap và xử lý ảnh.

Tài liệu: [TurtleBot Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html),
[Nav2 Simple Commander](https://docs.nav2.org/commander_api/index.html),
[Ultralytics](https://docs.ultralytics.com/).
