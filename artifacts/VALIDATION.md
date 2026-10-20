# Kiểm chứng thực tế — 2026-10-13

Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 6.18.0, TurtleBot 4 Lite, world maze.
Source bài UR3 được giữ nguyên; package mới là turtlebot4_slam_nav.

- Build ament_cmake và kiểm tra cú pháp Python/YAML/XML/Bash: đạt.
- Simulator dùng world, URDF, bridge và controller chính thức. Chỉ đổi sensor renderer
  Ogre 1 → Ogre 2 bằng overlay description; không sửa physics/geometry/controller.
- Đã nhận scan có khoảng cách hợp lệ, odometry, clock và TF. Native Ogre 1 trên Intel
  UHD 630 trả mọi tia tại range_min; renderer phần mềm đúng nhưng chậm. Cyclone DDS
  và giao tiếp localhost được dùng cho lần chạy cuối.
- Đã điều khiển robot qua nhiều nhánh maze bằng lệnh vận tốc trong kiểm thử mapping,
  tạo map thật từ SLAM Toolbox. Map 401×401, 0.05 m/pixel, khoảng 76.8% vùng sàn trống
  được quan sát theo phép đối chiếu gần đúng. Bản đồ vẫn còn vùng xám; chưa nghiệm thu
  video mapping bằng teleop và yêu cầu chất lượng bản đồ cuối cùng.
- Đã dừng SLAM/Gazebo, restart cùng world, map_server load map vừa lưu và AMCL chạy.
  Kiểm tra save/load giữ đúng 67.189 ô unknown, 88.298 ô free và 5.314 ô occupied.
- Action undock trả SUCCEEDED. Sau undock robot đã quay khoảng π rad; initial pose
  kiểm thử được đặt gần (-0.3,0,π), không dùng yaw spawn 0 sau khi đã quay.

Ba action Nav2 thực tế đều SUCCEEDED. AMCL, planner_server, controller_server và
bt_navigator được kiểm tra active trước khi gửi goal. Script không publish cmd_vel,
chỉ gửi initialpose và NavigateToPose; ground truth Gazebo chỉ được đọc để đối chiếu.

| Goal x/y/yaw | Trạng thái | Sai số x/y trong map | AMCL lệch Gazebo | Thời gian thực |
| --- | --- | --- | --- | --- |
| (-2,-3,0) | SUCCEEDED | 0.216 m | 0.067 m | 48.9 s |
| (2,-3,0) | SUCCEEDED | 0.250 m | 0.020 m | 53.6 s |
| (-2,4,π/2) | SUCCEEDED | 0.235 m | 0.056 m | 107.3 s |

Initial localization lệch Gazebo khoảng 0.050 m và 0.009 rad. Không ghi nhận BUMP,
CLIFF, STALL hoặc WHEEL_DROP trong ba goal. Có BACKUP_LIMIT (type 0) sau undock,
đã hết trong khoảng đầu của goal 1; không tắt safety_override. 143 mẫu ground truth
cho khoảng cách nhỏ nhất từ tâm robot tới tường maze khoảng 0.431 m. Đây là kiểm tra
lấy mẫu, không thay thế quan sát liên tục trong video.

Video teleop/RViz, ba goal bấm qua Nav2 Goal, Drive public, GitHub public và danh sách
câu hỏi của giảng viên chưa có. PDF là bản nháp và không thay thế các bằng chứng này.

Artifacts: build.txt, system_check_ogre2.txt, system_localized.txt,
map_coverage.json, map_roundtrip_check.json, navigation_probe.txt,
navigation_results.json, navigation_validation.json, map_preview.png. Lần thử có initial yaw sai được giữ riêng
tại initial_pose_error và không tính vào kết quả navigation.
