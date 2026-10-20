# TurtleBot 4 — SLAM Toolbox, AMCL và Nav2

Package thực hành cho **Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (Ignition 6)**.
Dùng robot `lite` và world `maze` chính thức, không tự viết simulator/controller.
Không trộn các lệnh `turtlebot4_gz_bringup` của Jazzy với bản Humble này.

## Cài đặt và build

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
cd /home/chuc/workspaces/ur_gz
rosdep install --from-paths src/turtlebot4_slam_nav --ignore-src -r -y
colcon build --symlink-install --packages-select turtlebot4_slam_nav
source install/setup.bash
```

**Trên máy này**, dùng Ogre 2 và Cyclone DDS như đã kiểm thử (xem giải thích ở mục
Xử lý lỗi). Thực hiện sau build, trước mọi lệnh chạy:

```bash
ros2 run turtlebot4_slam_nav prepare_ogre2_overlay.py
source /home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/scripts/lab_env.bash
```

Các terminal khác cũng source `lab_env.bash` sau ROS/workspace. Máy mới cần thêm
`sudo apt install ros-humble-rmw-cyclonedds-cpp` nếu dùng cấu hình này.

Nếu clone repository riêng, đặt package tại `<workspace>/src/turtlebot4_slam_nav`
và thay đường dẫn workspace trong các lệnh. Mỗi terminal cần source ROS và workspace:

```bash
source /opt/ros/humble/setup.bash
source /home/chuc/workspaces/ur_gz/install/setup.bash
source /home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/scripts/lab_env.bash
```

## 1. Kiểm tra mô phỏng

```bash
ros2 launch turtlebot4_slam_nav simulation.launch.py
```

Gazebo tự chạy với `-r`. Trong RViz đổi **Fixed Frame = odom** khi chưa có SLAM/AMCL.
RobotModel và LaserScan (`/scan`, Best Effort) phải hiển thị. Robot khởi đầu trên dock;
cần **undock trước khi teleop**:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 run turtlebot4_slam_nav check_system.py
ros2 topic info /scan -v
ros2 topic info /odom -v
ros2 topic info /cmd_vel -v
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.35 -p repeat_rate:=10.0 -p key_timeout:=0.6
```

Phím `i`: tiến; `j/l`: quay; `k`: dừng; `,`: lùi. Terminal teleop phải có focus.
Quan sát robot di chuyển trong Gazebo và pose `/odom` thay đổi để chứng minh nhận vận tốc.
`check_system.py` chỉ đọc sensor/TF, không tự chạy robot.

Hoặc dùng bảng nút để điều khiển thủ công khi mapping (chỉ chạy một teleop):

```bash
sudo apt install python3-pyqt5
ros2 run turtlebot4_slam_nav teleop_buttons.py
```

Nhấn giữ Tiến/Lùi/Trái/Phải; thả nút, mất focus hoặc bấm DỪNG sẽ gửi lệnh dừng.
Đóng bảng điều khiển trước khi gửi Nav2 Goal. Đây là teleoperation bằng GUI,
không thay thế bước navigation tự động.

## 2. Mapping bằng teleoperation

Dừng launch simulation bằng Ctrl+C rồi chạy:

```bash
ros2 launch turtlebot4_slam_nav mapping.launch.py
```

Hoặc giữ simulation đang chạy và dùng `mapping.launch.py start_sim:=false`.
Chỉ có một simulation tại một thời điểm. RViz **Fixed Frame = map**.
Undock rồi chạy teleop như mục 1. Di chuyển chậm, quay từng bước, đi qua các nhánh
hành lang, quan sát hai phía tường và quay lại vùng đã đi để tạo loop closure.
World maze rộng khoảng 20×20 m; không chỉ khảo sát vùng gần điểm xuất phát.
Kiểm tra các nhánh chưa quan sát (màu xám), tường kép, vùng bị xoay/lệch; quay lại
quan sát nếu cần. Map phải bao phủ phần lớn khu vực robot đi được trước khi nộp.

Lưu vào **source repository**, không lưu vào `/opt/ros`:

```bash
ros2 run turtlebot4_slam_nav save_map.sh \
  /home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/maps/map
```

Nếu prefix đã tồn tại, chọn `map_new` để giữ bản cũ, kiểm tra rồi thay file.
Map mới cần gồm `map.yaml` và `map.pgm`; `image:` trong YAML phải trỏ đúng PGM.
Script đặt `free_thresh=0.196`, `occupied_thresh=0.65`: với PGM trinary của Humble,
pixel unknown có mức xám 205 và cần giữ là unknown sau khi load lại. Ngưỡng free 0.25
có thể đọc các pixel đó thành free. Đã đối chiếu [map_io.cpp của Nav2 Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp).
Đây là occupancy map cho AMCL, không phải pose graph để tiếp tục SLAM.
Muốn tiếp tục SLAM cần lưu thêm pose graph bằng service serialize của SLAM Toolbox.

## 3. Khởi động lại và localization

Dừng teleop, SLAM và Gazebo bằng Ctrl+C. Đảm bảo các node cũ đã tắt, rồi chạy cùng world,
cùng pose spawn (mặc định `(0,0,0)`):

```bash
ros2 launch turtlebot4_slam_nav localization.launch.py \
  map:=/home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/maps/map.yaml
```

Trong RViz dùng **2D Pose Estimate**: đặt vị trí/hướng trên map tương ứng với robot
thực tế trong Gazebo. Hệ `map` từ SLAM có thể lệch nhẹ so với hệ world Gazebo;
đối chiếu các tường và laser, không coi mọi tọa độ world là tọa độ map.
Undock, di chuyển chậm một đoạn khi chưa gửi navigation goal để AMCL cập nhật.
Action undock trên bản Create 3 này quay robot khoảng 180° sau khi lùi khỏi dock.
Nếu đặt initial pose **sau undock**, phải dùng hướng đang có trong Gazebo, không dùng
lại yaw spawn bằng 0. Đặt đúng hướng đặc biệt quan trọng tại các vùng maze gần đối xứng.
Laser phải khớp tường và particle cloud phải hội tụ. Sau đó dừng teleop.

```bash
ros2 run turtlebot4_slam_nav check_system.py --map
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

AMCL phải `active`; SLAM Toolbox phải đã dừng (`ros2 node list`).

## 4. Navigation với Nav2

Dừng launch localization, khởi động lại bằng launch tổng hợp:

```bash
ros2 launch turtlebot4_slam_nav navigation.launch.py \
  map:=/home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/maps/map.yaml
```

Nếu đã giữ simulation/localization chạy, chỉ khởi chạy Nav2 chính thức để tránh
trùng node; dùng cấu hình của package:

```bash
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true \
  params_file:=/home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/config/nav2.yaml
```

Trong lần chạy mới: đặt **2D Pose Estimate**, undock, kiểm tra localization rồi
**tắt terminal teleop**. Đợi Navigation 2 hiển thị hệ thống active. Chọn **Nav2 Goal**,
click và kéo để đặt hướng cuối. Gửi lần lượt ít nhất **3 goal ở 3 khu vực khác nhau**,
chỉ gửi goal tiếp theo khi goal trước thành công. Goal phải nằm trong vùng trắng có
đủ khoảng trống cho robot; không đặt trên tường, vùng xám hoặc sát vật cản.

Quay đầy đủ global plan, local costmap, robot tự di chuyển và trạng thái hoàn thành.
Không publish `/cmd_vel` bằng tay sau khi gửi goal. Không dùng thao tác đổi pose Gazebo
để hoàn thành goal. Có thể ghi trạng thái action bằng:

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

Mã goal status: `4 = SUCCEEDED`, `5 = CANCELED`, `6 = ABORTED`.
Kiểm tra cả 3 goal thành công, robot dừng đúng đích và không chạm vật cản.

## Cấu trúc và pipeline

```text
turtlebot4_slam_nav/
├── launch/     simulation, mapping, localization, navigation
├── config/     slam.yaml, localization.yaml, nav2.yaml
├── rviz/       lab.rviz: robot, scan, map, particles, costmaps, paths
├── maps/       map.yaml, map.pgm từ SLAM thực tế
├── scripts/    kiểm tra sensor/TF và lưu map
├── docs/       báo cáo và hướng dẫn quay demo
└── artifacts/  bằng chứng kiểm chứng
```

```mermaid
flowchart LR
  G[Gazebo + TurtleBot 4/Create 3] <--> B[ros_gz_bridge]
  B --> S[/scan + /odom + TF + /clock]
  S --> M[SLAM Toolbox khi mapping]
  M --> F[map.yaml + map.pgm]
  F --> MS[map_server]
  S --> A[AMCL khi localization/navigation]
  MS --> A
  A --> TF[map → odom]
  R[RViz Nav2 Goal] --> BT[bt_navigator]
  MS --> N[Nav2 costmaps]
  S --> N
  N --> P[NavFn global planner]
  P --> C[DWB controller]
  BT --> P
  BT --> C
  C --> V[velocity_smoother → /cmd_vel]
  V --> B
```

Config dựa trên `turtlebot4_navigation` Humble (Clearpath Robotics, Apache-2.0).
SLAM giảm mật độ pose khi robot đứng yên; Nav2 tăng radius bảo vệ lên 0.19 m và không
lập đường qua vùng chưa biết. RViz dựa trên `turtlebot4_viz` và bỏ các sensor TB3 không tồn tại.
Các frame/topic cụ thể và ý nghĩa nằm trong báo cáo.

## Nộp bài

Hướng dẫn tự đăng file bằng trình duyệt: [UPLOAD_GITHUB.md](docs/UPLOAD_GITHUB.md).

Xem [đề bài](docs/ASSIGNMENT.md), [kịch bản demo](docs/DEMO_CHECKLIST.md) và
[báo cáo đã trình bày lại](docs/REPORT_WORD.md).
Báo cáo chỉnh sửa bằng Word: [DOCX](docs/BAO_CAO_TURTLEBOT4.docx) hoặc
[DOC](docs/BAO_CAO_TURTLEBOT4.doc). Bản xem nhanh/nộp PDF:
[BAO_CAO_TURTLEBOT4.pdf](docs/BAO_CAO_TURTLEBOT4.pdf).
Điền thông tin sinh viên, link video và GitHub trước khi nộp.
Video phải cho thấy mapping bằng teleop và **gửi goal qua RViz**; kiểm thử action bằng
script không thay thế yêu cầu này. Chỉ đánh dấu đạt khi đã có bằng chứng.
Upload video lên Drive, chọn “Anyone with the link”/Viewer, kiểm tra link bằng cửa sổ
ẩn danh. Repository GitHub cần Public, chứa package và map thật; điền cả hai link vào
báo cáo rồi xuất PDF cuối cùng. Không commit build/install/log, token hoặc dữ liệu riêng.

Xuất lại Word sau khi chỉnh sửa `docs/REPORT_WORD.md` (hoặc chỉnh trực tiếp file Word):

```bash
sudo apt install python3-docx python3-pil libreoffice-writer
python3 src/turtlebot4_slam_nav/tools/export_word.py
libreoffice --headless --convert-to 'doc:MS Word 97' \
  --outdir src/turtlebot4_slam_nav/docs \
  src/turtlebot4_slam_nav/docs/BAO_CAO_TURTLEBOT4.docx
libreoffice --headless --convert-to pdf \
  --outdir src/turtlebot4_slam_nav/docs \
  src/turtlebot4_slam_nav/docs/BAO_CAO_TURTLEBOT4.docx
```

Xuất lại PDF sau khi cập nhật báo cáo:

```bash
sudo apt install python3-reportlab fonts-dejavu-core python3-numpy python3-pil \
  python3-matplotlib python3-yaml
python3 src/turtlebot4_slam_nav/tools/export_report.py
```

Các script `tools/mapping_probe.py`, `tools/navigation_probe.py`, `tools/map_metrics.py`
phục vụ kiểm thử tích hợp/đánh giá, không tự thay thế teleop và Nav2 Goal trong video.

## Xử lý lỗi

**Máy lag khi mở Gazebo và RViz:** chạy Gazebo ở chế độ server, chỉ mở RViz với cấu hình
15 FPS để xem map, robot và đường đi. Sau khi source môi trường, dùng:

```bash
ros2 launch turtlebot4_slam_nav navigation.launch.py gui:=false \
  rviz_config:=/home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/rviz/demo.rviz
```

Simulator vẫn chạy physics/sensor; chỉ bỏ cửa sổ Gazebo. Khi quay phần cần thấy Gazebo,
dùng lại `gui:=true`. Để dừng toàn bộ, **Ctrl+C ở terminal launch**; đóng riêng cửa sổ
RViz không dừng simulator và các node ROS phía sau.

**RViz không hiện robot, Global Status báo Error:** chỉ chạy một phiên simulator.
Hai lần `navigation.launch.py` đồng thời sẽ tạo node trùng tên và nhiều nguồn `/clock`,
khiến RViz báo `Detected jump back in time` rồi xóa TF. Dừng các phiên cũ bằng Ctrl+C,
đợi Gazebo tắt hẳn, sau đó mở lại một phiên.

Khi localization, đặt **2D Pose Estimate** đúng vị trí/hướng thực tế để AMCL phát
`map → odom`; trước bước này robot có thể chưa hiện với Fixed Frame = map và Nav2
còn chờ TF. Nếu robot vừa spawn, chưa undock, hướng mặc định là yaw 0; nếu đã undock,
dùng hướng hiện tại (trong bản kiểm thử quay khoảng π rad). Với simulation chưa chạy
SLAM/AMCL, dùng Fixed Frame = odom. Kiểm tra bằng
`ros2 run turtlebot4_slam_nav check_system.py --map` khi đã localization.

Trên máy phát triển này, **Ogre 1 + Intel UHD 630** trả mọi tia LiDAR ở range_min;
renderer phần mềm cho scan đúng nhưng rất chậm. Overlay chỉ đổi renderer sensor
`ogre → ogre2` của mô hình Create 3 chính thức và Cyclone DDS đã khởi động ổn định.
Không sửa file hệ thống hoặc thay controller. Để tái hiện môi trường đã kiểm thử:

```bash
# Máy mới: cài RMW nếu chưa có. Máy này có bản extract tại ~/.local/share/tb4_dds.
sudo apt install ros-humble-rmw-cyclonedds-cpp
source /opt/ros/humble/setup.bash
source /home/chuc/workspaces/ur_gz/install/setup.bash
ros2 run turtlebot4_slam_nav prepare_ogre2_overlay.py
source /home/chuc/workspaces/ur_gz/src/turtlebot4_slam_nav/scripts/lab_env.bash
ros2 launch turtlebot4_slam_nav mapping.launch.py
```

Source `lab_env.bash` ở **mọi terminal** của bài, kể cả terminal lưu map, teleop và
localization/Nav2. Nó chọn ROS_DOMAIN_ID=43 và Cyclone DDS; không trộn với terminal
Fast DDS/domain khác. Overlay nằm trong `~/.cache/turtlebot4_slam_nav/ogre2`, được
tạo từ package đã cài, không cần đưa toàn bộ mô hình upstream vào repository.
Nếu muốn dùng bản simulator nguyên gốc, mở terminal mới và bỏ bước source này.
Sau Ctrl+C chờ Gazebo/node cũ tắt hẳn trước khi restart.
Giao tiếp ROS/Gazebo được cố định về localhost cho bài mô phỏng một máy, để không mất
`/scan` hoặc `/clock` khi đổi Wi-Fi/USB mạng. Script bỏ `SNAP_COMMON` được kế thừa từ
VS Code Snap để SLAM Toolbox không tự chuyển đường dẫn serialize sang thư mục Snap.

- `/scan` thiếu: kiểm tra Gazebo đang chạy, LiDAR bridge, renderer GPU; trên máy không
  có GPU hoạt động thử `LIBGL_ALWAYS_SOFTWARE=1` (chậm hơn). Headless vẫn cần renderer
  cho GPU LiDAR: `gui:=false rviz:=false` không có nghĩa là bỏ rendering sensor.
- Robot không di chuyển: undock, kiểm tra action trả success, publisher/subscriber
  `/cmd_vel`, hazard Create 3 và thời gian `/clock`; không tắt an toàn để che lỗi.
- RViz không thấy map: Fixed Frame `map`, topic `/map`, durability Transient Local.
- TF lỗi: mọi node dùng `use_sim_time=true`, không chạy đồng thời SLAM và AMCL;
  một nguồn `map → odom`, một simulation, không trộn ROS_DOMAIN_ID giữa terminal.
- Nav2 abort: xem `planner_server`/`controller_server`, goal nằm trên vùng đã map,
  đúng initial pose và đủ clearance; thử goal gần trước rồi khảo sát lại vùng thiếu.
- Thay world/model/spawn khi localization phải khớp lần mapping và cập nhật báo cáo.

## Tài liệu chính thức

- [TurtleBot 4 Simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- [TurtleBot 4 Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
- [Nav2 concepts](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/)

Launch đã đối chiếu source cài trên Humble: `turtlebot4_navigation` dùng argument
`params` cho SLAM/localization và `params_file` cho Nav2. Mô phỏng dùng component spawn
chính thức để truyền đúng `model` và dùng `map` của bài thay vì map mẫu upstream.
