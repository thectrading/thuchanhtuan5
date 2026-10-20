# TurtleBot 4: Autonomous Navigation + YOLO

**Tô Văn Chúc – 23020727** · Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress · TurtleBot 4 Lite.

Dùng lại `maps/map.yaml` + `maps/map.pgm` của world **maze** từ bài trước.
Node Python `waypoint_navigator.py` gọi action `/navigate_to_pose` để đi lần lượt
Goal 1 → Goal 2 → Goal 3. Node `yolo_detector.py` nhận camera RGB, dùng `cv_bridge`
và YOLOv8n để xuất bounding box, class và confidence. Hai node chạy thành hai process.

## 1. Cài đặt và build

Clone/đặt package trong `<workspace>/src/turtlebot4_slam_nav`. Các lệnh dưới đây chạy
ở thư mục workspace; trên máy hiện tại là `/home/chuc/workspaces/ur_gz`.

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-venv
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/turtlebot4_slam_nav --ignore-src -r -y
colcon build --symlink-install --packages-select turtlebot4_slam_nav
source install/setup.bash
```

Node điều hướng dùng action trực tiếp; `nav2_simple_commander` là lựa chọn khác
được đề bài cho phép. Không cần gọi hai cách cùng lúc.

Cài YOLO trong Python 3.10 venv có `--system-site-packages` để dùng `cv_bridge` từ apt.
Giữ NumPy < 2 vì binary `cv_bridge` Humble dùng ABI NumPy 1.x. Dùng CPU để không tải CUDA:

```bash
/usr/bin/python3 -m venv --system-site-packages ~/venvs/tb4-yolo
~/venvs/tb4-yolo/bin/python -m pip install --upgrade pip
~/venvs/tb4-yolo/bin/python -m pip install torch==2.5.1+cpu torchvision==0.20.1+cpu \
  --index-url https://download.pytorch.org/whl/cpu
~/venvs/tb4-yolo/bin/python -m pip install -r src/turtlebot4_slam_nav/requirements-yolo.txt
mkdir -p ~/models/tb4
cd ~/models/tb4
~/venvs/tb4-yolo/bin/python -c 'from ultralytics import YOLO; YOLO("yolov8n.pt")'
```

Lần tải weights đầu cần Internet. Không đưa venv, weights hoặc `build/install/log`
vào GitHub. Ultralytics/weights có điều khoản giấy phép riêng; xem tài liệu upstream.

Mỗi terminal source ROS, workspace và cấu hình của máy:

```bash
cd /home/chuc/workspaces/ur_gz  # đổi nếu dùng workspace khác
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtlebot4_slam_nav prepare_ogre2_overlay.py
ros2 run turtlebot4_slam_nav prepare_camera_overlay.py --rate 10
source src/turtlebot4_slam_nav/scripts/lab_env.bash
```

Overlay Ogre 2 sửa lỗi LiDAR range_min trên GPU Intel của máy thực hành; chỉ cần tạo
một lần. Overlay camera giới hạn RGB-D từ 30 xuống 10 Hz để giảm tải rendering/DDS
trên máy thực hành; giữ nguyên độ phân giải, pose camera, LiDAR, map và controller.
lab_env.bash mặc định dùng Cyclone DDS/domain 43, như các lượt ba goal đã đạt.
Dùng cùng môi trường ở mọi terminal. Có thể chọn RMW/domain khác bằng biến
TB4_RMW_IMPLEMENTATION/TB4_ROS_DOMAIN_ID trước khi source; lần thử Fast DDS/domain 45
vẫn gặp lỗi TF, chưa phải giải pháp đã nghiệm thu. Không sửa `/opt/ros`.

## 2. Load bản đồ và localization

Chỉ chạy **một** simulator, dừng SLAM/teleop cũ trước khi chạy:

```bash
ros2 launch turtlebot4_slam_nav navigation.launch.py world:=maze \
  map:="$PWD/src/turtlebot4_slam_nav/maps/map.yaml" \
  rviz_config:="$PWD/src/turtlebot4_slam_nav/rviz/autonomy.rviz" \
  rviz_use_sim_time:=false
```

Thêm `gui:=false` nếu chỉ cần RViz để giảm tải. Map được tạo trong **maze**, không dùng
map này với warehouse/depot. Trong RViz: Fixed Frame = `map`, map topic `/map` với
Transient Local, scan `/scan` với Best Effort. Cấu hình `autonomy.rviz` có cả ảnh RGB
và ảnh YOLO; ảnh YOLO xuất hiện sau khi chạy detector.

Máy quay demo dùng `rviz_use_sim_time:=false` để tránh cửa sổ RViz ngừng cập nhật.
Tham số này chỉ đổi đồng hồ của phần hiển thị; AMCL, Nav2, robot và camera vẫn dùng
thời gian simulation, giữ nguyên timestamp dữ liệu ROS. Giá trị mặc định của launch
là `true`; máy khác có thể giữ mặc định nếu RViz cập nhật bình thường.

Dùng **2D Pose Estimate** đặt đúng vị trí/hướng robot thực tế trong Gazebo. Khi vừa
spawn trên dock ở gốc world, yaw ≈ 0. Undock bằng action trước khi chạy node điều hướng:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 run turtlebot4_slam_nav check_system.py --map
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

Undock của Create 3 trên máy này quay robot khoảng π rad. Nếu đặt initial pose **sau**
undock, dùng hướng hiện tại, không dùng lại yaw spawn = 0. Map và world có thể lệch;
không lấy ground truth Gazebo làm localization. Kiểm tra laser trùng tường, robot
trong RViz khớp Gazebo và AMCL hội tụ. Đóng teleop trước khi chạy autonomy.

## 3. Khảo sát ít nhất 3 goal trong frame map

Trước khi chạy code, dùng Publish Point và đọc:

```bash
ros2 topic echo /clicked_point
ros2 topic echo /amcl_pose --once
```

Publish Point chỉ cho x/y/z; yaw phải chọn theo hướng cuối mong muốn hoặc lấy từ
quaternion AMCL: `yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))` (radian).
Chọn vùng trắng đủ clearance. Các điểm tái sử dụng từ kiểm thử map hiện có:

| Goal | x (m) | y (m) | yaw (rad) | Khu vực |
| --- | --- | --- | --- | --- |
| 1 | -2.0 | -3.0 | 0.0 | Phía tây nam vùng trung tâm |
| 2 | 2.0 | -3.0 | 0.0 | Phía đông nam vùng trung tâm |
| 3 | -2.0 | 4.0 | 1.5707963268 | Hành lang phía bắc |

Sửa `config/waypoints.yaml`: ba mảng `goal_x`, `goal_y`, `goal_yaw` cùng độ dài ≥ 3.
Khi thay map phải khảo sát lại; không dùng tọa độ world thay tọa độ map.

## 4. Thêm vật thể và chạy tự động cùng YOLO

Maze mặc định chủ yếu có tường; YOLO không đảm bảo nhận diện tường như một class COCO.
Có thể thêm ghế có mesh và collision thật, giữ nguyên maze đã quét:

```bash
ign fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair
ros2 launch turtlebot4_slam_nav demo_objects.launch.py
```

Lệnh này chạy một lần mỗi lần restart world. Ghế nằm ở tọa độ world (-2.5, 0.5, 0).
Không đặt vật thể vào đường robot hoặc sát vị trí initial pose; LiDAR cập nhật vật
cản mới vào costmap. Nguồn mô hình: [models/README.md](models/README.md).

Sau khi xác nhận localization và undock, terminal thứ hai chạy:

```bash
ros2 launch turtlebot4_slam_nav autonomy.launch.py \
  python_executable:="$HOME/venvs/tb4-yolo/bin/python" \
  model:="$HOME/models/tb4/yolov8n.pt"
```

Trên máy hiện tại có sẵn môi trường và weights đã chạy thử. Khi đứng ở workspace:

```bash
ros2 launch turtlebot4_slam_nav autonomy.launch.py \
  python_executable:="$PWD/.venv-tb4-yolo/bin/python" \
  model:="$HOME/.cache/turtlebot4_slam_nav/models/yolov8n.pt"
```

Thêm `show_window:=true` để mở OpenCV; mặc định xem ảnh `/yolo/annotated_image` trên
RViz, Image display với transport `raw`. Không cần activate venv cho launch: launch
truyền interpreter vào process detector; executable điều hướng dùng Python ROS từ apt.

**Sau khi chạy node, không dùng teleop và không đặt Nav2 Goal bằng chuột.**
2D Pose Estimate và Publish Point chỉ dùng trong bước chuẩn bị.
Terminal tự in `Goal 1 SUCCEEDED`, rồi Goal 2, Goal 3. Goal bị reject/abort/cancel hoặc
quá thời gian thì node dừng chuỗi và trả exit code khác 0; không ghi đạt cho goal sau.
Ctrl+C gửi yêu cầu cancel goal đang chạy và chờ kết quả trong giới hạn thời gian.
`goal_timeout` và `readiness_timeout` tính bằng thời gian thực, kể cả khi Gazebo pause.
Không có publisher `/cmd_vel` trong code sinh viên; Nav2 điều khiển robot.
Nếu detector thoát, launch dừng hai node autonomy để hủy navigation đang chạy.
Sau goal cuối detector tiếp tục chạy cho tới Ctrl+C; simulator/Nav2 nằm ở terminal riêng.

Có thể chạy từng node để kiểm tra:

```bash
ros2 run turtlebot4_slam_nav waypoint_navigator.py --ros-args \
  --params-file src/turtlebot4_slam_nav/config/waypoints.yaml
~/venvs/tb4-yolo/bin/python src/turtlebot4_slam_nav/scripts/yolo_detector.py --ros-args \
  --params-file src/turtlebot4_slam_nav/config/yolo.yaml \
  -p model:="$HOME/models/tb4/yolov8n.pt"
ros2 topic echo /yolo/detections
ros2 topic echo /navigate_to_pose/_action/status
cat /tmp/tb4_waypoints.json
```

`/yolo/detections` là `std_msgs/String` chứa JSON gồm header ảnh, kích thước ảnh và
các `{class_id, class_name, confidence, xyxy}`; xyxy là pixel, không phải tọa độ map.
Ảnh annotation giữ nguyên timestamp/frame_id camera. Sensor subscription dùng Best
Effort. Worker chỉ giữ frame mới nhất, mặc định tối đa 3 FPS/416 px và 2 luồng CPU. Ảnh
annotation dùng Best Effort/depth 1 để hạn chế tải DDS; trong RViz dùng Best Effort.
Mỗi 2 giây log class/confidence hoặc không có detection trên ngưỡng 0.4.
YOLO chỉ quan sát; việc tránh vật cản do Nav2 + LiDAR đảm nhiệm. COCO có person, chair,
car, bottle; không có class bình chữa cháy. Nhận diện class khác cần đổi/training model.

## 5. Cấu trúc và luồng chương trình

```text
launch/    simulation, localization, navigation, autonomy, demo_objects
scripts/   waypoint_navigator.py, yolo_detector.py, công cụ bài trước
config/    waypoints.yaml, yolo.yaml, localization.yaml, nav2.yaml
maps/      map.yaml + map.pgm từ bài SLAM trước
rviz/      autonomy.rviz: map, LiDAR, paths, camera RGB và YOLO
models/    SDF ghế tùy chọn, tham chiếu mesh Gazebo Fuel
 test/     kiểm thử action ROS: thành công, abort, timeout/cancel, thiếu localization
 docs/     REPORT.md, REPORT_DRAFT.pdf, DEMO_CHECKLIST.md
```

```mermaid
flowchart LR
  MAP[map.yaml + map.pgm] --> MS[map_server]
  L[LiDAR /scan + odometry] --> A[AMCL]
  MS --> A
  A --> TF[map → odom → base_link]
  W[waypoint_navigator] -->|NavigateToPose: G1 → G2 → G3| N[Nav2 BT / NavFn / DWB]
  MS --> N
  L --> N
  TF --> N
  N -->|velocity_smoother /cmd_vel| R[TurtleBot 4 Gazebo]
  R -->|/oakd/rgb/preview/image_raw| C[cv_bridge BGR]
  C --> Y[YOLOv8n]
  Y --> O[/yolo/annotated_image + /yolo/detections + terminal]
```

Code gửi quaternion `z=sin(yaw/2), w=cos(yaw/2)` cùng timestamp ROS trong frame map.
Chờ AMCL, planner, controller và bt_navigator active, nhận `/amcl_pose` và TF trước
khi gửi goal. Nav2 dùng NavFn Dijkstra trên global costmap; DWB chọn vận tốc bám đường
trên local costmap, velocity_smoother phát vận tốc cho Create 3. Goal checker cấu hình
tolerance 0.25 m / 0.25 rad. Chi tiết và giới hạn bằng chứng trong báo cáo.

## 6. Kiểm tra và nộp bài

```bash
# Domain riêng, tránh ảnh hưởng simulator ở domain 43
ROS_DOMAIN_ID=44 python3 -m pytest -q src/turtlebot4_slam_nav/test
python3 src/turtlebot4_slam_nav/tools/export_report.py
```

[Báo cáo PDF](docs/REPORT_DRAFT.pdf) · [Nguồn báo cáo](docs/REPORT.md) ·
[Kịch bản quay demo](docs/DEMO_CHECKLIST.md) · [Đăng GitHub/Drive](docs/UPLOAD_GITHUB.md).
Bản báo cáo SLAM/goal RViz từ tuần trước còn trong các file `BAO_CAO_TURTLEBOT4.*`;
**bài hiện tại dùng REPORT_DRAFT.pdf**. Không dùng video bấm goal RViz cũ thay video
node tự động + YOLO. Bổ sung lớp, link video Drive công khai và repository GitHub Public
vào REPORT.md rồi xuất lại PDF trước khi nộp. Log kiểm thử không thay thế video.

## Xử lý lỗi

- Không có map/TF: kiểm tra initial pose, AMCL active, không chạy SLAM đồng thời.
- Không chạy robot: kiểm tra undock thành công, clock đang chạy, không có teleop khác.
- YOLO thiếu `cv_bridge`: tạo venv bằng Python 3.10 với `--system-site-packages` và source ROS.
- Lỗi NumPy ABI: dùng `numpy<2`, `opencv-python<4.12` như requirements, không nâng NumPy 2.
- Không có ảnh: `ros2 topic info /oakd/rgb/preview/image_raw -v`, kiểm tra camera bridge/QoS.
- Không detection: kiểm tra vật thể có trong class model, góc nhìn, ánh sáng; không tự gán nhãn.
- TF/RViz ngừng cập nhật khi bật camera: dùng camera overlay 10 Hz như
  cấu hình trên, restart sạch toàn bộ simulation; không chạy tiếp goal khi TF cũ.
  Máy này từng lặp lại TF cũ trong các lượt quay; việc đổi RMW chưa đủ để khẳng định đã xử lý dứt điểm lỗi.
- Máy chậm: `gui:=false`, RViz 5 FPS, giảm `max_fps`/`image_size`/`inference_threads` trong `config/yolo.yaml`.
- Các node không thấy nhau: source cùng `lab_env.bash`/ROS_DOMAIN_ID/RMW ở mọi terminal.
- Goal timeout: xem Nav2 log, clearance, TF và localization; không tăng timeout để che lỗi.

## Tài liệu chính thức

- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)
- [Nav2 Simple Commander Humble source](https://github.com/ros-navigation/navigation2/blob/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py)
- [Nav2 Simple Commander API](https://docs.nav2.org/commander_api/index.html) (đường dẫn đề bài; có thể thay đổi)
- [Ultralytics predict](https://docs.ultralytics.com/modes/predict/)
- [TurtleBot 4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)

Mapping từ tuần trước: xem `launch/mapping.launch.py`, `scripts/save_map.sh` và
[ghi chép tuần trước](docs/MAPPING_PREVIOUS.md); các yêu cầu demo RViz trong ghi chép
cũ thuộc bài trước.

Video đã quay và kiểm chứng: [TURTLEBOT4_NAV2_YOLO_DEMO.mp4](videos/TURTLEBOT4_NAV2_YOLO_DEMO.mp4)
(khoảng 5 phút 54 giây, không nằm trong ZIP source). Cả ba goal status=4; ảnh YOLO
và robot/đường đi trên RViz được ghi liên tục trong cùng lượt. Xem
[videos/README.md](videos/README.md) và `artifacts/autonomy_video_clean/validation.json`.
Các file ATTEMPT_* và ATTEMPT_VIEW_RESET_20261020.webm là lượt thử trước, không dùng để nộp.
Cần upload MP4 lên Drive và điền link công khai vào báo cáo.

Đóng gói lại source sau khi sửa báo cáo:

```bash
python3 src/turtlebot4_slam_nav/tools/package_submission.py
```

Thư mục mới là `submission_nav_yolo/turtlebot4_slam_nav`, ZIP mới là
`turtlebot4_nav_yolo_submission.zip`. Bộ nộp SLAM cũ được giữ nguyên.
