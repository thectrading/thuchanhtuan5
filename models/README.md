# Vật thể tùy chọn cho camera

`demo_chair.sdf` tham chiếu mesh của [Chair v3, Open Robotics trên Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair).
Không kèm mesh/texture bên thứ ba trong repository. Lần đầu cần mạng hoặc Fuel cache:

```bash
ign fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair
ros2 launch turtlebot4_slam_nav demo_objects.launch.py
```

Giữ nguyên maze, chỉ thêm một ghế tĩnh có collision ở (-2.5, 0.5, 0) trong **world Gazebo**.
Vị trí này không phải tọa độ nhận diện được suy ra từ camera. Nav2 cập nhật vật cản mới
qua LiDAR; cần kiểm tra clearance và AMCL trước khi chạy. YOLO COCO có class `chair`,
nhưng độ tin cậy phụ thuộc ánh sáng/góc nhìn/kích thước ảnh. `fire extinguisher` không
thuộc 80 class COCO; muốn nhận diện cần model khác hoặc fine-tune.
