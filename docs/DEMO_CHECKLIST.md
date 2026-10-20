# Video demo: Localization → 3 goal bằng code + YOLO

Sinh viên: **Tô Văn Chúc – 23020727**. Dùng world maze và map tuần trước.

1. Mở Gazebo/RViz, cho thấy `maps/map.yaml` và `map.pgm` được nạp bởi map_server.
   Không tạo lại map bằng SLAM trong bài này. Fixed Frame = map.
2. Dùng 2D Pose Estimate cho pose ban đầu, undock; cho thấy LiDAR khớp tường,
   robot trong Gazebo/RViz khớp nhau, `/amcl_pose`, AMCL active.
3. Trước khi lập trình/chạy: cho thấy ba tọa độ frame map trong `config/waypoints.yaml`.
   Có thể minh họa Publish Point `/clicked_point`; yaw tính/chọn riêng.
4. Thêm ghế tùy chọn bằng `demo_objects.launch.py`, kiểm tra nó nằm trong camera.
5. Đóng mọi teleop. Chạy `autonomy.launch.py` đúng interpreter/model như README.
6. Quay liên tục robot tự đi G1 (-2,-3,0) → G2 (2,-3,0) → G3 (-2,4,π/2).
   Từ lúc node chạy, không điều khiển bằng tay và không bấm Nav2 Goal trên RViz.
7. Trong cùng video phải thấy global plan/local costmap/robot, cửa sổ hoặc panel
   camera YOLO có bounding box, class, confidence, và log detection song song.
   Không chỉ chụp ảnh YOLO từ ảnh mẫu; phải là camera của robot trong simulation.
8. Ghi rõ `Goal 1/2/3 SUCCEEDED`, robot dừng ở mỗi điểm, không va chạm; lưu JSON kết quả.
   Nếu abort/reject/timeout thì sửa nguyên nhân và quay lại, không cắt ghép để che lỗi.
9. Chụp hình thật cho báo cáo: map kèm goal, robot đang bám plan, YOLO phát hiện ghế.
10. Upload Google Drive, chia sẻ Anyone with the link / Viewer, kiểm tra bằng ẩn danh.
    Repository GitHub Public chứa code/launch/config/map/README. Điền hai link vào
    `docs/REPORT.md`, thông tin lớp, rồi xuất `docs/REPORT_DRAFT.pdf`.

Có thể quay bằng công cụ ghi màn hình Ubuntu. Khi chỉ ghi cửa sổ RViz, cần mở cả panel
RGB và YOLO trong `rviz/autonomy.rviz`. Không dùng video `record_rviz_demo.py` cũ: công
cụ đó bấm goal bằng chuột và thuộc bài trước. Công cụ mới chỉ ghi hình/quan sát, không
được gửi goal thay node sinh viên.
