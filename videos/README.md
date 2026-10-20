# Video Navigation + YOLO đã kiểm chứng

File dùng để xem/nộp: **TURTLEBOT4_NAV2_YOLO_DEMO.mp4**.
Bản gốc: DEMO_NAV_YOLO_CLEAN.webm. Thời lượng khoảng 5 phút 54 giây; 1600×900,
8 FPS; video không có âm thanh. MP4 là chuyển định dạng toàn bộ bản ghi liên tục,
không ghép các lượt chạy. Hai file video không nằm trong ZIP source.

Video có các chương: nạp map/AMCL/undock; Goal 1; Goal 2; Goal 3. Trên màn hình
Xephyr :99 là RViz thật, cửa sổ camera đọc RGB/YOLO Image topic thật và bảng trạng
thái đọc JSON/log của waypoint_navigator. tools/demo_camera.py và demo_status.py
chỉ quan sát, không gửi goal, initialpose hoặc cmd_vel. RViz dùng wall time cho
hiển thị để tránh đứng khung hình; các node robot/AMCL/Nav2/YOLO vẫn dùng sim time.

Cả ba goal SUCCEEDED trong 64.581 / 66.982 / 132.916 giây thực. Có 154 khung hình
chứa detection trong khoảng navigation, confidence tối đa 0.9365. Không ghi nhận
BUMP/CLIFF/STALL/WHEEL_DROP trong message hazard được theo dõi. Đây không phải
chứng minh hình học liên tục về tiếp xúc vật cản; YOLO vẫn có false positive.

Dữ liệu cùng lượt: artifacts/autonomy_video_clean/ gồm setup.json, waypoints.json,
telemetry.json, validation.json, video_info.json, ảnh và log. File trong thư mục
autonomy_video_final/ là lượt thử trước (một lượt đạt nhưng RViz đứng hình, rồi lượt
lặp abort); không trộn các timestamp đó vào kết quả lượt CLEAN.

Upload MP4 lên Google Drive, bật Anyone with the link / Viewer và điền link thật
vào docs/REPORT.md rồi xuất lại PDF. GitHub Public và lớp vẫn chờ bổ sung.

Các file ATTEMPT_*, ATTEMPT_VIEW_RESET_20261020.webm và video bài SLAM cũ không phải
video nộp của lượt đã nghiệm thu này.
