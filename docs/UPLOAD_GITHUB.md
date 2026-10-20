# Nộp GitHub Public và Google Drive

Bộ source cập nhật nằm trong `submission_nav_yolo/turtlebot4_slam_nav/` ở workspace;
bản nén mới là `turtlebot4_nav_yolo_submission.zip`. Không dùng ZIP bài SLAM cũ.
Giải nén trước khi upload để người chấm đọc được code trực tiếp.

1. Tạo repository GitHub tên tùy chọn, ví dụ `turtlebot4-nav-yolo`, chọn **Public**.
2. Upload nội dung package: README, package.xml, CMakeLists, launch, config, scripts,
   models, rviz, maps, tools, test, docs và artifacts được chọn. Giữ `.gitignore`.
3. Đảm bảo maps/map.yaml và maps/map.pgm tồn tại; source điều hướng và YOLO đều có.
   Không upload venv, weights, token, build/install/log. Xem README để build lại.
4. Video trong `src/turtlebot4_slam_nav/videos/` nếu có chỉ là file cục bộ. Upload
   video đầy đủ lên Drive, chọn **Anyone with the link / Viewer** và kiểm tra ẩn danh.
5. Trong `docs/REPORT.md`, điền lớp và hai link thật. Tên/MSSV đã điền là
   **Tô Văn Chúc – 23020727**. Chạy `python3 tools/export_report.py` trong thư mục
   package để xuất lại `docs/REPORT_DRAFT.pdf`; upload PDF mới cùng source.
6. Mở repository từ cửa sổ ẩn danh để xác nhận Public, kiểm tra link video cũng xem
   được, rồi nộp link repository, Drive và PDF theo yêu cầu giảng viên.

Video bài này: load map tuần trước, AMCL, 3 goal liên tiếp **bằng node**, YOLO camera
chạy song song. Không dùng video bấm Nav2 Goal bằng RViz của tuần trước. Kịch bản chi
tiết: [DEMO_CHECKLIST.md](DEMO_CHECKLIST.md). Các file BAO_CAO_TURTLEBOT4.* thuộc báo cáo
SLAM cũ; bản PDF hiện tại là REPORT_DRAFT.pdf.

Khi clone để chạy, đặt package ở `<workspace>/src/turtlebot4_slam_nav` rồi build theo
README. Nếu sửa source sau khi đóng gói, tạo lại bộ upload để tránh nộp bản cũ.
