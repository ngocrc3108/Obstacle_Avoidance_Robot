# Giới thiệu
xây dựng một robot có khả năng tự động di chuyển và tránh vật cản trong quá trình di chuyển. Có khả năng gửi dữ liệu bản đồ về máy tính thông qua kết nối không dây. Sử dụng mô hình robot vi sai, ESP32, cảm biến RPLIDAR A1, … để tạo ra một hệ thống robot di động tránh vật cản.
# Sơ đồ hệ thống
## Sơ đồ phần cứng của robot
![](/images/So_do_phan_cung_robot.png)
Sơ đồ phần cứng gồm có: bộ 2 pin 18650, vi điều khiển ESP32, cảm biến lidar A1M8, hai động cơ DC 3V có chổi than và mạch điều khiển động cơ DC L298N. Sử dụng giao thức UART cho kết nối giữa ESP32 và cảm biến lidar A1M8.
## Sơ đồ kết nối giữa robot và máy tính
![Sơ đồ kết nối của robot và máy tính](/images/So_do_he_thong.png)
- Giúp quan sát kết quả từ cảm biến lidar.
- Kết nối giữa máy tính và robot chỉ là tùy chọn, robot có khả năng hoạt động độc lập mà không cần sự điều khiển từ máy tính.
# Kết quả và demo
## Mô hình robot
![](/images/mo_hinh_robot.jpg)
## Video demo
- [Demo thực tế](https://youtu.be/xoYMbwGrglY)
- [Testcases](https://www.youtube.com/watch?v=Yj0CbC2vuZs)