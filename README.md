# Giới thiệu
Xây dựng một robot có khả năng tự động di chuyển và tránh vật cản trong quá trình di chuyển. Có khả năng gửi dữ liệu bản đồ về máy tính thông qua kết nối không dây. Sử dụng mô hình robot vi sai, ESP32, cảm biến RPLIDAR A1, … để tạo ra một hệ thống robot di động tránh vật cản.
## Video demo
- [Demo thực tế](https://youtu.be/xoYMbwGrglY)
- [Testcases](https://www.youtube.com/watch?v=Yj0CbC2vuZs)
# Sơ đồ hệ thống
## Mô hình robot
![](/images/mo_hinh_robot.jpg)
## Sơ đồ phần cứng của robot
![](/images/So_do_phan_cung_robot.png)
Sơ đồ phần cứng gồm có: bộ 2 pin 18650, vi điều khiển ESP32, cảm biến lidar A1M8, hai động cơ DC 3V có chổi than và mạch điều khiển động cơ DC L298N. Sử dụng giao thức UART cho kết nối giữa ESP32 và cảm biến lidar A1M8.
# Giải thuật né vật cản
![](/images/giai_thuat_1.png)
Khi xe nhận biết có vật cản trong vùng nguy hiểm (màu hồng), xe thực hiện xử lý tín hiệu LiDAR (R = 800mm) để tìm 2 điểm AB - vùng trống để xe di chuyển. Sau đó thực hiện quay trái/phải theo phía vùng trống đến khi trong vùng nguy hiểm không còn vật cản thì thực hiện điều khiển xe đi thẳng.
# Kết nối giữa robot và máy tính
![Sơ đồ kết nối của robot và máy tính](/images/tcp_server.png)
- Kết nối giữa máy tính và robot chỉ là tùy chọn, robot có khả năng hoạt động độc lập mà không cần sự điều khiển từ máy tính.
- Giúp quan sát kết quả từ cảm biến lidar.
![Kết quả quan sát trên máy tính](/images/roboStudio.png)