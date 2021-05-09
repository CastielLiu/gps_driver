# gps解析程序

## 数据结构约定
为使不同品牌GPS均适用于东南大学至善智能车队及其他自动驾驶项目，约定各GPS解析程序除具备其特有消息类型外, 必须具有nav_msgs::Odometry消息且
1. pose.pose.position      UTM坐标
2. pose.pose.orientation   姿态四元数(ENU东北天)
3. twist.twist.linear      线速度(xyz-右前上)
4. pose.covariance[0]      航向角(rad)-ENU东北天/东向为0/逆时针为正
5. pose.covariance[1]      经度(deg)
6. pose.covariance[2]      纬度(deg)
7. pose.covariance[3]      卫星个数，当驱动无法获取卫星个数时，置该字段为-1(未知)
8. pose.covariance[4]      当前定位状态-按照GPGGA第6字段编码方式:
										0初始化， 1单点定位， 2码差分， 3无效PPS， 4固定解， 5浮点解， 
										6正在估算 7人工输入固定值， 8模拟模式， 9WAAS差分(有效定位,其他无效)
9. pose.covariance[5]      北向速度
10. pose.covariance[6]     东向速度
