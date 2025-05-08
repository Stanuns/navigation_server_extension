# navigation_server_extension
- 启动节点
```bashrc
ros2 run navigation_server_extension navigation_mgmt_server
```
## 导航服务接口
### 1 下发list进行导航
调用service名称: /navigation_server
- 传入参数：
cmd_name: 1
poses_list: [geometry_msgs/Pose1, geometry_msgs/Pose2, geometry_msgs/Pose3,...]
pose_estimate: ''


### 2 暂停导航，暂停当前导航点
- 传入参数：
cmd_name: 2
poses_list: ''
pose_estimate: ''

### 3 恢复导航，恢复进行之前暂停的导航点
- 传入参数：
cmd_name: 3
poses_list: ''
pose_estimate: ''


### 4 取消导航(取消当前的导航的list任务)
- 传入参数：
cmd_name: 4
poses_list: ''
pose_estimate: ''

### 5 手动重定位(位姿估计)
- 传入参数：
cmd_name: 5
poses_list: ''
pose_estimate: geometry_msgs/Pose1

服务调用成功返回参数：
result: True
message: 'Successfully ....'

服务调用失败返回参数：
result: False
message: 'Failed ...'