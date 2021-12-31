由于激光雷达安装在车的底盘，四周轮子会对雷达数据造成影响，因此需对四周轮子所在角度进行滤波处理
## 方法一：从源码处理
1. 订阅激光雷达节点`scan`，通过对激光雷达节点传过来的数据包进行过滤，从而获取某一扇区的雷达数据
2. 对想要去除的扇区进行清除
```py
#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("filteredscan", LaserScan, queue_size=10)

    def callback(self, data):

        newdata = data
        #从消息中读取的距离和强度数据是tuple，需要转成list以便操作
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        #通过清除不需要的扇区的数据来保留有效的数据
        for x in range(120,240):
            newdata.ranges[x]=0
            newdata.intensities[x]=0

        #前方180°的扇区
        #for x in range(90,270):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        #正前方60°的扇区
        #for x in range(30,330):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        self.pub.publish(newdata)
        rospy.loginfo(data)


if __name__ == '__main__':

    # Initialize
    rospy.init_node('LidarFilter', anonymous=False)
    lidar = DoFilter()

    rospy.spin()
```
tip：该方法我还没试过，应该可行

## 方法二：通过laser_filters进行过滤

### LaserScanAngularBoundsFilterInPlace
作用:相当与删除目标角度扇区的数据

参数设置示例：
```cpp
scan_filter_chain:
- name: angle
  type: LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: -1.57
    upper_angle: 1.57
```


```cpp
lower_angle (double) 最小角度（rad）
upper_angle (double) 最大角度（rad）
```

代码示例：
```cpp
scan_filter_chain:
# 左上轮
- name: angle1
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: 0.55
    upper_angle: 1
# 左下轮
- name: angle2
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: 1.9
    upper_angle: 2.5
# 右上轮
- name: angle3
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: -0.98
    upper_angle: -0.53
# 右下轮
- name: angle4
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: -2.49
    upper_angle: -1.93
```

## 其他滤波器
- LaserScanBoxFilter 无视一个区块内的数据（常用于无视机器人本体对激光雷达数据的干扰）
- ScanShadowsFilter 针对物体边沿的扫描和识别
- InterpolationFilter 插值滤波
- LaserScanIntensityFilter 设定强度阈值,超出则设置为nan
- LaserScanRangeFilter 设定距离阈值,超出则设置为nan
- LaserScanAngularBoundsFilter 将设定的角度外的扫描数据删除
- LaserScanAngularBoundsFilterInPlace 不会删除目标角度扇区外的数据，但会把对应扫描的距离值设为最大距离阈值+1
- LaserArrayFilter 使用中值过滤器等对距离和强度进行过滤


## 参考致谢
- [ROS中laser_filters软件包学习](https://blog.csdn.net/weixin_38437252/article/details/90020543)
- [laser_filters](http://wiki.ros.org/laser_filters)

