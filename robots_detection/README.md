# RELEASE 19.04.10 
## 发布说明
添加了检测目标帧数延迟，<armor.hpp> "pos_decsion delay" 处可修改延迟帧数；添加了近距离探测伪数据发布机制。

Author tel:15399257039

## 往期版本发行说明

### RELEASE 19.04.04 

修改多车装甲板的预筛选方案；修改ip部分一些函数的接口；彻底删除像素点角度预测备用方案；修改决策部分目标检测信息及装甲板位姿信息传递方式与内部接口；添加RELEASE mode 与 RECORD mode。

Author tel:15399257039

## 使用说明

### armor_param.hpp 模式说明

_对抗时仅开启 RELEASE mode 以达到最大效率_

_修改模式后记得编译_

* RELEASE mode 
> 发行模式，运行于ros环境下时开启，关闭后可以直接include<robots_detection_node.hpp> 在 gdb下新建cpp文件调用 void main_f(cv::Mat &Image) 调试

* TEST mode 
> 测试模式，用于显示可供观察识别效果的图片和输出运行时间

* DEBUG mode 
> 调试模式，用于输出位姿等少量信息

* DDBUG mode 
> 深度调试模式，用于输出目标筛选中的必要过程信息

* RECORD mode 
> 录像模式，用于保存录像 Ubuntu下默认保存于home区，可于 robots_detection_node.hpp 中自定义保存路径、名称、格式等信息

### armor_param.txt 部分参数调节说明

#### 敌方参数调试

* enemy_color 
> <0 , BLUE> <1, RED> 敌方颜色，战斗前调试

_以下参数对抗前非必要不建议调试_

#### 装甲板参数调试

* armor_plat_width 
> 装甲板两灯条中心实际距离 单位：m

* armor_plat_height 
> 装甲板灯条实际高度 单位：m

#### 图像处理参数调试（配合DEBUG mode 及 DDBUG mode）

* ratio_max 
> 装甲板长宽比最大值 可用于调节远近装甲板筛选条件，远则大，近则小，建议范围 3.9~5.0

* gray_min 
> 灰度图像分割阈值 低于此值会被置0，调节至灯条行为较为可观即可（形状近似矩形或椭圆） 建议范围 120~200 

* color_min 
> 通道分量做差后的分割阈值 低于此值会被置0 调节至大部分目标颜色能够不被置0即可 建议范围 70~120 

#### 云台调试参数调节

* x_os 
> 摄像头坐标系下 x方向偏置 目前安装条件下为0 

* y_os 
> 摄像头坐标系下 y方向偏置，即pitch轴电机中心与摄像头坐标系中心实际垂直距离 单位：m

* z_os 
> 摄像头坐标系下 z方向偏置，即yaw轴电机中心与摄像头坐标系中心实际水平距离 单位：m

* pitch_os 
> pitch轴角度偏置 下正上负 单位：弧度

* yaw_os 
> yaw轴角度偏置 左正右负 单位：弧度