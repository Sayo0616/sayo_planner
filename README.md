# sayo_planner  

---

## Contents Introduction
* **planner**    
规控器目录
  * **calculate_functions.py:**   
    数学计算方法
  * **const_var.py:**  
  常量定义文件  
  * **map_info:**   
    结构化地图信息相关类
* **sample_reference_code**  
算法示例目录
  + **ray_casting.py:**  
  射线法判断车道原理示例
  + **A_star.py:**
  A*算法示例
* **test**  
测试代码目录
  * **visualize_road_1.py:**  
    地图可视化文件1
  * **visualize_road_2.py:**  
  地图可视化文件2
* **visualization_of_traffic**  
道路可视化图片存储目录
---

## Install

* 方法一：  
    `cd`到`planner`目录下  
    `git clone https://github.com/Sayo0616/sayo_planner.git`

* 方法二：  
    下载压缩包，解压到`planner`目录下

---

## How to Test Lane-Match Algorithm  

1. **坐标点修改**：  
修改`test/visualize_road_2.py`文件中的`state`前两个值（分别为坐标x，y）  
2. **高精地图文件路径修改**：  
修改`test/visualize_road_2.py`文件中的`xodr_file`，更改高精地图xodr文件路径
3. **运行**：   
运行`test/visualize_road_2.py`文件  

---  
