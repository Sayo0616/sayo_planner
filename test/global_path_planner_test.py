from lane_visualizer import LaneVisualizer
import time
from map_info import MapInfo

from global_path_planner import GlobalPathPlanner

xodr_file = "../../../scenario/fragment/0_76_merge_82/0_76_merge_82.xodr"

points = [
    [1130, 971, "start", "blue"],   # 起始点
    [1004, 960, "target", "red"]   # 目标点
]

start_point = points[0][:2]
target_point = points[1][:2]

task_info = {
                        "startPos": start_point,
                        "targetPos": target_point,
                        "waypoint": [],
                        "dt": 0.1
                    }

map_info = MapInfo()
map_info.init_by_file(xodr_file)

visualizer = LaneVisualizer(width=20, height=20)
visualizer.init(map_info, points)

# 初始化规划器
global_planner = GlobalPathPlanner(map_info=map_info)
global_planner.init(task_info=task_info)

# 开始规划
start_clock = time.time()
lane_paths = global_planner.planning()
end_clock = time.time()

visualizer.visualize(if_plot_polygons=True)

print("Planning time: " + str(end_clock - start_clock))
if lane_paths is None:
    print("No path")
else:
    for lane_id in lane_paths:
        print(lane_id)
