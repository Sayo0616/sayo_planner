from lane_visualizer import LaneVisualizer
import time
from map_info import MapInfo

xodr_file = "../../../scenario/fragment/0_76_merge_82/0_76_merge_82.xodr"

xodr_file = 'H:\\competition\\OnSite\\first_track\\onsite-structured-test-3.1.0\\scenario\\fragment\\0_231_merge_236\\0_231_merge_236.xodr' #"../../../scenario/fragment/0_76_merge_82/0_76_merge_82.xodr"

points = [
    [1136, 970, "start", "blue"],   # 起始点
    [1008, 967, "target", "red"]   # 目标点
]

start_point = points[0][:2]
target_point = points[1][:2]

map_info = MapInfo()
map_info.init_by_file(xodr_file)

visualizer = LaneVisualizer(width=20, height=20)
visualizer.init(map_info, points)

visualizer.visualize(if_plot_polygons=True)

start_clock = time.time()

lanes_located = map_info.find_lanes_located(start_point)

end_clock = time.time()

print("点{point}所处的车道为：".format(point=start_point))

for lane in lanes_located:
    print(lane.lane_id)

lanes_located = map_info.find_lanes_located(target_point)

print("点{point}所处的车道为：".format(point=target_point))

for lane in lanes_located:
    print(lane.lane_id)

print("耗时：", str(end_clock - start_clock))
