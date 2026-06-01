from .geometry_line import GeometryLine
from .dual_lane_road import DualLaneRoad


__all__ = ['GeometryLine', 'DualLaneRoad']



# # 添加单条实线
# road.add_horizontal_line(y_meters=25, line_type="solid", color=(255, 255, 255), width=4)

# # 添加多条虚线
# road.add_multiple_horizontal_lines([10, 20, 30, 40])