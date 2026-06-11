import pygame
from typing import List, Tuple, Optional
from .geometry_line import GeometryLine

YELLOW = (255, 215, 0)
DARK_YELLOW = (204, 153, 0)
ROAD_GRAY = (160, 160, 160)

class DualLaneRoad:
    def __init__(self, center_x: float, y_start: float, y_end: float,
                 lane_width: float = 4.0, scale: float = 20.0,
                 road_id: int = 1):
        self.center_x = center_x
        self.y_start = y_start
        self.y_end = y_end
        self.lane_width = lane_width
        self.scale = scale
        self.road_id = road_id
        self.total_width = lane_width * 2
        self.geometry_lines: List[GeometryLine] = []
        
        self.start_position = None
        self.goal_position = None
        
        GeometryLine.reset_id_counter()
        self._create_road_geometry()
        self._calculate_goal_position()
    
    def _create_road_geometry(self):
        self.geometry_lines.clear()
        
        left_edge_x = self.center_x - self.total_width / 2
        right_edge_x = self.center_x + self.total_width / 2
        
        left_line = GeometryLine(
            start=(left_edge_x, self.y_start),
            end=(left_edge_x, self.y_end),
            color=YELLOW,
            width=4,
            line_type="solid",
            scale=self.scale
        )
        left_line.line_id = self.road_id * 1000 + 1
        self.geometry_lines.append(left_line)
        
        right_line = GeometryLine(
            start=(right_edge_x, self.y_start),
            end=(right_edge_x, self.y_end),
            color=YELLOW,
            width=4,
            line_type="solid",
            scale=self.scale
        )
        right_line.line_id = self.road_id * 1000 + 2
        self.geometry_lines.append(right_line)
        
        center_line = GeometryLine(
            start=(self.center_x, self.y_start),
            end=(self.center_x, self.y_end),
            color=DARK_YELLOW,
            width=3,
            line_type="dashed",
            dash_length=0.75,
            scale=self.scale
        )
        center_line.line_id = self.road_id * 1000 + 3
        self.geometry_lines.append(center_line)
        
        self._create_special_lines(left_edge_x, right_edge_x)
    
    def _create_special_lines(self, left_edge_x: float, right_edge_x: float):
        base_y = 15
        
        l1013_start_x = left_edge_x
        l1013_start_y = base_y - 2.5 - 6.0
        l1013_end_x = self.center_x
        l1013_end_y = base_y - 2.5 - 6.0

        l1014_start_x = self.center_x
        l1014_start_y = l1013_start_y + 41.0
        l1014_end_x = right_edge_x
        l1014_end_y = l1013_end_y + 41.0

        l1015_start_x = self.center_x
        l1015_start_y = l1014_start_y + 18.5
        l1015_end_x = right_edge_x
        l1015_end_y = l1014_end_y + 18.5
        
        line_1013 = GeometryLine(
            l1013_start_x=l1013_start_x,
            l1013_start_y=l1013_start_y,
            l1013_end_x=l1013_end_x,
            l1013_end_y=l1013_end_y,
            color=YELLOW,
            width=3,
            line_type="dashed",
            dash_length=0.85,
            scale=self.scale
        )
        line_1013.line_id = self.road_id * 1000 + 13
        self.geometry_lines.append(line_1013)
        
        line_1014 = GeometryLine(
            l1014_start_x=l1014_start_x,
            l1014_start_y=l1014_start_y,
            l1014_end_x=l1014_end_x,
            l1014_end_y=l1014_end_y,
            color=YELLOW,
            width=3,
            line_type="dashed",
            dash_length=0.75,
            scale=self.scale
        )
        line_1014.line_id = self.road_id * 1000 + 14
        self.geometry_lines.append(line_1014)
        
        line_1015 = GeometryLine(
            l1015_start_x=l1015_start_x,
            l1015_start_y=l1015_start_y,
            l1015_end_x=l1015_end_x,
            l1015_end_y=l1015_end_y,
            color=YELLOW,
            width=3,
            line_type="dashed",
            dash_length=0.75,
            scale=self.scale
        )
        line_1015.line_id = self.road_id * 1000 + 15
        self.geometry_lines.append(line_1015)
    
    def _calculate_goal_position(self):
        line_1002 = self.get_line_by_id(1002)
        line_1003 = self.get_line_by_id(1003)
        line_1014 = self.get_line_by_id(1014)
        line_1015 = self.get_line_by_id(1015)
        
        if line_1002 and line_1003 and line_1014 and line_1015:
            left_bound_x = line_1002.start[0]
            right_bound_x = line_1003.start[0]
            top_bound_y = line_1014.start[1]
            bottom_bound_y = line_1015.start[1]
            
            goal_x = 58.25
            goal_y = 50.56
            
            self.goal_position = (goal_x, goal_y)
            
            print(f"库位边界计算:")
            print(f"  左边界(1002) X: {left_bound_x:.2f}m")
            print(f"  右边界(1003) X: {right_bound_x:.2f}m")
            print(f"  上边界(1014) Y: {top_bound_y:.2f}m")
            print(f"  下边界(1015) Y: {bottom_bound_y:.2f}m")
            print(f"  目标位置: ({goal_x:.2f}, {goal_y:.2f})m")
    
    def set_start_position(self, x_m: float, y_m: float):
        self.start_position = (x_m, y_m)
        print(f"起点位置: ({x_m:.2f}, {y_m:.2f})m")
    
    def draw(self, surface: pygame.Surface):
        for line in self.geometry_lines:
            line.draw(surface)
        
        self.draw_start_goal_markers(surface)
    
    def draw_start_goal_markers(self, surface: pygame.Surface):
        RED = (255, 0, 0)
        GREEN = (0, 255, 0)
        MARKER_RADIUS = 8
        
        if self.start_position:
            x_px = self.start_position[0] * self.scale
            y_px = self.start_position[1] * self.scale
            pygame.draw.circle(surface, RED, (int(x_px), int(y_px)), MARKER_RADIUS)
            pygame.draw.circle(surface, (255, 255, 255), (int(x_px), int(y_px)), MARKER_RADIUS - 2)
        
        if self.goal_position:
            x_px = self.goal_position[0] * self.scale
            y_px = self.goal_position[1] * self.scale
            pygame.draw.circle(surface, GREEN, (int(x_px), int(y_px)), MARKER_RADIUS)
            pygame.draw.circle(surface, (255, 255, 255), (int(x_px), int(y_px)), MARKER_RADIUS - 2)
    
    def get_line_by_id(self, line_id: int) -> Optional[GeometryLine]:
        for line in self.geometry_lines:
            if line.line_id == line_id:
                return line
        return None
    
    def get_all_lines_info(self) -> List[dict]:
        return [line.get_info() for line in self.geometry_lines]
    
    def get_road_boundaries(self) -> Tuple[float, float]:
        left = self.center_x - self.total_width / 2
        right = self.center_x + self.total_width / 2
        return (left, right)
    
    def add_horizontal_line(self, y_meters: float, line_type: str = "dashed", 
                           color: Tuple[int, int, int] = YELLOW, width: int = 3):
        left_x = self.center_x - self.total_width / 2
        right_x = self.center_x + self.total_width / 2
        
        horizontal_line = GeometryLine(
            start=(left_x, y_meters),
            end=(right_x, y_meters),
            color=color,
            width=width,
            line_type=line_type,
            dash_length=0.75,
            scale=self.scale
        )
        horizontal_line.line_id = self.road_id * 1000 + 10 + len(self.geometry_lines)
        self.geometry_lines.append(horizontal_line)
        return horizontal_line
    
    def add_multiple_horizontal_lines(self, y_positions: List[float], 
                                     line_type: str = "dashed",
                                     color: Tuple[int, int, int] = YELLOW, 
                                     width: int = 3):
        lines = []
        for y in y_positions:
            line = self.add_horizontal_line(y, line_type, color, width)
            lines.append(line)
        return lines
    
    def __repr__(self):
        return f"DualLaneRoad(id={self.road_id}, center_x={self.center_x}, width={self.total_width}m, lines={len(self.geometry_lines)})"
