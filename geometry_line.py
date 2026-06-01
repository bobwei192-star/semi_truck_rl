import pygame
import math
from typing import Tuple, Optional

class GeometryLine:
    _id_counter = 0
    
    def __init__(self, start: Tuple[float, float] = None, end: Tuple[float, float] = None, 
                 color: Tuple[int, int, int] = (255, 255, 0),
                 width: int = 2, line_id: Optional[int] = None,
                 line_type: str = "solid", dash_length: float = 1.0,
                 show_id: bool = True, scale: float = 20.0, **kwargs):
        self.scale = scale
        if start is not None and end is not None:
            self.start = start
            self.end = end
        else:
            start_x = None
            start_y = None
            end_x = None
            end_y = None
            
            for key, value in kwargs.items():
                if key.endswith('_start_x'):
                    start_x = value
                elif key.endswith('_start_y'):
                    start_y = value
                elif key.endswith('_end_x'):
                    end_x = value
                elif key.endswith('_end_y'):
                    end_y = value
            
            if start_x is None or start_y is None or end_x is None or end_y is None:
                raise ValueError("必须提供 start/end 或 xxx_start_x/xxx_start_y/xxx_end_x/xxx_end_y 参数")
            
            self.start = (start_x, start_y)
            self.end = (end_x, end_y)
        
        self.color = color
        self.width = width
        self.line_id = line_id if line_id is not None else GeometryLine._generate_id()
        self.line_type = line_type
        self.dash_length = dash_length
        self.show_id = show_id
        self._length = None
        self._font = pygame.font.SysFont(None, 16)
        
    @classmethod
    def _generate_id(cls) -> int:
        cls._id_counter += 1
        return cls._id_counter
    
    @classmethod
    def reset_id_counter(cls):
        cls._id_counter = 0
    
    @property
    def length(self) -> float:
        if self._length is None:
            dx = self.end[0] - self.start[0]
            dy = self.end[1] - self.start[1]
            self._length = math.sqrt(dx * dx + dy * dy)
        return self._length
    
    def draw(self, surface: pygame.Surface):
        s = self.scale
        if self.line_type == "solid":
            pygame.draw.line(surface, self.color, 
                           (int(self.start[0] * s), int(self.start[1] * s)),
                           (int(self.end[0] * s), int(self.end[1] * s)), self.width)
        elif self.line_type == "dashed":
            self._draw_dashed_line(surface)
        
        if self.show_id:
            self._draw_id(surface)
    
    def _draw_id(self, surface: pygame.Surface):
        s = self.scale
        id_text = f"ID:{self.line_id}"
        text_surface = self._font.render(id_text, True, (0, 0, 0))
        
        offset_x = 5
        offset_y = 5
        text_pos = (
            int(self.start[0] * s) + offset_x,
            int(self.start[1] * s) + offset_y
        )
        
        pygame.draw.rect(surface, (255, 255, 255), 
                        (text_pos[0], text_pos[1], text_surface.get_width(), text_surface.get_height()))
        surface.blit(text_surface, text_pos)
    
    def _draw_dashed_line(self, surface: pygame.Surface):
        s = self.scale
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        total_length = self.length
        
        if total_length == 0:
            return
        
        unit_x = dx / total_length
        unit_y = dy / total_length
        
        gap_length = self.dash_length
        current_pos = 0.0
        
        while current_pos < total_length:
            dash_start_x = self.start[0] + unit_x * current_pos
            dash_start_y = self.start[1] + unit_y * current_pos
            
            dash_end_pos = min(current_pos + self.dash_length, total_length)
            dash_end_x = self.start[0] + unit_x * dash_end_pos
            dash_end_y = self.start[1] + unit_y * dash_end_pos
            
            pygame.draw.line(surface, self.color,
                           (int(dash_start_x * s), int(dash_start_y * s)),
                           (int(dash_end_x * s), int(dash_end_y * s)), self.width)
            
            current_pos += self.dash_length + gap_length
    
    def get_info(self) -> dict:
        return {
            'id': self.line_id,
            'start': self.start,
            'end': self.end,
            'length': self.length,
            'type': self.line_type,
            'color': self.color,
            'width': self.width
        }
    
    def __repr__(self):
        return f"GeometryLine(id={self.line_id}, start={self.start}, end={self.end}, type='{self.line_type}')"
