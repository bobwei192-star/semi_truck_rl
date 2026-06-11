import numpy as np
import pygame

TRUCK_BORDER = (255, 0, 0)
TRAILER_BORDER = (165, 42, 42)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
GRAY = (128, 128, 128)
DARK_GREEN = (0, 100, 0)


def draw_truck(surface, truck, scale):
    x_px = truck.x_m * scale
    y_px = truck.y_m * scale

    t_len = (truck.truckWheelbase + 2 * truck.truckMargin) * scale
    t_wid = truck.truckWidth * scale
    tr_len = (truck.trailerWheelbase + truck.trailerMarginFront + truck.trailerMarginBack) * scale
    tr_wid = truck.trailerWidth * scale

    sf = 2
    t_big = pygame.Surface((int(t_len * sf), int(t_wid * sf)), pygame.SRCALPHA)
    pygame.draw.rect(t_big, TRUCK_BORDER, t_big.get_rect(), width=4 * sf)
    t_surf = pygame.transform.smoothscale(t_big, (int(t_len), int(t_wid)))

    tr_big = pygame.Surface((int(tr_len * sf), int(tr_wid * sf)), pygame.SRCALPHA)
    pygame.draw.rect(tr_big, TRAILER_BORDER, tr_big.get_rect(), width=4 * sf)
    tr_surf = pygame.transform.smoothscale(tr_big, (int(tr_len), int(tr_wid)))

    rot_t = pygame.transform.rotate(t_surf, -np.degrees(truck.h_rad))
    rot_tr = pygame.transform.rotate(tr_surf, -np.degrees(truck.h_rad + truck.g_rad))

    cnx = x_px + (-t_len / 2 + truck.truckMargin * scale) * np.cos(truck.h_rad)
    cny = y_px + (-t_len / 2 + truck.truckMargin * scale) * np.sin(truck.h_rad)
    tr_cx = cnx + (truck.trailerMarginFront - (truck.trailerWheelbase + truck.trailerMarginBack) / 2) * scale * np.cos(truck.h_rad + truck.g_rad)
    tr_cy = cny + (truck.trailerMarginFront - (truck.trailerWheelbase + truck.trailerMarginBack) / 2) * scale * np.sin(truck.h_rad + truck.g_rad)

    surface.blit(rot_t, rot_t.get_rect(center=(int(x_px), int(y_px))))
    surface.blit(rot_tr, rot_tr.get_rect(center=(int(tr_cx), int(tr_cy))))

    wl = truck.tireLen * scale
    ww = truck.tireWidth * scale
    for side in [-1, 1]:
        wx = (truck.truckWheelbase + truck.truckMargin) * scale - t_len / 2
        wy = side * t_wid / 2
        ws = pygame.Surface((int(wl), int(ww)), pygame.SRCALPHA)
        ws.fill(BLACK)
        rw = pygame.transform.rotate(ws, -np.degrees(truck.h_rad) - truck.s_deg)
        wp = (x_px + wx * np.cos(truck.h_rad) - wy * np.sin(truck.h_rad),
              y_px + wx * np.sin(truck.h_rad) + wy * np.cos(truck.h_rad))
        surface.blit(rw, rw.get_rect(center=wp))

    pygame.draw.circle(surface, RED, (int(x_px), int(y_px)), 5)


def draw_trajectory(surface, points, color=GREEN, radius=2):
    if len(points) < 2:
        return
    for i in range(1, len(points)):
        px, py = points[i - 1]
        cx, cy = points[i]
        if abs(cx - px) > 200 or abs(cy - py) > 200:
            continue
        pygame.draw.circle(surface, color, (int(cx), int(cy)), radius)


def init_pygame_screen(fullscreen=False, caption='Truck Parking'):
    import os
    if not pygame.get_init():
        os.environ.setdefault('SDL_VIDEODRIVER', 'x11')
        os.environ.setdefault('MESA_GL_VERSION_OVERRIDE', '3.3')
        os.environ.setdefault('MESA_GLSL_VERSION_OVERRIDE', '330')
        pygame.init()
    if fullscreen:
        info = pygame.display.Info()
        screen = pygame.display.set_mode(
            (info.current_w, info.current_h),
            pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
        width, height = info.current_w, info.current_h
    else:
        width, height = 1120, 700
        screen = pygame.display.set_mode((width, height),
                                          pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption(caption)
    return screen, width, height


def draw_corner_numbers(surface, truck, scale, offset_x=0, offset_y=0):
    """在车辆8个角外侧绘制编号（无背景方块，数字与角点有偏移）

    编号规则：
    牵引车头4个角（编号1-4）:
    1 - 车头右前角
    2 - 车头右后角
    3 - 车头左后角
    4 - 车头左前角

    挂车4个角（编号5-8）:
    5 - 挂车右前角（靠近车头）
    6 - 挂车右后角
    7 - 挂车左后角
    8 - 挂车左前角（靠近车头）
    """
    # 获取8个角的位置
    corners = truck.get_corner_positions()

    # 设置字体
    font = pygame.font.Font(None, 28)

    # 偏移量（像素）：数字与角点之间的距离
    offset_px = 18

    for i, (cx, cy) in enumerate(corners):
        # 转换为像素坐标
        px = cx * scale + offset_x
        py = cy * scale + offset_y

        corner_num = i + 1

        # 根据编号设置颜色
        if corner_num <= 4:
            text_color = (255, 0, 0)      # 车头角：红色
        else:
            text_color = (165, 42, 42)    # 挂车角：棕色

        # 渲染数字文本（带黑色描边增强可见性）
        text_surface = font.render(str(corner_num), True, text_color)
        text_w, text_h = text_surface.get_size()

        # 确定偏移方向：数字放在角点外侧
        # 车头角（1-4）：根据车头朝向计算外侧
        # 挂车角（5-8）：根据挂车朝向计算外侧
        if corner_num <= 4:
            # 车头角的外侧方向
            if corner_num == 1:      # 右前 -> 右上方
                dx, dy = 1, -1
            elif corner_num == 2:    # 右后 -> 右下方
                dx, dy = 1, 1
            elif corner_num == 3:    # 左后 -> 左下方
                dx, dy = -1, 1
            else:                    # 左前 -> 左上方
                dx, dy = -1, -1
        else:
            # 挂车角的外侧方向
            if corner_num == 5:      # 右前 -> 右上方
                dx, dy = 1, -1
            elif corner_num == 6:    # 右后 -> 右下方
                dx, dy = 1, 1
            elif corner_num == 7:    # 左后 -> 左下方
                dx, dy = -1, 1
            else:                    # 左前 -> 左上方
                dx, dy = -1, -1

        # 计算数字位置（带偏移）
        text_x = int(px + dx * offset_px - text_w / 2)
        text_y = int(py + dy * offset_px - text_h / 2)

        # 绘制黑色描边（增强可见性）
        for ox, oy in [(-1, -1), (-1, 1), (1, -1), (1, 1), (0, -1), (0, 1), (-1, 0), (1, 0)]:
            outline_surface = font.render(str(corner_num), True, (0, 0, 0))
            surface.blit(outline_surface, (text_x + ox, text_y + oy))

        # 绘制数字
        surface.blit(text_surface, (text_x, text_y))

        # 绘制角点标记（小圆点）
        pygame.draw.circle(surface, (0, 0, 0), (int(px), int(py)), 3)
