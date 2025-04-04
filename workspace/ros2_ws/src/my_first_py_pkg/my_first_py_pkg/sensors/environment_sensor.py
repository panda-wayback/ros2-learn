#!/usr/bin/env python3
from .base_sensor import BaseSensor

class EnvironmentSensor(BaseSensor):
    """环境传感器"""
    def __init__(self, wall_hit_threshold=0.1):
        super().__init__('environment_sensor')
        self.wall_hit_threshold = wall_hit_threshold
        self.last_x = None
        self.last_y = None
        self.collision_count = 0
        self._last_reading = {
            'collision': False,
            'collision_count': 0,
            'position': {'x': None, 'y': None},
            'movement': {'dx': 0, 'dy': 0}
        }

    def update(self, current_x, current_y):
        """更新环境状态"""
        if not self.is_enabled():
            return self._last_reading

        if self.last_x is None or self.last_y is None:
            self.last_x = current_x
            self.last_y = current_y
            self._last_reading['position'] = {'x': current_x, 'y': current_y}
            return self._last_reading

        # 计算移动距离
        dx = abs(current_x - self.last_x)
        dy = abs(current_y - self.last_y)
        
        # 检测碰撞
        is_collision = dx < self.wall_hit_threshold and dy < self.wall_hit_threshold
        if is_collision:
            self.collision_count += 1
        
        # 更新传感器读数
        self._last_reading = {
            'collision': is_collision,
            'collision_count': self.collision_count,
            'position': {'x': current_x, 'y': current_y},
            'movement': {'dx': dx, 'dy': dy}
        }
        
        # 更新位置
        self.last_x = current_x
        self.last_y = current_y
        
        return self._last_reading

    def get_collision_count(self):
        """获取碰撞次数"""
        return self.collision_count

    def reset(self):
        """重置环境传感器状态"""
        self.last_x = None
        self.last_y = None
        self.collision_count = 0
        self._last_reading = {
            'collision': False,
            'collision_count': 0,
            'position': {'x': None, 'y': None},
            'movement': {'dx': 0, 'dy': 0}
        } 