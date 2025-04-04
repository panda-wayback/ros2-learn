#!/usr/bin/env python3

class BaseSensor:
    """传感器基类"""
    def __init__(self, name):
        self.name = name
        self._enabled = True
        self._last_reading = None
        self._last_update_time = None

    def enable(self):
        """启用传感器"""
        self._enabled = True

    def disable(self):
        """禁用传感器"""
        self._enabled = False

    def is_enabled(self):
        """检查传感器是否启用"""
        return self._enabled

    def get_reading(self):
        """获取传感器读数"""
        return self._last_reading

    def get_last_update_time(self):
        """获取最后更新时间"""
        return self._last_update_time

    def update(self):
        """更新传感器读数"""
        pass 