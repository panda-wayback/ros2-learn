#!/usr/bin/env python3

class BaseController:
    """控制器基类"""
    def __init__(self, name):
        self.name = name
        self._enabled = True

    def enable(self):
        """启用控制器"""
        self._enabled = True

    def disable(self):
        """禁用控制器"""
        self._enabled = False

    def is_enabled(self):
        """检查控制器是否启用"""
        return self._enabled

    def reset(self):
        """重置控制器状态"""
        pass

    def update(self):
        """更新控制器状态"""
        pass 