#!/usr/bin/env python3
from simple_pid import PID
from .base_controller import BaseController

class DirectionController(BaseController):
    """方向盘位置控制器
    
    功能：
    1. 接收目标方向盘位置（-1到1）
       - -1：方向盘左极限
       -  0：方向盘正中
       -  1：方向盘右极限
    2. 使用PID控制器平滑方向盘位置变化，防止突然转向
    3. 输出平滑后的方向盘位置（-1到1）
    """
    def __init__(self):
        super().__init__('direction_controller')
        
        # PID控制器配置
        self.pid = PID(
            Kp=0.3,   # 比例系数：控制响应速度
            Kd=0.1,   # 微分系数：抑制超调
            setpoint=0.0,
            output_limits=(-1.0, 1.0),
            sample_time=0.01
        )
        
        # 记录当前方向盘位置
        self.current_position = 0.0

    def _process_input(self, target_position):
        """处理目标位置输入"""
        return min(max(-1.0, target_position), 1.0)

    def compute(self, target_position):
        """计算平滑后的方向盘位置"""
        if not self.is_enabled():
            return self.current_position
        
        # 1. 处理输入
        target_position = self._process_input(target_position)
        
        # 2. 更新当前位置
        self.current_position = self.pid(target_position, self.current_position)
        
        return self.current_position

    def reset(self):
        """重置控制器状态"""
        self.pid.reset()
        self.current_position = 0.0

    def tune(self, kp=None, kd=None):
        """调整PID参数
        
        Args:
            kp (float, optional): 比例系数，值越小越平滑
            kd (float, optional): 微分系数，抑制超调
        """
        if kp is not None:
            self.pid.Kp = kp
        if kd is not None:
            self.pid.Kd = kd 