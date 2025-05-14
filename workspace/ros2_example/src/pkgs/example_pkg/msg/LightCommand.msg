# 请求部分
string action  # 动作类型：power(开关), brightness(亮度), color(颜色)
string value   # 动作值：可以是数字、字符串或布尔值
---
# 响应部分
string status  # 执行状态：success/error
string message # 详细信息 