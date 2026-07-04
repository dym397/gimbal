#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
UAV 实时测距物理运动门控与自适应卡尔曼滤波器。
基于一维恒速(CV)模型，实施创新值检验与动态观测协方差调节。
"""

import numpy as np

class RuntimeMotionGate:
    def __init__(self, config: dict):
        # 从配置中提取超参数
        quality_cfg = config.get("measurement_quality", {})
        gate_cfg = config.get("motion_gate", {})
        filter_cfg = config.get("filters", {})

        self.max_speed = gate_cfg.get("max_radial_speed_mps", 50.0)
        self.max_accel = gate_cfg.get("max_radial_acceleration_mps2", 25.0)
        self.downweight_thresh = gate_cfg.get("downweight_threshold", 1.5)
        self.reject_thresh = gate_cfg.get("reject_threshold", 3.0)

        # 初始化卡尔曼滤波器状态 [distance, velocity]^T
        self.x = np.zeros(2, dtype=np.float32)
        # 初始化误差协方差矩阵 P
        self.P = np.array([[10.0, 0.0], [0.0, 5.0]], dtype=np.float32)

        # 过程噪声协方差 Q
        q_val = filter_cfg.get("kalman_process_noise", 0.01)
        self.Q = np.array([[q_val, 0.0], [0.0, q_val * 0.1]], dtype=np.float32)

        # 测量噪声协方差 R
        self.base_R = filter_cfg.get("kalman_measurement_noise", 1.0)
        self.R = self.base_R

        self.timestamp = None
        self.initialized = False

    def reset(self, initial_distance: float = 0.0):
        """重置滤波器状态"""
        self.x[0] = initial_distance
        self.x[1] = 0.0
        self.P = np.array([[10.0, 0.0], [0.0, 5.0]], dtype=np.float32)
        self.initialized = True
        self.timestamp = None

    def process(self, measured_distance: float, t: float, measurement_valid: bool = True) -> tuple[str, float, float]:
        """
        处理单帧输入，返回 (gate_decision, filtered_distance, radial_velocity)
        决策包含: ACCEPT, DOWNWEIGHT, REJECT
        """
        if not self.initialized:
            self.reset(measured_distance)
            self.timestamp = t
            return "ACCEPT", float(self.x[0]), float(self.x[1])

        dt = t - self.timestamp if self.timestamp is not None else 0.04
        if dt <= 0:
            dt = 0.04 # 容错

        self.timestamp = t

        # 1. 预测 (Coasting)
        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float32)
        x_pred = F.dot(self.x)
        P_pred = F.dot(self.P).dot(F.T) + self.Q

        # 限制预测速度
        x_pred[1] = np.clip(x_pred[1], -self.max_speed, self.max_speed)

        if not measurement_valid:
            # 如果观测值无效，卡尔曼不应该被虚拟的 fallback 测量所污染，而只进行 Coasting 预测
            self.x = x_pred
            self.P = P_pred
            return "REJECT", float(self.x[0]), float(self.x[1])

        # 基于当前估计距离计算自适应测量噪声 R，远距离小目标增大噪声容忍度并平滑收敛
        distance_factor = (measured_distance / 120.0) ** 2
        adaptive_scale = 1.0 + 8.0 * min(9.0, distance_factor) # 最大放大 73 倍
        current_R = self.base_R * adaptive_scale

        # 2. 计算残差与创新协方差
        H = np.array([1.0, 0.0], dtype=np.float32) # 仅观测距离
        predicted_distance = float(x_pred[0])
        innovation = measured_distance - predicted_distance

        S = float(P_pred[0, 0] + current_R)
        sigma = np.sqrt(max(1e-6, S))
        abs_inno = abs(innovation)

        decision = "ACCEPT"
        used_R = current_R

        # 3. 门控判定
        if abs_inno > self.reject_thresh * sigma:
            # 拒接观测值，完全退化为 Coasting 物理预测
            decision = "REJECT"
            self.x = x_pred
            self.P = P_pred
            # 限制速度在合理范围
            self.x[1] = np.clip(self.x[1], -self.max_speed, self.max_speed)
            return decision, float(self.x[0]), float(self.x[1])

        elif abs_inno > self.downweight_thresh * sigma:
            # 降权：放大观测噪声 R 相当于减小卡尔曼增益 K
            decision = "DOWNWEIGHT"
            # 动态非线性放大因子
            scaling = (abs_inno / (self.downweight_thresh * sigma)) ** 2
            used_R = current_R * (1.0 + 9.0 * scaling) # 基础放大并加上偏差惩罚

        # 4. 卡尔曼更新
        S = P_pred[0, 0] + used_R
        K = P_pred.dot(H.T) / S

        self.x = x_pred + K * innovation
        self.P = (np.eye(2) - np.outer(K, H)).dot(P_pred)

        # 加速度与速度门限限制
        delta_v = self.x[1] - x_pred[1]
        max_delta_v = self.max_accel * dt
        if abs(delta_v) > max_delta_v:
            self.x[1] = x_pred[1] + np.sign(delta_v) * max_delta_v

        self.x[1] = np.clip(self.x[1], -self.max_speed, self.max_speed)

        return decision, float(self.x[0]), float(self.x[1])
