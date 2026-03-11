#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 4: 扰动估计网络训练 + 导出 ONNX
在 GPU 工作站上运行
"""
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
import os


# ============================================================
# 1. 网络结构: 1D-CNN (轻量, NUC 友好)
# ============================================================
class DisturbanceNet(nn.Module):
    """
    输入: (batch, window=20, 12) — 最近 20 帧的飞行状态
    输出: (batch, 3)            — 当前时刻的扰动力矩 [d_Tx, d_Ty, d_Tz]
    
    参数量 ~12K, CPU 推理 <0.3ms
    """
    def __init__(self, n_feat=12, n_out=3):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv1d(n_feat, 32, kernel_size=3), nn.ReLU(), nn.BatchNorm1d(32),
            nn.Conv1d(32, 64, kernel_size=3), nn.ReLU(), nn.BatchNorm1d(64),
            nn.Conv1d(64, 32, kernel_size=3), nn.ReLU(),
        )
        self.fc = nn.Sequential(
            nn.Linear(32, 32), nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(32, n_out),
        )

    def forward(self, x):
        # x: (batch, window, features) → (batch, features, window)
        x = x.permute(0, 2, 1)
        x = self.conv(x)
        x = x.mean(dim=2)    # 全局平均池化
        return self.fc(x)


# ============================================================
# 2. 数据集
# ============================================================
class DisturbanceDataset(Dataset):
    def __init__(self, path, window=20):
        raw = np.load(path).astype(np.float32)
        self.X = raw[:, :12]       # 输入: 12维状态
        self.Y = raw[:, 15:18]     # 标签: 力矩扰动 [d_Tx, d_Ty, d_Tz]
        self.W = window

        # 标准化
        self.x_mean = self.X.mean(0)
        self.x_std  = self.X.std(0) + 1e-8
        self.y_mean = self.Y.mean(0)
        self.y_std  = self.Y.std(0) + 1
