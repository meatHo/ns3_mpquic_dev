#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

def preprocess_raw_state(obs_dict):
    uu  = np.clip((obs_dict["RSRP_Uu"]+95)/15,-1.0,1.0)
    pc5 = np.clip((obs_dict["RSRP_PC5"]+95)/15,-1.0,1.0)
    prr = obs_dict["PRR"]
    vel = np.clip(obs_dict["Velocity"]/30.0, -1.0, 1.0)
    # lat = obs_dict["Latency"]   * 1000.0  # s->ms
    return np.array([uu, pc5, prr, vel], dtype=np.float32)

def compute_switch_penalty(action, last_actions=[None, None]):

    pen = 0.0

    prev_action, prev2_action = last_actions

    if prev_action is not None and action != prev_action:
        pen += 1.0

    if prev2_action is not None and prev_action is not None:
        if action == prev2_action and action != prev_action:
            pen += 4.0  # 추가 penalty

    last_actions[0] = action
    last_actions[1] = prev_action
    # print(f"패널티 {pen}")
    return pen

def compute_reward(prr_t, prr_tm1):
    a, b, c = 1, 0.5, 2
    τ_R = 0.99

    # prr_diff = (prr_t-prr_tm1)
    penalty = max(0, (τ_R-prr_t))
    return (
            a * prr_t * prr_t -
            # b * prr_diff -
            c * penalty
            # 시그모이드 쓰는것도 방법일듯
    )
# def compute_reward(prr_t, prr_tm1):
#     a, b, c = 1, 0.5, 2
#     τ_R = 0.98

#     # prr_diff = (prr_t-prr_tm1)
#     if prr_t<τ_R:
#         penalty=(τ_R-prr_t)*5
#     else:
#         penalty=-1
#     return (
#             a * prr_t * prr_t - c * penalty
#             # 시그모이드 쓰는것도 방법일듯
#     )



class FrameStack:
    def __init__(self, k, state_dim):
        self.k = k
        self.frames = deque(maxlen=k)
    def reset(self, s0):
        self.frames.clear()
        for _ in range(self.k):
            self.frames.append(s0)
        return np.concatenate(self.frames)
    def push(self, s):
        self.frames.append(s)
        return np.concatenate(self.frames)

class QNet(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 128), nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, action_dim)
        )
    def forward(self, x):
        return self.fc(x)

class ReplayBuffer:
    def __init__(self, capacity=8000):
        self.buffer = deque(maxlen=capacity)
    def push(self, s, a, r, s_next, done):
        self.buffer.append((s, a, r, s_next, done))
    def sample(self, batch_size):
        import random as _rnd
        batch = _rnd.sample(self.buffer, batch_size)
        s, a, r, s_next, d = map(np.array, zip(*batch))
        return s, a, r, s_next, d
    def __len__(self): return len(self.buffer)

class DQNAgent:
    def __init__(self, state_dim, action_dim, device):
        self.state_dim  = state_dim
        self.action_dim = action_dim
        self.device     = device
        self.q = QNet(state_dim, action_dim).to(device)
        self.q_target = QNet(state_dim, action_dim).to(device)
        self.q_target.load_state_dict(self.q.state_dict())
        self.optim = optim.Adam(self.q.parameters(), lr=1e-6) #러닝레이트 수정
        self.gamma = 0.99
        self.replay = ReplayBuffer(8000)
        self.eps_start, self.eps_end, self.eps_decay = 0.1, 0.0, 2000 #탐험률 축소
        self.global_step = 0
    def select_action(self, state, eval_mode=False):
        if eval_mode:
            with torch.no_grad():
                s = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
                return int(torch.argmax(self.q(s), dim=1).item())
        eps = self.eps_end + (self.eps_start - self.eps_end) * np.exp(-1.0 * self.global_step / self.eps_decay)
        self.global_step += 1
        if random.random() < eps:
            return random.randrange(self.action_dim)
        with torch.no_grad():
            s = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            return int(torch.argmax(self.q(s), dim=1).item())
    def update(self, batch_size=128):
        if len(self.replay) < batch_size: return
        s, a, r, s_next, d = self.replay.sample(batch_size)
        s = torch.tensor(s, dtype=torch.float32, device=self.device)
        a = torch.tensor(a, dtype=torch.int64,  device=self.device).unsqueeze(1)
        r = torch.tensor(r, dtype=torch.float32, device=self.device).unsqueeze(1)
        s_next = torch.tensor(s_next, dtype=torch.float32, device=self.device)
        d = torch.tensor(d, dtype=torch.float32, device=self.device).unsqueeze(1)
        q_val = self.q(s).gather(1, a)
        with torch.no_grad():
            # q_next = self.q_target(s_next).max(1)[0].unsqueeze(1) # DQN
            best_action = self.q(s_next).argmax(1).unsqueeze(1) # DDQN
            q_next = self.q_target(s_next).gather(1, best_action) # DDQN
            q_tar  = r + self.gamma * (1 - d) * q_next
        loss = nn.functional.smooth_l1_loss(q_val, q_tar)
        self.optim.zero_grad(); loss.backward(); self.optim.step()
    def update_target(self):
        self.q_target.load_state_dict(self.q.state_dict())