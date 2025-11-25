#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Single-file DQN training/evaluation for key-paper style reward only
- Action space: 2 (0=Uu, 1=PC5)
- Reward: Key-paper–style (Yacheur et al. 2023)
"""

import argparse, json, os
from collections import deque
import numpy as np
import torch, torch.nn as nn, torch.optim as optim
from ns3gym import ns3env

# -------------------------------
# Utils (FrameStack, Network, Buffer)
# -------------------------------
class FrameStack:
    def __init__(self, k, state_dim):
        self.k, self.state_dim = k, state_dim
        self.frames = deque(maxlen=k)
    def reset(self, s0):
        self.frames.clear()
        for _ in range(self.k): self.frames.append(s0)
        return np.concatenate(self.frames)
    def push(self, s):
        self.frames.append(s)
        return np.concatenate(self.frames)

class QNet(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim,128), nn.ReLU(),
            nn.Linear(128,128), nn.ReLU(),
            nn.Linear(128,action_dim))
    def forward(self, x): return self.fc(x)

class ReplayBuffer:
    def __init__(self, capacity=8000): self.buffer=deque(maxlen=capacity)
    def push(self,s,a,r,s_next,d): self.buffer.append((s,a,r,s_next,d))
    def sample(self,bs):
        import random; batch=random.sample(self.buffer,bs)
        s,a,r,s_next,d=map(np.array,zip(*batch)); return s,a,r,s_next,d
    def __len__(self): return len(self.buffer)

# -------------------------------
# Preprocess and reward
# -------------------------------
def preprocess_raw_state(obs_dict):
    uu  = np.clip((obs_dict["RSRP_Uu"]+95)/15,-1.0,1.0)
    pc5 = np.clip((obs_dict["RSRP_PC5"]+95)/15,-1.0,1.0)
    prr = obs_dict["PRR"]
    vel = np.clip(obs_dict["Velocity"]/30.0,-1.0,1.0)
    lat_ms = obs_dict["Latency"]*1000.0
    return np.array([uu,pc5,prr,vel,lat_ms],dtype=np.float32)

def compute_reward_keypaper(curr_state, prev_state=None,
                            tau_R=0.95,tau_L_ms=100.0,alpha=1.0,beta=1.0):
    uu,pc5,prr=curr_state[0],curr_state[1],curr_state[2]
    reception_success=1.0 if prr>=tau_R else 0.0
    latency_ok=1.0 if curr_state[4]<=tau_L_ms else 0.0
    ps=1.0 if (prr>=tau_R and latency_ok>=0.5) else 0.0
    lq=0.0
    if prev_state is not None:
        mean_now=0.5*(uu+pc5)
        mean_prev=0.5*(prev_state[0]+prev_state[1])
        lq=mean_now-mean_prev
    return 0.5*reception_success+0.5*alpha*ps+0.5*beta*lq

# -------------------------------
# Agent
# -------------------------------
class DQNAgent:
    def __init__(self,state_dim,action_dim,device):
        self.q=QNet(state_dim,action_dim).to(device)
        self.q_target=QNet(state_dim,action_dim).to(device)
        self.q_target.load_state_dict(self.q.state_dict())
        self.optim=optim.Adam(self.q.parameters(),lr=5e-4)
        self.gamma=0.99; self.replay=ReplayBuffer(8000)
        self.eps_start,self.eps_end,self.eps_decay=1.0,0.05,2000
        self.global_step=0; self.device=device
    def select_action(self,state,eval_mode=False):
        if eval_mode:
            with torch.no_grad():
                s=torch.tensor(state,dtype=torch.float32,device=self.device).unsqueeze(0)
                return int(torch.argmax(self.q(s),dim=1).item())
        import random
        eps=self.eps_end+(self.eps_start-self.eps_end)*np.exp(-1.0*self.global_step/self.eps_decay)
        self.global_step+=1
        if random.random()<eps: return random.randrange(self.q.fc[-1].out_features)
        with torch.no_grad():
            s=torch.tensor(state,dtype=torch.float32,device=self.device).unsqueeze(0)
            return int(torch.argmax(self.q(s),dim=1).item())
    def update(self,bs=128):
        if len(self.replay)<bs: return
        s,a,r,s_next,d=self.replay.sample(bs)
        s=torch.tensor(s,dtype=torch.float32,device=self.device)
        a=torch.tensor(a,dtype=torch.int64,device=self.device).unsqueeze(1)
        r=torch.tensor(r,dtype=torch.float32,device=self.device).unsqueeze(1)
        s_next=torch.tensor(s_next,dtype=torch.float32,device=self.device)
        d=torch.tensor(d,dtype=torch.float32,device=self.device).unsqueeze(1)
        q_val=self.q(s).gather(1,a)
        with torch.no_grad():
            best=self.q(s_next).argmax(1).unsqueeze(1)
            q_next=self.q_target(s_next).gather(1,best)
            q_tar=r+self.gamma*(1.0-d)*q_next
        loss=nn.functional.smooth_l1_loss(q_val,q_tar)
        self.optim.zero_grad(); loss.backward(); self.optim.step()
    def update_target(self): self.q_target.load_state_dict(self.q.state_dict())

# -------------------------------
# Main
# -------------------------------
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['train','eval'],default='train')
    parser.add_argument('--port',type=int,default=5554)
    parser.add_argument('--episodes',type=int,default=100)
    parser.add_argument('--steps',type=int,default=55)
    parser.add_argument('--framestack',type=int,default=2)
    parser.add_argument('--save-dir',type=str,default='./real_final2/keypaper/')
    parser.add_argument('--load',type=str,default='')
    args=parser.parse_args()

    device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    env=ns3env.Ns3Env(port=args.port,stepTime=0.5,startSim=False,simSeed=0,simArgs={},debug=False)
    base_state_dim=5
    state_dim=base_state_dim*args.framestack; action_dim=2
    agent=DQNAgent(state_dim,action_dim,device); fstack=FrameStack(args.framestack,base_state_dim)

    if args.load:
        agent.q.load_state_dict(torch.load(args.load,map_location=device))
        agent.q_target.load_state_dict(agent.q.state_dict())
        print(f"✅ Loaded {args.load}")
    os.makedirs(args.save_dir,exist_ok=True)

    prev_next=None
    for ep in range(args.episodes):
        raw_obs=env.reset()
        s0=preprocess_raw_state({
            "RSRP_Uu":raw_obs[0],"RSRP_PC5":raw_obs[1],
            "PRR":raw_obs[2],"Velocity":raw_obs[3],"Latency":raw_obs[4]})
        S=fstack.reset(s0)
        total_reward,total_prr=0.0,0.0
        total_pc5=0
        total_uu=0
        for t in range(args.steps):
            a=agent.select_action(S,eval_mode=(args.mode=='eval'))
            if a==0:
                total_uu+=1
            elif a==1:
                total_pc5+=1
            raw_next,_,done,_=env.step(a)
            s_next=preprocess_raw_state({
                "RSRP_Uu":raw_next[0],"RSRP_PC5":raw_next[1],
                "PRR":raw_next[2],"Velocity":raw_next[3],"Latency":raw_next[4]})
            S_next=fstack.push(s_next)
            r=compute_reward_keypaper(s_next,prev_state=prev_next)
            prev_next=s_next.copy()
            if args.mode=='train':
                agent.replay.push(S,a,r,S_next,float(done))
                if len(agent.replay)>999: agent.update()
            total_reward+=r; total_prr+=s_next[2]; S=S_next
            if done: break
        if args.mode=='train':
            if ep%5==0: agent.update_target()
            if ep%10==0:
                path=os.path.join(args.save_dir,f"ckpt_ep{ep}.pth")
                torch.save(agent.q.state_dict(),path); print(f"💾 Saved {path}")
        avg_prr=(total_prr/max(1,t+1))*100
        result_file = "package.json"
        data={}
        try:
            with open(result_file, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            print(f"파일 읽기 오류: '{result_file}' 파일을 찾을 수 없습니다.")
        # print(f"[Episode {ep}] total_reward={total_reward:.2f}, Avg PRR={avg_prr:.3f}, Avg Latency={avg_latency:.2f}ms, PC5={total_pc5}, Uu={total_uu}")
        print(f"[Episode {ep}] total_reward={total_reward:.2f}, Avg PRR={avg_prr:.3f}, PC5={total_pc5}, Uu={total_uu}")
        print(f"TOTAL SENT={data.get('Total Sent')}, TOTAL RECEIVED={data.get('Total Received')}, TOTAL PRR={data.get('Total PRR')}%")
    if args.mode=='train':
        final=os.path.join(args.save_dir,'dqn_final.pth')
        torch.save(agent.q.state_dict(),final); print(f"✅ Final model saved: {final}")
    env.close()

if __name__=='__main__': main()
