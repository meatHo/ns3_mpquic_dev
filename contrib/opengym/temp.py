import subprocess
import numpy as np
import random
import re
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

# === DQN 모델 정의 ===
class DQN(nn.Module):
    def __init__(self, n_states, n_actions):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(n_states, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, n_actions)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# === Replay Buffer ===
class ReplayBuffer:
    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state):
        self.buffer.append((state, action, reward, next_state))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state = map(np.array, zip(*batch))
        return state, action, reward, next_state

    def __len__(self):
        return len(self.buffer)

# === DQN Agent ===
class DQNAgent:
    def __init__(self, n_states, n_actions, alpha=1e-3, gamma=0.9, epsilon=0.3):  # epsilon 초기값 0.3 (낮춤)
        self.n_actions = n_actions
        self.gamma = gamma
        self.epsilon = epsilon

        self.policy_net = DQN(n_states, n_actions)
        self.target_net = DQN(n_states, n_actions)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=alpha)
        self.criterion = nn.MSELoss()

        self.replay_buffer = ReplayBuffer()
        self.batch_size = 32
        self.target_update_interval = 20
        self.step_count = 0

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.n_actions)
        with torch.no_grad():
            state_tensor = torch.tensor([state], dtype=torch.float32)
            q_values = self.policy_net(state_tensor)
            return q_values.argmax().item()

    def update(self):
        if len(self.replay_buffer) < self.batch_size:
            return

        state, action, reward, next_state = self.replay_buffer.sample(self.batch_size)

        state = torch.tensor(state, dtype=torch.float32)
        next_state = torch.tensor(next_state, dtype=torch.float32)
        action = torch.tensor(action, dtype=torch.long)
        reward = torch.tensor(reward, dtype=torch.float32)

        q_values = self.policy_net(state).gather(1, action.unsqueeze(1)).squeeze(1)
        next_q_values = self.target_net(next_state).max(1)[0].detach()
        target = reward + self.gamma * next_q_values

        loss = self.criterion(q_values, target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # target network 주기적 업데이트
        self.step_count += 1
        if self.step_count % self.target_update_interval == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())

# === 행동 공간 정의 ===
actions = ["stay", "gnb1", "gnb2", "gnb3", "gnb4", "gnb5", "rsu1", "rsu2", "rsu3"]
n_actions = len(actions)

# === NS-3 실행 함수 ===
def run_ns3(action_idx):
    target = actions[action_idx]
    result = subprocess.run(
        ["./ns3", "run", f"uusidelink --handoverTarget={target}"],
        cwd="/home/yebin/ns-3-dev",
        capture_output=True, text=True
    )

    output = result.stdout + result.stderr
    throughput, pkt_loss, density = 0.0, 0, 1
    rssi_dict = {a: -100 for a in actions if a != "stay"}

    for line in output.splitlines():
        if "RESULT:" in line:
            match_tp = re.search(r"throughput=([0-9\.]+)", line)
            match_loss = re.search(r"loss=([0-9]+)", line)
            match_density = re.search(r"density=([0-9]+)", line)

            if match_tp:
                throughput = float(match_tp.group(1))
            if match_loss:
                pkt_loss = int(match_loss.group(1))
            if match_density:
                density = int(match_density.group(1))

            # 여러 기지국 RSSI 파싱
            for a in rssi_dict.keys():
                m = re.search(rf"rssi_{a}=(-?[0-9\.]+)", line)
                if m:
                    rssi_dict[a] = float(m.group(1))

    rssi = rssi_dict.get(target, -100)
    return throughput, rssi, pkt_loss, density

# === 상태 인코딩 ===
def encode_state(current_bs_idx, rssi, density):
    rssi_level = 0 if rssi < -85 else 1
    density_level = 0 if density < 10 else 1
    return current_bs_idx * 4 + rssi_level * 2 + density_level

# === 보상 함수 (스루풋 개선 여부에 따른 패널티 차등 적용) ===
def compute_reward(throughput, pkt_loss, action, prev_action, prev_thr):
    reward = throughput
    if pkt_loss > 0:
        reward -= 1.0
    if action != prev_action and action != 0:
        if throughput > prev_thr:   # 핸드오버 후 스루풋이 개선됨
            reward -= 0.5
        else:                       # 핸드오버했는데 개선되지 않음
            reward -= 2.0
    if action == 0:
        reward += 0.5   # stay 보상 강화
    return reward

# === 메인 실행 ===
if __name__ == "__main__":
    n_states = len(actions) * 4
    agent = DQNAgent(n_states, n_actions)

    n_episodes = 500
    n_steps = 20
    avg_throughputs = []
    ho_counts = []

    for episode in range(n_episodes):
        state_idx = 0
        prev_action = 0
        prev_thr = 0.0
        episode_throughputs = []
        handover_count = 0

        for step in range(n_steps):
            state_vec = np.zeros(n_states)
            state_vec[state_idx] = 1.0

            action = agent.select_action(state_vec)
            throughput, rssi, pkt_loss, density = run_ns3(action)

            current_bs_idx = action if action != 0 else state_idx // 4
            next_state_idx = encode_state(current_bs_idx, rssi, density)

            next_state_vec = np.zeros(n_states)
            next_state_vec[next_state_idx] = 1.0

            reward = compute_reward(throughput, pkt_loss, action, prev_action, prev_thr)

            agent.replay_buffer.push(state_vec, action, reward, next_state_vec)
            agent.update()

            # 핸드오버 카운트
            if action != prev_action and action != 0:
                handover_count += 1

            state_idx = next_state_idx
            prev_action = action
            prev_thr = throughput  # 직전 스루풋 갱신
            episode_throughputs.append(throughput)

            print(f"[Ep {episode} Step {step}] action={actions[action]}, thr={throughput:.3f} Mbps, rssi={rssi:.1f}, loss={pkt_loss}, reward={reward:.3f}")

        # 에피소드 평균 스루풋 & 핸드오버 횟수 기록
        avg_thr = np.mean(episode_throughputs)
        avg_throughputs.append(avg_thr)
        ho_counts.append(handover_count)

        print(f"=== Episode {episode} avg throughput = {avg_thr:.3f} Mbps, handovers = {handover_count} ===")
        print("-" * 50)

        # epsilon decay (더 빠르게 줄어듦)
        agent.epsilon = max(0.05, agent.epsilon * 0.97)

    # === 스루풋 그래프 저장 ===
    plt.figure(figsize=(8, 5))
    plt.plot(range(n_episodes), avg_throughputs, color='blue', linestyle='-')
    plt.title("DQN: Average Throughput (Modified)")
    plt.xlabel("Episode")
    plt.ylabel("Throughput (Mbps)")
    plt.grid(True)
    plt.savefig("throughput_dqn.png")
    print("스루풋 그래프를 throughput_dqn.png 파일로 저장했습니다!")

    np.save("throughput_dqn.npy", avg_throughputs)
    print("스루풋 데이터를 throughput_dqn.npy로 저장했습니다!")

    # === 핸드오버 빈도 그래프 저장 ===
    plt.figure(figsize=(8, 5))
    plt.plot(range(n_episodes), ho_counts, color='red', linestyle='-')
    plt.title("DQN: Handover Count (Modified)")
    plt.xlabel("Episode")
    plt.ylabel("Number of Handovers")
    plt.grid(True)
    plt.savefig("handover_dqn.png")
    print("핸드오버 그래프를 handover_dqn.png 파일로 저장했습니다!")

    np.save("handover_dqn.npy", ho_counts)
    print("핸드오버 데이터를 handover_dqn.npy로 저장했습니다!")
