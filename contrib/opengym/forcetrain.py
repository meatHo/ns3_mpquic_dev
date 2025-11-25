from dqn_agent2 import DQNAgent, preprocess_raw_state, FrameStack
import torch
import numpy as np

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 환경 state_dim과 action_dim은 기존과 동일하게
    state_dim = 4 * 2   # FRAME_STACK_K=2 (fine_tune1.py 기준)
    action_dim = 2
    agent = DQNAgent(state_dim, action_dim, device)

    # 기존 학습된 모델 불러오기
    model_path = "./test/prr_triple/fine98/fine99/dqn_ns3_final_fine2.pth"
    agent.q.load_state_dict(torch.load(model_path, map_location=device))
    agent.q_target.load_state_dict(agent.q.state_dict())
    print(f"✅ 모델 로드 완료: {model_path}")

    # === 강제 학습 시킬 데이터 정의 ===
    # 예: 특정 RSRP, PRR 값에서 action=1(PC5 선택)을 강제
    forced_cases = [
        {
            "obs": {"RSRP_Uu": -99.1302, "RSRP_PC5": -85.0076, "PRR": 0.488889, "Velocity": 13.1034},
            "action": 1   # PC5를 선택하도록 강제
        },
        {
            "obs": {"RSRP_Uu": -75, "RSRP_PC5": -120, "PRR": 0.95, "Velocity": 1.0},
            "action": 0   # Uu를 선택하도록 강제
        },
    ]

    # === ReplayBuffer에 삽입 ===
    for case in forced_cases:
        s = preprocess_raw_state(case["obs"])
        S = np.concatenate([s, s])  # FrameStack=2라서 동일 state 2번 concat
        a = case["action"]
        r = 10.0   # 보상을 크게 줘서 강하게 학습
        S_next = S
        done = False
        agent.replay.push(S, a, r, S_next, done)

    # === 강제 업데이트 반복 ===
    print("🚀 강제 학습 시작")
    for i in range(200):  # 충분히 반복
        agent.update(batch_size=2)
        if i % 50 == 0:
            print(f"Iteration {i} - 업데이트 진행 중...")

    # === 저장 ===
    save_path = "./test/prr_triple/fine98/dqn_ns3_forced.pth"
    torch.save(agent.q.state_dict(), save_path)
    print(f"💾 강제 학습된 모델 저장 완료: {save_path}")

if __name__ == "__main__":
    main()
