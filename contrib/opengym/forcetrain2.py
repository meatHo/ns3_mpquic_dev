from dqn_agent1 import DQNAgent, preprocess_raw_state, FrameStack
import torch
import numpy as np

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 환경 state_dim과 action_dim은 기존과 동일하게
    state_dim = 4 * 2   # FRAME_STACK_K=2 (fine_tune1.py 기준)
    action_dim = 2
    agent = DQNAgent(state_dim, action_dim, device)

    # 기존 학습된 모델 불러오기
    model_path = "/home/kiho/ns-3-quic/contrib/opengym/real_final/fine1/checkpoint_ep80.pth"
    agent.q.load_state_dict(torch.load(model_path, map_location=device))
    agent.q_target.load_state_dict(agent.q.state_dict())
    print(f"✅ 모델 로드 완료: {model_path}")

    # === 강제 학습 시킬 데이터 정의 ===
    # 예: 특정 RSRP, PRR 값에서 action=1(PC5 선택)을 강제
    forced_cases = [
    {
        # === 첫 번째 케이스 (14~16초, 17초에 PRR 급락) ===
        "prev": {"RSRP_Uu": -86.4914, "RSRP_PC5": -102.974, "PRR": 1, "Velocity": 12.4016},
        "curr": {"RSRP_Uu": -89.5942, "RSRP_PC5": -103.457, "PRR": 0.994444, "Velocity": 13.1034},
        "next": {"RSRP_Uu": -90.0803, "RSRP_PC5": -105.72, "PRR": 0.0422222, "Velocity": 13.1034},
        "action": 0   # UU 선택을 강화
    },
    {
        # === 두 번째 케이스 (30~32초, 33초에 PRR 급락) ===
        "prev": {"RSRP_Uu": -97.9508, "RSRP_PC5": -101.115, "PRR": 1.0, "Velocity": 13.1034},
        "curr": {"RSRP_Uu": -100.976, "RSRP_PC5": -102.486, "PRR": 1.0, "Velocity": 13.1034},
        "next": {"RSRP_Uu": -104.391, "RSRP_PC5": -102.309, "PRR": 0.642222, "Velocity": 13.1034},
        "action": 0   # UU 선택을 강화
    }
]





    # === ReplayBuffer에 삽입 ===
    # === ReplayBuffer에 삽입 ===
    for case in forced_cases:
        s_prev = preprocess_raw_state(case["prev"])
        s_curr = preprocess_raw_state(case["curr"])
        s_next = preprocess_raw_state(case["next"])

        S = np.concatenate([s_prev, s_curr])         # (prev + curr)
        S_next = np.concatenate([s_curr, s_next])    # (curr + next)

        a = case["action"]
        r = 100.0   # 보상 크기는 100~200 정도로도 충분
        done = False

        agent.replay.push(S, a, r, S_next, done)


    # === 강제 업데이트 반복 ===
    print("🚀 강제 학습 시작")
    for i in range(500):  # 충분히 반복
        agent.update(batch_size=2)
        if i % 50 == 0:
            print(f"Iteration {i} - 업데이트 진행 중...")

    # === 저장 ===
    save_path = "./real_final/fine1/dqn_ns3_forced1.pth"
    torch.save(agent.q.state_dict(), save_path)
    print(f"💾 강제 학습된 모델 저장 완료: {save_path}")

if __name__ == "__main__":
    main()


# ExecuteActions called at 14.0117 with action=1 몇번쨰? 10
# sent : 100  recv : 90
# Time 15.0117s
# UE 0 PRR=1
# Uu RSRP(avg)=-96.3264 dB
# Sl RSRP(avg)=-107.69 dB
# UE Pos=(280.286,4590.19,59) Vel=(-0.3,-13.1,0)13.1034


# ExecuteActions called at 15.0117 with action=1 몇번쨰? 11
# sent : 100  recv : 89
# Time 16.0117s
# UE 0 PRR=0.988889
# Uu RSRP(avg)=-103.478 dB
# Sl RSRP(avg)=-104.956 dB
# UE Pos=(280.046,4577.07,59) Vel=(-0.3,-13.1,0)13.1034


# ExecuteActions called at 16.0117 with action=1 몇번쨰? 12
# sent : 100  recv : 24
# Time 17.0117s
# UE 0 PRR=0.266667
# Uu RSRP(avg)=-100.4 dB
# Sl RSRP(avg)=-108.312 dB
# UE Pos=(279.806,4563.98,59) Vel=(-0.3,-13.1,0)13.1034




# ExecuteActions called at 30.0156 with action=1 몇번쨰? 26
# sent : 100  recv : 90
# Time 31.0117s
# UE 0 PRR=1
# Uu RSRP(avg)=-104.27 dB
# Sl RSRP(avg)=-93.8063 dB
# UE Pos=(276.446,4380.51,60) Vel=(-0.3,-13.1,0)13.1034


# ExecuteActions called at 31.0117 with action=1 몇번쨰? 27
# sent : 100  recv : 89
# Time 32.0117s
# UE 0 PRR=0.988889
# Uu RSRP(avg)=-102.38 dB
# Sl RSRP(avg)=-91.8736 dB
# UE Pos=(276.206,4367.41,60) Vel=(-0.3,-13.2,0)13.2034


# ExecuteActions called at 32.0117 with action=1 몇번쨰? 28
# sent : 100  recv : 60
# Time 33.0117s
# UE 0 PRR=0.666667
# Uu RSRP(avg)=-100.152 dB
# Sl RSRP(avg)=-89.3781 dB
# UE Pos=(275.966,4354.32,60) Vel=(-0.3,-13.1,0)13.1034