from ns3gym import ns3env
import torch
import json
import os
from dqn_agent2 import (
    DQNAgent, FrameStack,
    preprocess_raw_state
)

def main():
    PORT = 5554
    STEP_TIME = 0.5
    FRAME_STACK_K = 2
    MAX_EPISODES = 20   # 검증용으로 20회만 돌려보기
    MAX_STEPS = 55

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    env = ns3env.Ns3Env(
        port=PORT, stepTime=STEP_TIME,
        startSim=False, simSeed=0, simArgs={}, debug=False
    )

    state_dim = 4 * FRAME_STACK_K
    action_dim = 2
    agent = DQNAgent(state_dim, action_dim, device)
    fstack = FrameStack(FRAME_STACK_K, 4)

    # ========================================================== #
    # 저장된 모델 불러오기 (반드시 학습에서 저장한 것과 동일 구조)
    # model_path = "./test/900_90_fine_reward/dqn_ns3_final_fine.pth"
    model_path = "/home/kiho/ns-3-quic/contrib/opengym/real_final2/past/realfinal2.pth"
    try:
        agent.q.load_state_dict(torch.load(model_path, map_location=device))
        agent.q_target.load_state_dict(agent.q.state_dict())
        print(f"✅ 모델 '{model_path}' 불러오기 완료 (검증 모드)")
    except FileNotFoundError:
        print(f"❌ 모델 파일 없음: {model_path}")
        return
    # ========================================================== #

    agent.q.eval()  # 평가 모드 (Dropout/BatchNorm 등 있을 경우 필요)

    for ep in range(MAX_EPISODES):
        raw_obs = env.reset()
        s0 = preprocess_raw_state({
            "RSRP_Uu": raw_obs[0],
            "RSRP_PC5": raw_obs[1],
            "PRR": raw_obs[2],
            "Velocity": raw_obs[3],
        })
        S = fstack.reset(s0)

        total_reward = 0
        total_prr = 0.0
        total_pc5 = 0
        total_uu = 0

        for t in range(MAX_STEPS):
            # 학습이 아니라 검증이므로 exploration 없이 행동 선택
            with torch.no_grad():
                a = agent.select_action(S, eval_mode=True)
                # 여기서 eval_mode=True는 select_action() 안에서
                # ε-greedy 대신 argmax 쓰도록 수정해주는 게 좋음

            if a == 0:
                total_uu += 1
            elif a == 1:
                total_pc5 += 1

            raw_next, _, done, _ = env.step(a)
            s_next = preprocess_raw_state({
                "RSRP_Uu": raw_next[0],
                "RSRP_PC5": raw_next[1],
                "PRR": raw_next[2],
                "Velocity": raw_next[3],
            })
            S_next = fstack.push(s_next)

            prr_t = s_next[2]
            total_prr += prr_t
            total_reward += prr_t  # 검증에서는 보상함수 대신 PRR 자체 누적도 가능

            S = S_next
            if done:
                break

        num_steps = t + 1
        avg_prr = total_prr / num_steps * 100

        result_file = "package.json"
        data = {}
        try:
            with open(result_file, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            print(f"⚠️ package.json 파일 없음 (ns-3 결과 미연동)")

        print(f"[Eval Episode {ep}] total_reward={total_reward:.2f}, Avg PRR={avg_prr:.3f}, PC5={total_pc5}, Uu={total_uu}")
        print(f"TOTAL SENT={data.get('Total Sent')}, TOTAL RECEIVED={data.get('Total Received')}, TOTAL PRR={data.get('Total PRR')}%")

    env.close()
    print("✅ 검증 완료")

if __name__ == "__main__":
    main()
