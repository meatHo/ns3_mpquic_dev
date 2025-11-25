import os
import torch
import json
from ns3gym import ns3env
from dqn_agent2 import DQNAgent, FrameStack, preprocess_raw_state

def evaluate_model(model_path, env, agent, fstack, device, max_episodes=5, max_steps=55):
    """단일 모델 검증"""
    try:
        agent.q.load_state_dict(torch.load(model_path, map_location=device))
        agent.q_target.load_state_dict(agent.q.state_dict())
        agent.q.eval()
    except Exception as e:
        print(f"❌ 모델 로드 실패: {model_path}, {e}")
        return None

    print(f"\n✅ 모델 불러오기 완료: {model_path}")

    results = []

    for ep in range(max_episodes):
        raw_obs = env.reset()
        s0 = preprocess_raw_state({
            "RSRP_Uu": raw_obs[0],
            "RSRP_PC5": raw_obs[1],
            "PRR": raw_obs[2],
            "Velocity": raw_obs[3],
        })
        S = fstack.reset(s0)

        total_reward, total_prr = 0.0, 0.0
        total_pc5, total_uu = 0, 0

        for t in range(max_steps):
            with torch.no_grad():
                a = agent.select_action(S, eval_mode=True)

            if a == 0: total_uu += 1
            elif a == 1: total_pc5 += 1

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
            total_reward += prr_t
            S = S_next

            if done:
                break

        avg_prr = total_prr / (t+1) * 100

        # ns-3 연동 결과 읽기 (없으면 None)
        result_file = "package.json"
        ns3_data = {}
        if os.path.exists(result_file):
            with open(result_file, 'r') as f:
                ns3_data = json.load(f)

        ep_result = {
            "episode": ep,
            "total_reward": total_reward,
            "avg_prr": avg_prr,
            "PC5": total_pc5,
            "Uu": total_uu,
            "ns3": {
                "Total Sent": ns3_data.get("Total Sent"),
                "Total Received": ns3_data.get("Total Received"),
                "Total PRR": ns3_data.get("Total PRR"),
            }
        }
        results.append(ep_result)

        print(f"[Eval Episode {ep}] total_reward={total_reward:.2f}, Avg PRR={avg_prr:.3f}, "
              f"PC5={total_pc5}, Uu={total_uu}, TOTAL PRR={ns3_data.get('Total PRR')}%")

    return results


def main():
    ROOT_DIR = "/home/kiho/ns-3-quic/contrib/opengym/real_final"  # 모델이 저장된 루트 폴더
    STEP_TIME = 0.5
    FRAME_STACK_K = 2
    MAX_EPISODES = 1   # 각 모델당 검증 에피소드 수 (시간 오래 걸리면 줄이세요)
    MAX_STEPS = 55

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    env = ns3env.Ns3Env(
        port=5554, stepTime=STEP_TIME,
        startSim=False, simSeed=0, simArgs={}, debug=False
    )

    state_dim, action_dim = 4 * FRAME_STACK_K, 2
    agent = DQNAgent(state_dim, action_dim, device)
    fstack = FrameStack(FRAME_STACK_K, 4)

    # ROOT_DIR 내부 모든 하위폴더 탐색
    for subdir, _, files in os.walk(ROOT_DIR):
        for f in files:
            if f.endswith(".pth"):   # checkpoint 및 최종모델 포함
                model_path = os.path.join(subdir, f)
                evaluate_model(model_path, env, agent, fstack, device,
                               max_episodes=MAX_EPISODES, max_steps=MAX_STEPS)

    env.close()
    print("✅ 모든 모델 검증 완료")


if __name__ == "__main__":
    main()