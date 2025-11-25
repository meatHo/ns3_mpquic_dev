from ns3gym import ns3env
import torch
import json
import os
from dqn_agent import (
    DQNAgent, FrameStack,
    preprocess_raw_state, compute_reward
)

def main():
    PORT = 5554
    STEP_TIME = 1
    FRAME_STACK_K = 2
    MAX_EPISODES = 200
    MAX_STEPS = 55

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    env = ns3env.Ns3Env(
        port=PORT, stepTime=STEP_TIME,
        startSim=False, simSeed=0, simArgs={}, debug=False
    )

    state_dim = 4 * FRAME_STACK_K # latency 뺐으니까 4
    action_dim = 2
    agent = DQNAgent(state_dim, action_dim, device)
    fstack = FrameStack(FRAME_STACK_K, 4)

    for ep in range(MAX_EPISODES):
        raw_obs = env.reset()
        s0 = preprocess_raw_state({
            "RSRP_Uu": raw_obs[0],
            "RSRP_PC5": raw_obs[1],
            "PRR": raw_obs[2],
            "Velocity": raw_obs[3],
            # "Latency": raw_obs[4],
        })
        S = fstack.reset(s0)
        prr_tm1 = s0[2]

        total_reward = 0
        last_actions = [None, None]
        total_prr = 0.0
        # total_latency = 0.0
        total_pc5 = 0
        total_uu = 0
        for t in range(MAX_STEPS):
            a = agent.select_action(S)
            if a==0:
                total_uu+=1
            elif a==1:
                total_pc5+=1
            # switch_pen = compute_switch_penalty(a, last_actions)
            # print(f"[DEBUG] STEP {t} - Sending action {a} to ns-3")
            raw_next, _, done, _ = env.step(a)
            s_next = preprocess_raw_state({
                "RSRP_Uu": raw_next[0],
                "RSRP_PC5": raw_next[1],
                "PRR": raw_next[2],
                "Velocity": raw_next[3],
                # "Latency": raw_next[4],
            })
            S_next = fstack.push(s_next)

            prr_t = s_next[2]
            # lat_ms = s_next[4]
            r = compute_reward(prr_t, prr_tm1)
            prr_tm1 = prr_t

            total_prr += prr_t
            # total_latency += lat_ms

            agent.replay.push(S, a, r, S_next, done)
            if len(agent.replay)>1999:
                agent.update()

            total_reward += r
            S = S_next
            if done:
                break

        # target network 동기화
        if ep % 5 == 0:
            agent.update_target()

        # 체크포인트 저장
        if ep % 10 == 0:
            ckpt_name = f"./real_final2/past/checkpoint_ep{ep}.pth"
            torch.save(agent.q.state_dict(), ckpt_name)
            print(f"💾 체크포인트 저장: {ckpt_name}")

        num_steps = t + 1
        avg_prr = total_prr / num_steps * 100
        # avg_latency = total_latency / num_steps
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
        try:
            total_prr_value = float(data.get("Total PRR", 0))
            if total_prr_value >= 95.0:
                ckpt_name = f"./real_final2/past/checkpoint_ep{ep}_highprr.pth"
                torch.save(agent.q.state_dict(), ckpt_name)
                print(f"💾 조건 만족 (TOTAL PRR={total_prr_value}%) → 모델 저장: {ckpt_name}")
        except (ValueError, TypeError):
            print("⚠️ TOTAL PRR 값을 숫자로 변환할 수 없음")
    # 최종 모델 저장
    torch.save(agent.q.state_dict(), "./real_final2/past/dqn_ns3_final.pth")
    print("✅ 최종 모델 저장 완료: dqn_ns3_final.pth")

    env.close()

if __name__ == "__main__":
    main()