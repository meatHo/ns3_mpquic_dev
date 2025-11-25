from ns3gym import ns3env
import torch
import json
import math

# ==== 간단 L3 필터(지수평활) ====
class L3Filter:
    def __init__(self, tau_sec=0.2, Ts=0.5):
        # tau: L3 필터 시정수(초), Ts: 샘플 주기(초)
        # y_k = alpha * y_{k-1} + (1-alpha) * x_k, alpha = exp(-Ts/tau)
        self.alpha = math.exp(-Ts / max(tau_sec, 1e-6))
        self.y = None

    def reset(self):
        self.y = None

    def update(self, x):
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * self.y + (1 - self.alpha) * x
        return self.y

# ==== Event A3 기반 핸드오버 컨트롤러 ====
class A3Controller:
    def __init__(self,
                 step_time_sec=0.5,
                 hysteresis_db=3.0,
                 ttt_ms=80,
                 cio_serv_db=0.0,
                 cio_neig_db=0.0,
                 handback_margin_db=1.0):
        """
        hysteresis_db: A3 히스테리시스 (dB)
        ttt_ms: Time-To-Trigger (ms) — 연속 유지 시간
        cio_*_db: CIO(cell individual offset), 기본 0
        handback_margin_db: 핑퐁 방지용 역방향 전환시 추가 마진
        """
        self.step_time = step_time_sec
        self.hyst = hysteresis_db
        self.ttt_sec = ttt_ms / 1000.0
        self.cio_s = cio_serv_db
        self.cio_n = cio_neig_db
        self.ho_timer = 0.0
        self.current = 0  # 0=Uu(서빙), 1=PC5(이웃)
        self.handback_margin = handback_margin_db

        # L3 필터(측정 안정화)
        self.f_uu = L3Filter(tau_sec=0.2, Ts=step_time_sec)
        self.f_pc5 = L3Filter(tau_sec=0.2, Ts=step_time_sec)

    def reset(self, init_serving=0):
        self.current = init_serving
        self.ho_timer = 0.0
        self.f_uu.reset()
        self.f_pc5.reset()

    def decide(self, rsrp_uu_dbm, rsrp_pc5_dbm):
        """
        입력: 원시 RSRP(dBm)
        출력: 액션 a (0=Uu 사용, 1=PC5 사용)
        """
        m_uu = self.f_uu.update(rsrp_uu_dbm)
        m_pc5 = self.f_pc5.update(rsrp_pc5_dbm)

        # A3 조건: neighbor + CIO_n > serving + CIO_s + Hysteresis
        # 현재 서빙이 Uu일 때: neighbor=PC5
        # 현재 서빙이 PC5일 때: neighbor=Uu (대칭적 적용하되, 역방향은 마진 추가)
        if self.current == 0:
            cond = (m_pc5 + self.cio_n) > (m_uu + self.cio_s + self.hyst)
            needed = self.ttt_sec
            if cond:
                self.ho_timer += self.step_time
                if self.ho_timer >= needed:
                    self.current = 1
                    self.ho_timer = 0.0
            else:
                self.ho_timer = 0.0
        else:
            # 역방향 전환(A3 대칭, 핑퐁 방지 위해 추가 마진)
            cond = (m_uu + self.cio_s) > (m_pc5 + self.cio_n + self.hyst + self.handback_margin)
            needed = self.ttt_sec
            if cond:
                self.ho_timer += self.step_time
                if self.ho_timer >= needed:
                    self.current = 0
                    self.ho_timer = 0.0
            else:
                self.ho_timer = 0.0

        return self.current  # 0=Uu, 1=PC5


def preprocess_raw_state(obs_dict):
    # 사용자가 기존 쓰던 전처리와 동일(지연 미사용)
    import numpy as np
    uu  = np.clip((obs_dict["RSRP_Uu"]+95)/15, -1.0, 1.0)
    pc5 = np.clip((obs_dict["RSRP_PC5"]+95)/15, -1.0, 1.0)
    prr = obs_dict["PRR"]
    vel = np.clip(obs_dict["Velocity"]/30.0, -1.0, 1.0)
    return np.array([uu, pc5, prr, vel], dtype=np.float32)


def main():
    PORT = 5554
    STEP_TIME = 0.5
    MAX_EPISODES = 200
    MAX_STEPS = 55

    # === 환경 생성 ===
    env = ns3env.Ns3Env(
        port=PORT, stepTime=STEP_TIME,
        startSim=False, simSeed=0, simArgs={}, debug=False
    )

    # === A3 컨트롤러 설정 ===
    # 가장 흔히 쓰이는 조합: Hysteresis=3 dB, TTT=80 ms
    a3 = A3Controller(
        step_time_sec=STEP_TIME,
        hysteresis_db=3.0,
        ttt_ms=80,
        cio_serv_db=0.0,
        cio_neig_db=0.0,
        handback_margin_db=1.0
    )

    for ep in range(MAX_EPISODES):
        raw_obs = env.reset()

        # A3 초기화(초기 서빙: Uu=0 가정)
        a3.reset(init_serving=0)

        # 통계
        total_prr = 0.0
        total_pc5 = 0
        total_uu  = 0
        total_steps = 0

        # 첫 상태 처리
        s0 = preprocess_raw_state({
            "RSRP_Uu":   raw_obs[0],
            "RSRP_PC5":  raw_obs[1],
            "PRR":       raw_obs[2],
            "Velocity":  raw_obs[3],
        })

        for t in range(MAX_STEPS):
            # === A3 의사결정 ===
            rsrp_uu  = raw_obs[0]   # dBm
            rsrp_pc5 = raw_obs[1]   # dBm
            a = a3.decide(rsrp_uu, rsrp_pc5)   # 0=Uu, 1=PC5

            if a == 0: total_uu  += 1
            else:      total_pc5 += 1

            # === 스텝 진행 ===
            raw_next, _, done, _ = env.step(a)
            if raw_next is None:
                break

            # 통계
            total_steps += 1
            total_prr   += raw_next[2]

            raw_obs = raw_next  # 다음 루프 입력

            if done:
                break

        # 결과 출력
        if total_steps > 0:
            avg_prr = (total_prr / total_steps) * 100.0
        else:
            avg_prr = 0.0

        print(f"[Episode {ep}] Avg PRR={avg_prr:.3f}%, PC5={total_pc5}, Uu={total_uu}")

        # (선택) ns-3가 생성하는 JSON 리포트 읽기
        result_file = "package.json"
        try:
            with open(result_file, 'r') as f:
                data = json.load(f)
            print(f"TOTAL SENT={data.get('Total Sent')}, TOTAL RECEIVED={data.get('Total Received')}, TOTAL PRR={data.get('Total PRR')}%")
        except FileNotFoundError:
            pass

    env.close()


if __name__ == "__main__":
    main()
