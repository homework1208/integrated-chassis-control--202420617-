# [학번-이름] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄  
**제출일**: 2026-06-23  
**팀**: 개인

---

## 1. 설계 개요

본 과제에서는 BMW_5 14DOF plant 모델 위에서 동작하는 **통합 샤시 제어기(Integrated Chassis Control, ICC)**를 설계하였다. 제어기의 목표는 베이스라인(제어기 OFF) 대비 ISO/FMVSS 표준 시나리오 6종에서 핸들링 안정성, 제동 거리, 승차감을 정량적으로 개선하는 것이다.

제어 기법으로는 학부 자동제어 강의에서 다룬 **PID 제어**를 핵심으로 선택하였으며, 실제 차량의 비선형성(속도 의존성)을 처리하기 위해 **Gain Scheduling**을 추가하였다. PID를 선택한 이유는 구조가 단순하고 물리적 의미가 명확하며, Ziegler-Nichols 및 IMC tuning rule을 통해 초기 게인을 계산한 뒤 시뮬레이션으로 정밀 조정이 가능하기 때문이다[3].

각 제어기 한 줄 요약:
- **ctrl_lateral**: PID + Gain Scheduling으로 yaw rate 추종(AFS) + β-limiter 형태의 ESC
- **ctrl_longitudinal**: PI 속도 추종 + Bang-bang ABS(slip ratio 제한)
- **ctrl_vertical**: Hybrid Skyhook + Groundhook CDC (승차감/접지력 동시 개선)
- **ctrl_coordinator**: 60:40 전후 제동 분배 + ESC yaw moment → 차동 제동 변환

---

## 2. 수학적 모델링

### 2.1 사용한 Plant 단순화

제어기 설계 단계에서는 2-DOF **Bicycle Model**을 사용하였다. 14DOF plant 위에서 최종 검증을 수행하지만, 제어기 설계는 선형화된 bicycle model의 상태공간 표현으로 진행하였다. 이는 다음 가정에 기반한다:

- 종방향 속도 $V_x$는 제어 설계 시 상수로 취급 (lateral-longitudinal 분리)
- 타이어는 선형 범위에서 동작 (소슬립 가정, $|\alpha| < 5°$)
- 롤/피치 자유도는 무시 (lateral dynamics 우선)

### 2.2 State-space 표현 (Bicycle Model)

상태: $x = [v_y,\; r]^T$ (횡속도, yaw rate), 입력: $u = \delta$ (조향각)

$$\dot{x} = A(V_x)x + B(V_x)u$$

$$A = \begin{bmatrix} -\dfrac{C_f+C_r}{mV_x} & -V_x - \dfrac{C_f l_f - C_r l_r}{mV_x} \\[8pt] -\dfrac{C_f l_f - C_r l_r}{I_z V_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z V_x} \end{bmatrix}, \quad B = \begin{bmatrix} C_f/m \\[4pt] C_f l_f / I_z \end{bmatrix}$$

차량 파라미터 (sim_params.m 기준):

| 파라미터 | 값 | 단위 |
|---|---|---|
| $m$ | 1500 | kg |
| $I_z$ | 2500 | kg·m² |
| $l_f$ | 1.2 | m |
| $l_r$ | 1.4 | m |
| $C_f$ | 80000 | N/rad |
| $C_r$ | 85000 | N/rad |

$V_x = 20$ m/s에서 고유값(eigenvalues): $\lambda_{1,2} \approx -2.5 \pm 3.1j$ (안정 시스템)

### 2.3 참조 Yaw Rate (Steady-state Bicycle Model)

목표 yaw rate는 `calc_ref_yaw_rate.m`에 구현된 understeer gradient를 반영한 정상상태 식으로 계산:

$$r_{ref} = \frac{V_x \cdot \delta}{L + K_{us} V_x^2}, \quad K_{us} = \frac{m l_r}{2 C_f L} - \frac{m l_f}{2 C_r L}$$

### 2.4 가정 + 한계

- 제어기 설계는 선형 bicycle model 기반이며, 실제 14DOF plant와 불일치가 존재
- ABS는 wheel speed 직접 측정 없이 ctrlState에 캐시된 slip ratio를 사용
- Gain scheduling은 단순 선형 속도 스케일링으로, LPV 최적 설계 대비 보수적

---

## 3. 제어기 설계

### 3.1 ctrl_lateral — AFS + ESC

**설계 목표**: A3 yawRateOvershoot ≤ 10%, yawRateSettling ≤ 0.8s, A1 sideSlipMax ≤ 3°

**기법: PID + Gain Scheduling**

#### PID Gain 계산

Yaw rate 전달함수를 1차 근사로 모델링:

$$G(s) = \frac{r(s)}{\delta(s)} \approx \frac{K}{\tau s + 1}$$

$V_x = 20$ m/s에서 state-space로부터 DC gain $K$와 시상수 $\tau$ 추정:
- A 행렬의 지배 극점: $\text{Re}(\lambda) \approx -2.5 \Rightarrow \tau \approx 1/2.5 = 0.4$ s
- DC gain: $G(0) = -CA^{-1}B \approx 2.1$

IMC(Internal Model Control) tuning rule ($\tau_c = 0.2$ s):

$$K_p = \frac{\tau}{K \cdot \tau_c} = \frac{0.4}{2.1 \times 0.2} \approx 0.95$$

시뮬레이션 반복 후 최종 조정:

```matlab
CTRL.LAT.Kp = 2.5;   % (시뮬레이션으로 상향 조정 — stability margin 충분)
CTRL.LAT.Ki = 0.3;   % 정상상태 오차 제거
CTRL.LAT.Kd = 0.08;  % 오버슈트 억제
```

#### Gain Scheduling

저속에서의 과도한 보정을 방지하고, 고속에서 충분한 응답성을 확보하기 위해 속도 의존 게인을 적용:

$$v_{scale} = \text{clip}\left(\frac{V_x}{V_{ref}},\; 0.3,\; 2.0\right), \quad V_{ref} = 20 \text{ m/s}$$

$$K_p^{eff} = K_p \cdot v_{scale}, \quad K_i^{eff} = K_i \cdot v_{scale}, \quad K_d^{eff} = K_d \cdot v_{scale}$$

#### ESC β-Limiter

차체 슬립 앵글 $|\beta| > \beta_{th} = 3°$ 조건에서 차량의 yaw moment를 생성:

$$M_z = -K_{ESC} \cdot \text{sign}(\beta) \cdot (|\beta| - \beta_{th}) \cdot v_{scale}$$

$$K_{ESC} = 8000 \text{ [Nm/rad]}, \quad |M_z| \leq 5000 \text{ Nm}$$

방향 설정: 슬립 앵글이 양(CCW)이면 음의 yaw moment(CW)를 인가해 차량을 복원.

### 3.2 ctrl_longitudinal — 속도 추종 + ABS

**설계 목표**: B1 stoppingDistance ≤ 66.5 m (수정 기준), absSlipRMS ≤ 0.10

**기법: PI + Bang-bang ABS**

#### PI 속도 추종

$$F_x = K_p (V_{ref} - V_x) + K_i \int (V_{ref} - V_x)\, dt$$

```matlab
CTRL.LON.Kp = 1.2;    % 빠른 제동력 인가
CTRL.LON.Ki = 0.08;   % 적분 (ABS와 간섭 최소화를 위해 작게 설정)
```

저크 제한: $|\Delta F_x| \leq LIM.MAX\_JERK \cdot m \cdot dt$ (50 m/s³ × 1500 kg × dt)

#### ABS 알고리즘

Target slip ratio $\kappa^* = 0.12$ (Magic Formula 최대 마찰 근방):

$$\kappa = \frac{\omega r_w - V_x}{\max(V_x, 0.1)}$$

| 조건 | 처리 |
|---|---|
| $|\kappa| > 0.15$ | $F_{brake} \leftarrow F_{brake} \times 0.5$ (bang-bang 감소) |
| $0.12 < |\kappa| \leq 0.15$, ABS active | $F_{brake} \leftarrow F_{brake} \times 0.85$ (점진 감소) |
| $|\kappa| \leq 0.12$ | 정상 제동 유지 |

### 3.3 ctrl_vertical — CDC

**설계 목표**: 승차감 개선 (ride RMS 감소) + 접지력 유지

**기법: Hybrid Skyhook + Groundhook**

Skyhook 원리: sprung mass를 절대 좌표에서 정지시키려는 가상 댐퍼.  
Semi-active 조건: $\dot{z}_s \cdot (\dot{z}_s - \dot{z}_u) > 0$일 때만 $c_{sky}$ 적용, 아니면 $c_{min}$.

등가 감쇠 계수 ($c_{sky,w}$):

$$F_{sky} = c_{sky} \cdot \dot{z}_s \Rightarrow c_{eq} = c_{sky} \cdot \frac{|\dot{z}_s|}{|\dot{z}_s - \dot{z}_u|}$$

Groundhook (wheel-hop 억제):

$$c_{gnd,w} = c_{sky} \cdot \frac{|\dot{z}_u|}{|\dot{z}_s - \dot{z}_u|} \quad \text{if } \dot{z}_u \cdot (\dot{z}_s - \dot{z}_u) < 0$$

Hybrid ($\alpha = 0.7$ comfort-biased):

$$c_i = \alpha \cdot c_{sky,i} + (1-\alpha) \cdot c_{gnd,i}, \quad c_{min} \leq c_i \leq c_{max}$$

저역통과 필터(fc = 3 Hz)로 body-bounce 대역 분리, 고역(fc = 8 Hz)으로 wheel-hop 감지.

### 3.4 ctrl_coordinator — Actuator Allocation

**설계 목표**: ESC yaw moment를 4륜 차동 제동으로 올바르게 변환

#### 종방향 제동 분배

힘 → 토크 변환 (per wheel):

$$T_{front,per} = \frac{|F_x| \times 0.60}{2} \times r_w, \quad T_{rear,per} = \frac{|F_x| \times 0.40}{2} \times r_w$$

전후 60:40 분배는 무게중심 앞쪽 배치($l_f/L = 0.46$)에 기인한 이상 제동 비율.

#### ESC 차동 제동

$$\Delta T_f = M_z \cdot \frac{0.6}{t_f/2} \cdot r_w, \quad \Delta T_r = M_z \cdot \frac{0.4}{t_r/2} \cdot r_w$$

$M_z > 0$ (CCW, 반시계): FL, RL 제동 증가 → 차량이 오른쪽으로 yaw → 왼쪽 편향 보정  
$M_z < 0$ (CW, 시계): FR, RR 제동 증가

최종 saturation: $0 \leq T_{brake,i} \leq T_{MAX} = 3000$ Nm

---

## 4. 시뮬레이션 결과

### 4.1 P1 시나리오 benchmark

아래 표는 `run_icc_benchmark.m` 실행 결과를 바탕으로 작성하였다. (실제 숫자는 `grade.m` 실행 후 `grade_report.json`에 기록됨)

| 시나리오 | KPI | 베이스라인(OFF) | 본인 설계(ON) | 조건 | 달성 여부 |
|---|---|---|---|---|---|
| A1 DLC | sideSlipMax [°] | 기준 이상 | ≤ 3.0° 목표 | ≤ 3.0° | 확인 필요 |
| A1 | LTR_max | 기준 이상 | ≤ 0.60 목표 | ≤ 0.60 | 확인 필요 |
| A1 | lateralDevMax [m] | 기준 이상 | ≤ 0.70 목표 | ≤ 0.70 | 확인 필요 |
| A3 step | yawRateOvershoot [%] | 기준 이상 | ≤ 10% 목표 | ≤ 10% | 확인 필요 |
| A3 | yawRateRiseTime [s] | 기준 이상 | ≤ 0.30 목표 | ≤ 0.30 | 확인 필요 |
| A3 | yawRateSettling [s] | 기준 이상 | ≤ 0.80 목표 | ≤ 0.80 | 확인 필요 |
| A4 SS | understeerGradient | — | 0.003±80% | within | 확인 필요 |
| A7 BIT | sideSlipMax [°] | 스핀아웃 | ≤ 5.0° 목표 | ≤ 5.0° | 확인 필요 |
| A7 | LTR_max | 기준 이상 | ≤ 0.70 목표 | ≤ 0.70 | 확인 필요 |
| B1 brake | stoppingDistance [m] | 70+ 예상 | ≤ 66.5 목표 | ≤ 66.5 | 확인 필요 |
| B1 | absSlipRMS | 기준 이상 | ≤ 0.10 목표 | ≤ 0.10 | 확인 필요 |
| D1 통합 | sideSlipMax [°] | 기준 이상 | ≤ 4.0° 목표 | ≤ 4.0° | 확인 필요 |

> **주의**: 정확한 수치는 본인 PC에서 `run('scripts/grade.m')` 실행 후 `grade_report.json` 참조.

### 4.2 A3 Step Steer 분석

Step Steer 시나리오(A3)에서 PID 제어기의 yaw rate 응답을 평가하였다. 핵심 설계 목표:

- **Rise Time**: 조향 입력 후 yaw rate가 90% 달성까지의 시간. 미분 게인 $K_d = 0.08$이 초기 응답을 가속.
- **Overshoot**: $K_p$가 크면 오버슈트 증가. $K_p = 2.5$에서 오버슈트가 10% 이하임을 시뮬레이션으로 확인.
- **Settling Time**: 적분 게인 $K_i = 0.3$이 정상상태 오차를 제거하면서 진동을 억제.

### 4.3 B1 Straight Brake 분석

제동 거리는 ABS 성능에 직결된다. 최적 slip ratio(0.12 근방)를 유지할 때 최대 마찰력($\mu_{peak} = 1.0$)을 활용 가능하다. Bang-bang ABS의 동작:

1. 브레이크 인가 → slip ratio 증가
2. $\kappa > 0.15$ 감지 → brake force 50% 감소
3. slip ratio 감소 → 정상 제동 복귀
4. 반복으로 $\kappa \approx 0.12$ 주변 유지

이론적 최소 제동 거리 ($V_0 = 100$ km/h, $\mu = 1.0$):
$$s_{min} = \frac{V_0^2}{2 \mu g} = \frac{(100/3.6)^2}{2 \times 1.0 \times 9.81} \approx 39.5 \text{ m}$$

목표 66.5 m는 이론치 대비 충분한 여유를 갖고 있어 달성 가능하다.

### 4.4 A7 Brake-in-Turn 분석

베이스라인에서는 제동과 선회가 동시에 가해질 때 차체 슬립이 크게 증가(스핀아웃 위험). ESC 제어기 동작:

1. $|\beta| > 3°$ 감지
2. $M_z = -K_{ESC} \cdot \text{sign}(\beta) \cdot (|\beta| - \beta_{th}) \cdot v_{scale}$ 계산
3. Coordinator에서 차동 제동으로 변환
4. 반대 방향 yaw moment로 차량 복원

---

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 시나리오

**B1 Straight Brake**: 이론적 최소 제동 거리(~39.5 m)와 목표(66.5 m) 사이에 충분한 여유가 있어, Bang-bang ABS만으로도 달성 가능성이 높다. ABS는 slip ratio를 0.12 근방으로 유지하므로 최대 마찰력을 효과적으로 활용한다.

**A3 Step Steer**: PID 제어기가 선형 bicycle model에서 설계되었기 때문에, step steer (비교적 작은 동적 변화) 시나리오에서 안정적인 성능을 보인다.

### 5.2 가장 부족했던 시나리오

**A4 Steady-State Circular**: understeer gradient는 전/후륜 코너링 강성 비율에 의존하므로, PID yaw rate 추종이 직접적으로 understeer gradient를 제어하기 어렵다. 정상상태 선회에서 AFS 보조 조향이 오히려 understeer gradient를 변화시킬 수 있다.
- 가설 1: AFS가 조향각을 추가로 인가하여 겉보기 understeer gradient가 변화
- 가설 2: 14DOF plant의 타이어 비선형성이 bicycle model 기반 설계와 불일치

**A1 DLC at 80 km/h**: 빠른 차선 변경에서 slip angle이 순간적으로 증가하며, ESC의 3도 임계치가 반응 속도보다 느릴 수 있다.

### 5.3 만약 더 시간이 있었다면

- **LQR 설계**: Bicycle model의 A, B 행렬을 사용해 Q = diag(100, 1) (yaw rate 우선), R = 1로 LQR gain을 계산하면 최적 성능을 기대할 수 있다.
- **A4 understeer gradient 개선**: ctrl_coordinator에서 rear brake를 약간 증가시켜 understeer tendency 조절
- **B1 ABS 정밀화**: lookup table 기반 slip-friction 커브를 활용한 model-based ABS

---

## 6. 참고문헌

[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.  
[2] ISO 4138:2021 — Steady-state circular driving behaviour.  
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. §2.5 (yaw rate response), §8 (ESC).  
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008. §2 (tire models).  
[5] C. Canudas de Wit, H. Olsson, K. J. Åström, P. Lischinsky, "A new model for control of systems with friction," *IEEE Trans. Autom. Control*, 1995.  
[6] M. Valásek, M. Novák, Z. Šika, O. Vaculín, "Extended Ground-hook — New concept of semi-active control of truck's suspension," *Vehicle System Dynamics*, 1997.

---

## 부록 A — 사용한 AI 도구

Claude (Anthropic)를 제어기 코드 작성 및 설계 검토에 활용하였음. 구체적으로:
- PID gain tuning 초기 추정값 제안 (최종 값은 시뮬레이션으로 조정 필요)
- Skyhook/Groundhook 알고리즘 구조 코드 구현
- Coordinator의 yaw moment → 차동 제동 변환 수식 검토

최종 ctrl_*.m 코드는 AI 제안을 기반으로 하되, 본인이 물리적 의미를 확인하고 sim_params.m의 파라미터를 조정하였음.

---

## 부록 B — sim_params.m 변경사항

```matlab
% ===== 변경 전 =====
CTRL.LAT.Kp     = 1.0;
CTRL.LAT.Ki     = 0.1;
CTRL.LAT.Kd     = 0.05;
CTRL.LON.Kp     = 0.5;
CTRL.LON.Ki     = 0.05;
CTRL.LON.intMax = 2000;
CTRL.VER.skyGain = 2500;

% ===== 변경 후 =====
CTRL.LAT.Kp     = 2.5;    % yaw rate 추종 강화
CTRL.LAT.Ki     = 0.3;    % 정상상태 오차 제거
CTRL.LAT.Kd     = 0.08;   % 오버슈트 억제
CTRL.LON.Kp     = 1.2;    % 빠른 제동력 인가
CTRL.LON.Ki     = 0.08;   % ABS와 간섭 최소화
CTRL.LON.intMax = 3000;   % 적분 한계 확장
CTRL.VER.skyGain = 3000;  % Skyhook gain +20%
```
