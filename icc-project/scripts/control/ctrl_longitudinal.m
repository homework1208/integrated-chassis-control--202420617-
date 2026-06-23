function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 + ABS)  v6
%
%   v5 문제 분석:
%   - stoppingDistance=68.6m (목표 <60m)
%   - muUtil=0.948인데 왜 68m? → 초반 1~2초 잠김 구간
%   - t=1.0s: brake 인가, ax=0, wheelSlip=0 → is_braking=false → ABS 미작동
%   - 이 초반 구간에서 바퀴가 잠겨 mu_kinetic<mu_peak → 거리 손실
%
%   v6 핵심 개선:
%   1. prev_vx로 감속 감지 → brake 시작 즉시 ABS 작동
%   2. absSlipRMS: slip -0.12±0.02 정밀 유지

    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0; end
    if ~isfield(ctrlState, 'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState, 'absScale');  ctrlState.absScale  = ones(4,1); end
    if ~isfield(ctrlState, 'prev_vx');   ctrlState.prev_vx   = vx; end

    mass = 1500;

    %% PI 속도 추종
    err_v = vxRef - vx;
    ctrlState.intError = ctrlState.intError + err_v * dt;
    ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    Fx_pi = CTRL.LON.Kp * err_v + CTRL.LON.Ki * ctrlState.intError;
    Fx_max = LIM.MAX_AX * mass;
    Fx_pi = max(-Fx_max, min(Fx_max, Fx_pi));

    dF_max = LIM.MAX_JERK * mass * dt;
    dF = Fx_pi - ctrlState.prevForce;
    if abs(dF) > dF_max
        Fx_pi = ctrlState.prevForce + sign(dF) * dF_max;
    end
    ctrlState.prevForce = Fx_pi;

    %% ABS v6
    kappa_opt_lo = -0.14;
    kappa_opt_hi = -0.10;
    kappa_lock   = -0.17;

    wheelSlip = ctrlState.wheelSlip;
    absScale  = ctrlState.absScale;

    % 브레이크 감지 개선: vx 감소 감지 (ax 지연 없이 즉시 반응)
    vx_decreasing = (vx < ctrlState.prev_vx - 0.01);  % 10mm/s 이상 감속
    ctrlState.prev_vx = vx;

    is_braking = vx_decreasing || (ax < -0.3) || any(wheelSlip < -0.04);

    if is_braking && vx > 1.0
        for w = 1:4
            kw = wheelSlip(w);
            if kw < kappa_lock
                % 완전 잠김 → 빠르게 해제
                absScale(w) = absScale(w) * 0.68;
            elseif kw < kappa_opt_lo
                % 과잠김 → 서서히 해제
                absScale(w) = absScale(w) * 0.91;
            elseif kw >= kappa_opt_lo && kw <= kappa_opt_hi
                % 최적 구간 → 매우 천천히 복귀
                absScale(w) = absScale(w) + 0.003;
            else
                % 슬립 부족 → 복귀
                absScale(w) = absScale(w) + 0.012;
            end
            absScale(w) = max(0.08, min(1.0, absScale(w)));
        end
    else
        % 제동 안 할 때는 scale 완전 복귀
        absScale = ones(4,1);
    end

    ctrlState.absScale = absScale;

    forceCmd.Fx_total   = Fx_pi;
    forceCmd.brakeRatio = max(0, min(1, abs(Fx_pi)/max(Fx_max,1)));
    forceCmd.absScale   = absScale;

end
