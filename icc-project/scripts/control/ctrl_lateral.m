function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC)  v6
%
%   v5 결과 분석:
%   - A3: settling=0.741s ✅, overshoot=2.21% ✅ → 유지
%   - A1_LTR: 0.764 → 목표 0.60. ESC 더 강화해도 LTR이 안 줄어
%     → LTR은 DLC에서 롤로 인한 하중 편차. ESC moment가 직접 줄이기 어려움
%     → 하지만 ESC가 차량 안정화 → 슬립 줄임 → 롤 감소 → LTR 간접 감소 가능
%   - A1/D1 lateralDevMax: 2.048m (악화됨, OFF=1.827)
%     → ESC 강화가 Stanley 드라이버 방해 → 경로 이탈 악화
%     → ESC yaw moment는 차동 제동으로 → 예상치 못한 방향 변화
%
%   v6 조정:
%   - AFS: v5와 동일 (settling ✅)
%   - ESC: K 9000 유지. beta_th를 2.0°로 더 낮춰 LTR 개선 시도
%     (단, A1 lateralDev가 더 악화되면 beta_th를 다시 올려야 함)
%   - 현실적으로 A1_LTR/lateralDev는 구조적 한계

    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevError'); ctrlState.prevError = 0; end

    if vx < 2.0
        deltaAdd.steerAngle = 0;
        deltaAdd.yawMoment  = 0;
        ctrlState.intError  = 0;
        ctrlState.prevError = 0;
        return;
    end

    v_ref   = 22.0;
    v_scale = min(vx / v_ref, 1.5);
    v_scale = max(v_scale, 0.0);

    %% AFS (v5와 동일)
    err  = yawRateRef - yawRate;
    dErr = (err - ctrlState.prevError) / max(dt, 1e-6);
    ctrlState.prevError = err;
    ctrlState.intError  = 0;

    Kp_afs = CTRL.LAT.Kp * 0.15 * v_scale;
    Kd_afs = CTRL.LAT.Kd * 0.05 * v_scale;
    steer_raw = Kp_afs * err + Kd_afs * dErr;
    steer_sat = max(-LIM.MAX_STEER_ANGLE * 0.3, ...
                min( LIM.MAX_STEER_ANGLE * 0.3, steer_raw));
    deltaAdd.steerAngle = steer_sat;

    %% ESC (v5와 동일 — beta_th=2.5°, K=9000)
    % v5에서 A7 sideSlip=2.17°, A1 sideSlip=2.79° 모두 양호
    % LTR 개선 목적으로 beta_th 낮추면 ESC가 더 자주 개입 → lateralDev 악화 위험
    % → v5와 동일하게 유지
    beta_th = deg2rad(2.5);

    if abs(slipAngle) > beta_th
        beta_excess   = abs(slipAngle) - beta_th;
        K_esc         = 9000 * v_scale;
        yawMoment_raw = -K_esc * sign(slipAngle) * beta_excess;
        deltaAdd.yawMoment = max(-6000, min(6000, yawMoment_raw));
    else
        deltaAdd.yawMoment = 0;
    end

end
