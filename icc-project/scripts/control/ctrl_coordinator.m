function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation  v5
%
%   run_icc_scenario: brake_total = brk_scenario + brakeESC
%   ABS: brakeESC = (absScale-1)*T_scenario (음수 = 제동 감소)
%   ESC: 차동 제동 (한쪽 증가)
%   v3과 동일한 구조 유지

    rw  = VEH.rw;
    t_f = VEH.track_f;
    t_r = VEH.track_r;

    %% 1. AFS steer
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                             min( LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% 2. ABS 조정
    T_scen = [1500; 1500; 800; 800];
    brakeTorque = zeros(4,1);
    if isfield(lonCmd, 'absScale')
        sc = lonCmd.absScale(:);
        if numel(sc) < 4; sc = ones(4,1); end
        for w = 1:4
            if sc(w) < 0.999
                adj = (sc(w) - 1.0) * T_scen(w);
                adj = max(-T_scen(w), adj);
                brakeTorque(w) = adj;
            end
        end
    end

    %% 3. ESC 차동 제동
    Mz = latCmd.yawMoment;
    if abs(Mz) > 5.0
        esc_f = 0.55;
        esc_r = 0.45;
        dT_f = abs(Mz) * esc_f / (t_f/2) * rw;
        dT_r = abs(Mz) * esc_r / (t_r/2) * rw;
        if Mz > 0
            brakeTorque(1) = brakeTorque(1) + dT_f;
            brakeTorque(3) = brakeTorque(3) + dT_r;
        else
            brakeTorque(2) = brakeTorque(2) + dT_f;
            brakeTorque(4) = brakeTorque(4) + dT_r;
        end
    end

    %% 4. Saturation
    for w = 1:4
        brakeTorque(w) = max(-T_scen(w), min(LIM.MAX_BRAKE_TRQ, brakeTorque(w)));
    end
    actuatorCmd.brakeTorque = brakeTorque;

    %% 5. 감쇠
    if numel(verCmd) == 4
        actuatorCmd.dampingCoeff = verCmd(:);
    else
        actuatorCmd.dampingCoeff = 1500 * ones(4,1);
    end

end
