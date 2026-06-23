function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Hybrid Skyhook + Groundhook 알고리즘 적용.
%   Body-bounce (1~2 Hz)와 wheel-hop (10~15 Hz) 모드를 분리 처리.
%
%   Inputs:
%       suspState - struct
%           .zs_dot(4)  - sprung mass 속도 [m/s]
%           .zu_dot(4)  - unsprung mass 속도 [m/s]
%           .zs(4)      - sprung 변위 [m]
%           .zu(4)      - unsprung 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin, .cMax, .skyGain
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]

    %% 상태 초기화
    if ~isfield(ctrlState, 'zs_dot_filt')
        ctrlState.zs_dot_filt = zeros(4,1);
        ctrlState.zu_dot_filt = zeros(4,1);
    end

    %% suspState 유효성 확인
    if ~isfield(suspState, 'zs_dot') || isempty(suspState.zs_dot)
        dampingCmd = CTRL.VER.cMin * ones(4,1);
        return;
    end

    zs_dot = suspState.zs_dot(:);
    zu_dot = suspState.zu_dot(:);

    % 4개 요소 보장
    if numel(zs_dot) < 4; zs_dot(end+1:4) = 0; end
    if numel(zu_dot) < 4; zu_dot(end+1:4) = 0; end

    %% 저역통과 필터 (Body-bounce 대역, fc ≈ 3 Hz)
    % 1차 IIR: y = α*y_prev + (1-α)*x
    fc_body = 3.0;   % [Hz]
    alpha_b = exp(-2*pi*fc_body*dt);
    zs_filt = alpha_b * ctrlState.zs_dot_filt + (1 - alpha_b) * zs_dot;

    % 고역통과 = 전체 - 저역 (Wheel-hop 대역, fc ≈ 8 Hz)
    fc_wheel = 8.0;
    alpha_w  = exp(-2*pi*fc_wheel*dt);
    zu_filt  = alpha_w * ctrlState.zu_dot_filt + (1 - alpha_w) * zu_dot;

    ctrlState.zs_dot_filt = zs_filt;
    ctrlState.zu_dot_filt = zu_filt;

    %% Hybrid Skyhook + Groundhook
    % α: skyhook 비율, (1-α): groundhook 비율
    alpha_sh = 0.7;   % 승차감 우선 (comfort-oriented)
    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    cSky = CTRL.VER.skyGain;

    dampingCmd = zeros(4,1);

    for w = 1:4
        zs_w = zs_filt(w);    % sprung velocity (filtered)
        zu_w = zu_filt(w);    % unsprung velocity (filtered)
        rel_w = zs_w - zu_w;  % relative velocity

        % --- Skyhook component ---
        % 조건: sprung velocity 와 relative velocity 부호 동일할 때만 c_sky 적용
        if (zs_w * rel_w) > 0
            % c_sky: F_sky = cSky * zs_dot → c_equiv = cSky * |zs_dot| / |rel_dot|
            if abs(rel_w) > 1e-4
                c_sky_w = cSky * abs(zs_w) / abs(rel_w);
            else
                c_sky_w = cMax;
            end
            c_sky_w = min(c_sky_w, cMax);
        else
            c_sky_w = cMin;
        end

        % --- Groundhook component (wheel-hop 억제) ---
        if (zu_w * rel_w) < 0
            if abs(rel_w) > 1e-4
                c_gnd_w = cSky * abs(zu_w) / abs(rel_w);
            else
                c_gnd_w = cMax;
            end
            c_gnd_w = min(c_gnd_w, cMax);
        else
            c_gnd_w = cMin;
        end

        % Hybrid
        c_hybrid = alpha_sh * c_sky_w + (1 - alpha_sh) * c_gnd_w;

        % 범위 제한
        dampingCmd(w) = max(cMin, min(cMax, c_hybrid));
    end

end
