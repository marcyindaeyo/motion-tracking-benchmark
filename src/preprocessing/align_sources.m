function [t_ref, pos_vive_out, pos_cam_shifted, pos_br_shifted, lags_out] = ...
    align_sources(time_vive, pos_vive, ...
                  time_cam,  pos_cam, ...
                  time_br,   pos_br, ...
                  options)
% Unified alignment function for global or local use
% options.mode: 'global' or 'local'
% options.plot: true / false
% options.t_start, options.t_end: required if mode == 'local'
% options.buffer: optional (used in local mode)

    arguments
        time_vive
        pos_vive
        time_cam
        pos_cam
        time_br
        pos_br
        options.mode {mustBeMember(options.mode, {'global', 'local'})}
        options.plot logical = false
        options.t_start double = NaN
        options.t_end double = NaN
        options.buffer double = 0
    end

    %% ===== Step 1: Select reference time range =====
    if strcmp(options.mode, 'local')
        % Expand task window
        t_start = options.t_start - options.buffer;
        t_end   = options.t_end   + options.buffer;

        idx_vive = time_vive >= t_start & time_vive <= t_end;
        idx_cam  = time_cam  >= t_start & time_cam  <= t_end;
        idx_br   = time_br   >= t_start & time_br   <= t_end;

        t_ref = time_br(idx_br);  % Use BR time as reference
    else
        % Use full time range
        t_ref = time_br;
        idx_vive = true(size(time_vive));
        idx_cam  = true(size(time_cam));
        idx_br   = true(size(time_br));
    end

    %% ===== Step 2: Interpolation =====
    warnState = warning('off', 'MATLAB:interp1:NaNstrip');
    pos_vive = interp1(time_vive(idx_vive), pos_vive(idx_vive,:), t_ref, 'pchip', NaN);
    pos_cam  = interp1(time_cam(idx_cam),   pos_cam(idx_cam,:),   t_ref, 'pchip', NaN);
    pos_br   = interp1(time_br(idx_br),     pos_br(idx_br,:),     t_ref, 'pchip', NaN);
    warning(warnState);

    %% ===== Step 3: Fill missing + smooth =====
    pos_vive = fillmissing(pos_vive, 'movmedian', 11);
    pos_vive = fillmissing(pos_vive, 'nearest');
    pos_vive = movmedian(pos_vive, 5);

    pos_cam  = fillmissing(pos_cam, 'movmedian', 11);
    pos_cam  = fillmissing(pos_cam, 'nearest');
    pos_cam  = movmedian(pos_cam, 5);

    pos_br   = fillmissing(pos_br, 'movmedian', 11);
    pos_br   = fillmissing(pos_br, 'nearest');
    pos_br   = movmedian(pos_br, 5);

    %% ===== Step 4: Extract vertical signals for cross-correlation =====
    sig_vive = pos_vive(:,3);       % Z
    sig_cam  = -pos_cam(:,2);       % -Y
    sig_br   = pos_br(:,3);         % Z
    if strcmp(options.mode, 'global')
        lag_max = 3000;
    else
        lag_max = 300;
    end

    [c_cam, lags_cam] = xcorr(sig_vive, sig_cam, lag_max); 
    [c_br, lags_br]  = xcorr(sig_vive, sig_br, lag_max);
    figure;
    plot(lags_cam', c_cam); title('Cross-corr VIVE vs CAM');
    figure;
    plot(lags_br', c_br);  title('Cross-corr VIVE vs BR');

    lag_cam = lags_cam(c_cam == max(c_cam));
    lag_br  = lags_br(c_br == max(c_br));

    pos_cam_circ = circshift(pos_cam, lag_cam);
    pos_br_circ  = circshift(pos_br,  lag_br);

    dt_ref = mean(diff(t_ref));
    time_cam_shifted = t_ref + lag_cam * dt_ref;
    time_br_shifted  = t_ref + lag_br  * dt_ref;

    pos_cam_interp = interp1(time_cam_shifted, pos_cam, t_ref, 'pchip', NaN);
    pos_br_interp  = interp1(time_br_shifted,  pos_br,  t_ref, 'pchip', NaN);

    %% ===== Optional debug plots =====
    if options.plot
        figure;
        hold on;
        plot(t_ref, sig_vive, 'k', 'DisplayName', 'VIVE Z');
        plot(t_ref, -pos_cam_circ(:,2), '-', 'Color', [1 0 0], 'DisplayName', 'CAM -Y circ');
        plot(t_ref, pos_br_circ(:,3), '-', 'Color', [0 0 1], 'DisplayName', 'BR Z circ');
        plot(t_ref, -pos_cam_interp(:,2), '--', 'Color', [1 0.5 0.5], 'DisplayName', 'CAM -Y interp');
        plot(t_ref, pos_br_interp(:,3), '--', 'Color', [0.5 0.5 1], 'DisplayName', 'BR Z interp');
        title('Alignment Comparison: circshift vs interp'); grid on; legend;
    end

    %% ===== Output =====
    pos_vive_out = pos_vive;
    pos_cam_shifted = pos_cam_interp;  % you can switch this to _circ if you prefer
    pos_br_shifted  = pos_br_interp;

    lags_out.cam = lag_cam;
    lags_out.br  = lag_br;
end
