function [result, kabsch_param_out] = process_task_segment(task_row, data, is_calib, kabsch_param_in)

    if nargin < 3
        is_calib = false;
    end
    if nargin < 4
        kabsch_param_in = struct();
    end

    buffer = 3;

    % Local alignment within buffered window
    [t_ref, vive_all, cam_all, br_all, ~] = align_sources( ...
        data.time, data.pos_vive, ...
        data.time,  data.pos_cam, ...
        data.time,   data.pos_br, ...
        'mode', 'local', ...
        't_start', task_row.start_time, ...
        't_end',   task_row.end_time, ...
        'buffer', buffer, ...
        'plot', true);

    % Trim to original task time
    idx_task = t_ref >= task_row.start_time & t_ref <= task_row.end_time;
    t_trim   = t_ref(idx_task) - task_row.start_time;
    vive     = vive_all(idx_task, :);
    cam      = cam_all(idx_task, :);
    br       = br_all(idx_task, :);

    if is_calib
        [Uc, rc, cc, ~] = Kabsch(cam', vive');
        [Ub, rb, cb, ~] = Kabsch(br', vive');
        kabsch_param_out.cam = struct('U', Uc, 'r', rc, 'c', cc);
        kabsch_param_out.br  = struct('U', Ub, 'r', rb, 'c', cb);
        result = [];
        return;
    end

    cam_aligned = (kabsch_param_in.cam.c * kabsch_param_in.cam.U * cam' + kabsch_param_in.cam.r)';
    br_aligned  = (kabsch_param_in.br.c  * kabsch_param_in.br.U  * br'  + kabsch_param_in.br.r)';

    result.time = t_trim;
    result.vive = vive;
    result.cam  = cam_aligned;
    result.br   = br_aligned;
    result.err.cam = abs(cam_aligned - vive);
    result.err.br  = abs(br_aligned - vive);
    result.mean_err.cam = mean(result.err.cam);
    result.mean_err.br  = mean(result.err.br);
    result.rmse.cam = sqrt(mean((cam_aligned - vive).^2));
    result.rmse.br  = sqrt(mean((br_aligned - vive).^2));

    kabsch_param_out = [];  % not needed in non-calib mode
end
