%% accuracy report loop
clear; clc;

task_ids = 0:10;
hands = {'L', 'R'};
results = [];

for tid = task_ids
    for h = 1:length(hands)
        hand = hands{h};
        filename = sprintf('aligned_task%d_%s.mat', tid, hand);

        if ~isfile(filename)
            fprintf('Skipping missing file: %s\n', filename);
            continue;
        end

        try
            S = load(filename);
            if isfield(S, 'result_l')
                result = S.result_l;
            elseif isfield(S, 'result_r')
                result = S.result_r;
            else
                warning('No result_l or result_r in %s\n', filename);
                continue;
            end

            mae_cam_vec  = result.mean_err.cam;
            mae_br_vec   = result.mean_err.br;
            rmse_cam_vec = result.rmse.cam;
            rmse_br_vec  = result.rmse.br;

            mae_cam_3d  = norm(mae_cam_vec);
            mae_br_3d   = norm(mae_br_vec);
            rmse_cam_3d = norm(rmse_cam_vec);
            rmse_br_3d  = norm(rmse_br_vec);

            to_mm = @(x) x * 100;
            mae_cam_mm  = to_mm(mae_cam_vec);
            mae_br_mm   = to_mm(mae_br_vec);
            rmse_cam_mm = to_mm(rmse_cam_vec);
            rmse_br_mm  = to_mm(rmse_br_vec);
            mae_cam_3d_mm  = to_mm(mae_cam_3d);
            mae_br_3d_mm   = to_mm(mae_br_3d);
            rmse_cam_3d_mm = to_mm(rmse_cam_3d);
            rmse_br_3d_mm  = to_mm(rmse_br_3d);

            results = [results; {
                sprintf('task%d', tid), hand, ...
                mae_cam_mm(1), mae_cam_mm(2), mae_cam_mm(3), mae_cam_3d_mm, ...
                rmse_cam_mm(1), rmse_cam_mm(2), rmse_cam_mm(3), rmse_cam_3d_mm, ...
                mae_br_mm(1),  mae_br_mm(2),  mae_br_mm(3),  mae_br_3d_mm, ...
                rmse_br_mm(1), rmse_br_mm(2), rmse_br_mm(3), rmse_br_3d_mm
            }];

        catch ME
            warning('Error in %s: %s\n', filename, ME.message);
            continue;
        end
    end
end

T = cell2table(results, 'VariableNames', {
    'Task', 'Hand', ...
    'MAE_CAM_X_cm', 'MAE_CAM_Y_cm', 'MAE_CAM_Z_cm', 'MAE_CAM_3D_cm', ...
    'RMSE_CAM_X_cm', 'RMSE_CAM_Y_cm', 'RMSE_CAM_Z_cm', 'RMSE_CAM_3D_cm', ...
    'MAE_BR_X_cm',  'MAE_BR_Y_cm',  'MAE_BR_Z_cm',  'MAE_BR_3D_cm', ...
    'RMSE_BR_X_cm', 'RMSE_BR_Y_cm', 'RMSE_BR_Z_cm', 'RMSE_BR_3D_cm'
});

writetable(T, 'accuracy_results.csv');
fprintf('Accuracy results saved to accuracy_results.csv (%d rows)\n', height(T));



function aligned = dtw_align_segment(ref, target)
    aligned = zeros(size(ref));
    for dim = 1:3
        [~,ix,iy] = dtw(ref(:,dim), target(:,dim));

        [ix_unique, ia] = unique(ix, 'stable');
        iy_unique = iy(ia);

        aligned(:,dim) = interp1(ix_unique, target(iy_unique,dim), ...
                                 1:size(ref,1), 'linear', 'extrap')';
    end
end


function out = safe_interp_resample(seg, target_len)
    n = size(seg,1);
    if n < 2
        % Too short to interpolate — replicate the point
        out = repmat(seg(1,:), target_len, 1);
    else
        x_old = linspace(0, 1, n);
        x_new = linspace(0, 1, target_len);
        out = interp1(x_old, seg, x_new, 'linear');
    end
end
