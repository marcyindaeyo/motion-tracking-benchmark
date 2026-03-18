clc; clear;

%% Data Load

% Load BR data
load('data/br_left_pos.mat');
load('data/br_right_pos.mat');
data.left.time_br = br_left_pos(:, 1);
data.left.pos_br = br_left_pos(:, 2:4);
data.right.time_br = br_right_pos(:, 1);
data.right.pos_br = br_right_pos(:, 2:4);

% Load Camera data
load('data/camera.mat'); 
camera = remove_duplicate(camera);

time = double(camera.Time);

left_shoulder  = [camera.LEFT_SHOULDER_X,  camera.LEFT_SHOULDER_Y,  camera.LEFT_SHOULDER_Z];
right_shoulder = [camera.RIGHT_SHOULDER_X, camera.RIGHT_SHOULDER_Y, camera.RIGHT_SHOULDER_Z];
center = (left_shoulder + right_shoulder) / 2;

left_wrist  = [camera.LEFT_WRIST_X,  camera.LEFT_WRIST_Y,  camera.LEFT_WRIST_Z];
right_wrist = [camera.RIGHT_WRIST_X, camera.RIGHT_WRIST_Y, camera.RIGHT_WRIST_Z];

data.left.time_cam = time;
data.right.time_cam = time;
data.left.pos_cam = left_wrist  - center;
data.right.pos_cam = right_wrist - center;
data.left.pos_cam = movmean(data.left.pos_cam, 5);
data.right.pos_cam = movmean(data.right.pos_cam, 5);

% Load VIVE data
load('data/vive_chest.mat');    
load('data/vive_left.mat');     
load('data/vive_right.mat');  

vive_c = remove_duplicate(vive_c);
vive_l = remove_duplicate(vive_l);
vive_r= remove_duplicate(vive_r);

time_c = double(vive_c.Time);
pos_c = [vive_c.PosX, vive_c.PosZ, vive_c.PosY];
c_interp = @(t) interp1(time_c, pos_c, t, 'pchip', 'extrap');

data.left.time_vive = double(vive_l.Time);
pos_l = [vive_l.PosX, vive_l.PosZ, vive_l.PosY];
data.left.pos_vive = pos_l - c_interp(data.left.time_vive);

data.right.time_vive = double(vive_r.Time);
pos_r = [vive_r.PosX, vive_r.PosZ, vive_r.PosY];
data.right.pos_vive = pos_r - c_interp(data.right.time_vive);

%% Global Alignment 

[data_shifted.left.time, data_shifted.left.pos_vive, ...
 data_shifted.left.pos_cam, data_shifted.left.pos_br, ~] = ...
    align_sources(data.left.time_vive, data.left.pos_vive, ...
                  data.left.time_cam,  data.left.pos_cam, ...
                  data.left.time_br,   data.left.pos_br, ...
                  'mode', 'global',...
                  'plot', true);

[data_shifted.right.time, data_shifted.right.pos_vive, ...
 data_shifted.right.pos_cam, data_shifted.right.pos_br, ~] = ...
    align_sources(data.right.time_vive, data.right.pos_vive, ...
                  data.right.time_cam,  data.right.pos_cam, ...
                  data.right.time_br,   data.right.pos_br, ...
                  'mode', 'global',...
                  'plot', true);

%% Calibration & Kabsch
load('data/task.mat');

if isfile('kabsch_params.mat')
    load('kabsch_params.mat'); 
else
    calib_row = task_table(task_table.task == 0, :);
    if isempty(calib_row)
        error('could not find calib row, task id 0');
    end

    [~, kabsch_params_l] = process_task_segment(calib_row, ...
        data_shifted.left, true);

    [~, kabsch_params_r] = process_task_segment(calib_row, ...
        data_shifted.right, true);

    kabsch_params.l = kabsch_params_l;
    kabsch_params.r = kabsch_params_r;
    save('kabsch_params.mat', 'kabsch_params');

end

%% Task Analysis
close all
task_list = [6,7,8,9,10];  % change this list as needed

for task_id = task_list
    task_row = task_table(task_table.task == task_id, :);
    if isempty(task_row)
        warning("Task %d not found", task_id);
        continue;
    end

    result_l = process_task_segment(task_row, data_shifted.left, false, kabsch_params.l);
    save(sprintf("aligned_task%d_L.mat", task_id), "result_l");
    visualize_result(result_l, sprintf("Task%d_Left", task_id));
    fprintf("Task %d errors left:\n", task_id);
    fprintf("  CAM: %.4f  %.4f  %.4f\n", result_l.mean_err.cam);
    fprintf("  BR : %.4f  %.4f  %.4f\n", result_l.mean_err.br);

    result_r = process_task_segment(task_row, data_shifted.right, false, kabsch_params.r);
    save(sprintf("aligned_task%d_R.mat", task_id), "result_r");
    visualize_result(result_r, sprintf("Task%d_Right", task_id));
    fprintf("Task %d errors right:\n", task_id);
    fprintf("  CAM: %.4f  %.4f  %.4f\n", result_r.mean_err.cam);
    fprintf("  BR : %.4f  %.4f  %.4f\n", result_r.mean_err.br);
end



%% remove duplicate
function s_out = remove_duplicate(s_in)
    time = double(s_in.Time); 
    repeated = [false; diff(time) == 0];  
    keep = ~repeated;

    s_out = struct();
    s_out.Time = time(keep); 

    fields = fieldnames(s_in);
    for i = 1:numel(fields)
        field = fields{i};
        if strcmp(field, 'Time')
            continue
        end
        val = s_in.(field);

        if size(val, 1) == length(time)
            s_out.(field) = val(keep, :);
        else
            s_out.(field) = val;
        end
    end
end


%% visulization
function visualize_result(result, task_name)
    task_name = char(task_name); 

    labels = {'X', 'Y', 'Z'};
    figure('Name', ['Aligned Result - ' task_name]);
    for i = 1:3
        subplot(3,1,i)
        plot(result.time, result.vive(:,i), 'r-', 'LineWidth', 1.5); hold on;
        plot(result.time, result.cam(:,i), 'b-', 'LineWidth', 1.5);
        plot(result.time, result.br(:,i), 'g-', 'LineWidth', 1.5);
        ylabel(labels{i}); grid on;
        if i == 1
            title(['Aligned Position Comparison - ' task_name]);
            legend('VIVE', 'CAM', 'BodyRig');
        end
    end
    xlabel('Time (s)');
end

