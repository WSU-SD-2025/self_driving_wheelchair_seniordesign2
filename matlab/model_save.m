clc;

save_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/';

if ~exist('model_x_vL', 'var')
    error('model_x_vL not found in workspace');
end

if ~exist('model_x_vR', 'var')
    error('model_x_vR not found in workspace');
end

save(fullfile(save_dir, 'model_x_vL.mat'), 'model_x_vL');
save(fullfile(save_dir, 'model_x_vR.mat'), 'model_x_vR');

disp('X models saved successfully!');