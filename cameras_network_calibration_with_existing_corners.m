% detect checkerboard poses

clear, close all, home;
w = warning ('off','all');

is_debug = true;

checkerboard_nr = 5;
checkerboard_nc = 6;
checkerboard_grid_square_length = 0.1;
load_colors;
path = './calibration/calib_01_31_2019';
is_ROS = true; 
detection_folder_path = './calibration/calib_01_31_2019/ROS_checkerboard_detected_locations';

calibration_path = sprintf('%s/merge', path);
intrinsics_path = sprintf('%s/K.conf', path);
fileList = dir(sprintf('%s/*.jpg', calibration_path));
imageSize = get_image_size(fileList);

%% find camera ids
cam_ids = [];
for i = 1:numel(fileList)
    file_name_str = fileList(i).name;
    k1 = strfind(file_name_str,'-');
    cam_id = str2double(file_name_str(3:k1-1));
    is_repeated_cam = false;
    for c = 1:numel(cam_ids)
        if (cam_ids(c) == cam_id)
            is_repeated_cam = true;
        end
    end
    if (is_repeated_cam == false)
        cam_ids = [cam_ids cam_id];
    end
end
num_cams = numel(cam_ids);

% list checkerboard pose 
num_boards = numel(fileList)/num_cams;
for i = 1:num_boards
    str = fileList(i).name;
    k1 = strfind(str,'-');
    k2 = strfind(str,'.');
    pose_ids{i} = str(k1+1:k2-1);
end

clear global x y nc np xgrid ygrid kkRadial;
global x y nc np xgrid ygrid kkRadial; 
% where x y are image corner matrices
% nc is the number of cameras
% np is the number of planes
np = num_boards;
nc = num_cams;

% get standard checkerboard grid coordinates
% since MATLAB corner detection code is along the short side frist
[xgrid, ygrid] = meshgrid(0:checkerboard_nc-1,0:checkerboard_nr-1);
xgrid = xgrid * checkerboard_grid_square_length;
ygrid = ygrid * checkerboard_grid_square_length;
xgrid = reshape(xgrid, [1, numel(xgrid)]);
ygrid = reshape(ygrid, [1, numel(ygrid)]);
                                       
%% load intrinsics
[Ks] = load_intrinsics(intrinsics_path, num_cams);
Ks = reorder_intrinsics(Ks, cam_ids); % so that it matches the file order fileList (on disk)


%% detect all image corners
%[cam_image_extracts, valid_camera_boards] = detection_images_corners(fileList, num_cams, num_boards, ...
%    checkerboard_nr, checkerboard_nc, cam_ids);

[cam_image_extracts, valid_camera_boards] = load_image_corners(detection_folder_path, fileList, num_cams, num_boards, ...
    checkerboard_nr, checkerboard_nc, cam_ids);

% ROS results need to shift 1 pixel
if (is_ROS)
    num_cams = size(cam_image_extracts, 1);
    num_poses = size(cam_image_extracts, 2);
    for i = 1:num_cams
        for j = 1:num_poses
            if (size(cam_image_extracts{i,j}, 1))
                cam_image_extracts{i,j} = cam_image_extracts{i,j} + 1;
            end
        end
    end
end

%% compute stats about calibration coverage
num_boards_per_camera_raw = sum(valid_camera_boards,2);
for i = 1:num_cams
    fprintf('cam %d has %d boards that have valid detections.\n', cam_ids(i), num_boards_per_camera_raw(i));
end
% num_boards_per_camera = zeros(num_cams,1);
% for i = 1:num_cams
%     num_boards_per_camera(i) = num_boards_per_camera_raw(find(cam_ids==i));
%     disp(sprintf('cam %d has %d boards detected', i, num_boards_per_camera(i)));
% end

%% associate all boards and cameras into the same coordinate system
%% assume all boards can see at least two boards
%% and every board can be seen by at least two cameras
[is_network_connected, camera_degree_matrix, board_degree_matrix] = check_camera_network_connectivity(valid_camera_boards)
if (is_network_connected == false)
    disp('Camera network has calibration islands. Need more calibration images.');
    return;
end

[val starting_cam_id] = min(sum(camera_degree_matrix)); % starting_cam_id is the camera that we refer to as reference
[val starting_board_id] = min(sum(board_degree_matrix)); % starting_board_id is the board that we refer to as reference

%% generate checkerboard world points
squareSize = 0.1; % in meters
worldPoints = generateCheckerboardPoints([checkerboard_nr+1, checkerboard_nc+1], squareSize);

%% initial camera calibration
[camerasParams, RV, TV] = calibrate_cameras(num_cams, num_boards, imageSize, ...
                                           cam_image_extracts, worldPoints);

%% compute the relative camera poses and board poses struct matrix, 
% for convenience every element in the matrix is a 4x4 matrix ([R t; 0 0 0 1]
% the implementation is similar to connectivity analysis in function check_camera_network_connectivity

% rel_camera_poses(i,j) is the transformation so that transforms 3D points in the j-th cam
% coordinate system to the world and then to the i-th cam coordinate
rel_camera_poses = compute_relative_poses(camerasParams, ...
                                          valid_camera_boards);
           
%% compute global pose
reference_cam_id = starting_cam_id;
global_camera_poses = compute_global_cam_poses(rel_camera_poses, reference_cam_id);

%% compute checkerboard pose reference camera id
global_board_poses = compute_global_board_poses(global_camera_poses, ...
                                                camerasParams, ...
                                                valid_camera_boards);

%% draw the network + calibration boards
%% draw global poses, and return all 3D points of different checkerboard poses
B_global = draw_system(global_camera_poses, global_board_poses, checkerboard_nr, checkerboard_nc, checkerboard_grid_square_length);

%% draw reprojection with the initial global system
% seems the corner detection is row-first
for i = 1:num_cams
    Rs{i} = global_camera_poses{i}(1:3,1:3);
    Ts{i} = global_camera_poses{i}(1:3,4);
end
for i = 1:num_boards
    Rv{i} = global_board_poses{i}(1:3,1:3);
    Tv{i} = global_board_poses{i}(1:3,4);
end

idx = 1;
figure;
for cam_indx = 1:num_cams
    for board_idx = 1:num_boards
        
        dir = sprintf('%s/%s', fileList(idx).folder, fileList(idx).name);
        idx = idx + 1;
        if (~valid_camera_boards(cam_indx, board_idx))
            continue;
        end
        
        fprintf('cam %d - board %d\n', cam_ids(cam_indx), board_idx);
        evaluate_reprojection(dir, ...
                              Ks{cam_indx}, Rs{cam_indx}, Ts{cam_indx}, ...
                              B_global{board_idx}, 1, 1);
        title(sprintf('cam%d - board%d', cam_ids(cam_indx), board_idx));
        pause(0.5); clf;
    end
end

%% bundle adjustment
% start bundle adjustment
is_adjustK = false;
[Rc_output_0, Tc_output_0, RV_output_0, TV_output_0] = bundle_adjust(Ks, Rs, Ts, Rv, Tv, valid_camera_boards, ...
              checkerboard_nr, checkerboard_nc, cam_image_extracts, is_adjustK);
          
%% normalize the camera coordinate system so that the first camera is zeros(3) and [0 0 0]';
% Rc Tc are the camera poses, not camera extrinsics
% Rv Tv are the board poses (from board coordinates to world coordinate)
% therefore, to make the first camera the identity rotation and at the
% origin, just left multiply every transformation with the inverse of the
% first camera's poses
% Rc_output = Rc_output_0;
% Tc_output = Tc_output_0;
% RV_output = RV_output_0;
% TV_output = TV_output_0;
[Rc_output, Tc_output, RV_output, TV_output] = normalize_system(Rc_output_0, Tc_output_0, RV_output_0, TV_output_0);

%% visualize final result          
figure; hold on;
camera_size = 0.06;
line_thickness = 1;
for i = 1:nc
    draw_camera_color(camera_size, 0, 0, 0, Rc_output{i}, Tc_output{i}, 1, colors{i}, line_thickness); hold on;
    text(Tc_output{i}(1), Tc_output{i}(2), Tc_output{i}(3),...
        ['cam ' num2str(cam_ids(i))], 'HorizontalAlignment','left','FontSize',16);
end;
axis equal;
for j = 1:np
    B{j} = draw_checkboard_grid(RV_output{j}, TV_output{j}, ...
        checkerboard_nr, checkerboard_nc, ...
        checkerboard_grid_square_length, colors{j}, line_thickness);
    text(TV_output{j}(1), TV_output{j}(2), TV_output{j}(3),...
        ['board ' num2str(j)], 'HorizontalAlignment','left','FontSize',16);
end
% draw before BA result 
is_draw_before_BA_result = false;
if (is_draw_before_BA_result)
    hold on;
    for i = 1:nc
        draw_camera_color(camera_size, 0, 0, 0, Rs{i}, Ts{i}, 1, colors{i}, 1); hold on;
    end;
    axis equal;
    for j = 1:np
        B_{j} = draw_checkboard_grid(Rv{j}, Tv{j}, ...
            checkerboard_nr, checkerboard_nc, ...
            checkerboard_grid_square_length, colors{j}, 1);
    end
end

axis equal; axis tight; 

% %%
% % draw reprojection
% % seems the corner detection is row-first
% idx = 1;
% for cam_indx = 1:num_cams
%     for board_idx = 1:num_boards
%         
%         dir = sprintf('%s/%s', fileList(idx).folder, fileList(idx).name);
%         idx = idx + 1;
%         if (~valid_camera_boards(cam_indx, board_idx))
%             continue;
%         end
%         
%         f= figure(2);
%         f.WindowState = 'maximized';
%         evaluate_reprojection(dir, ...
%                               Ks{cam_indx}, Rc_output{cam_indx}, Tc_output{cam_indx}, ...
%                               B{board_idx}, 1, 1);
%         pause(1); close;
%     end
% end

%% save calibration results
t = datetime('now','Format','yyyy_MM_dd_HH_mm_ss');
file_path = sprintf('%s/ROS_precomputed_corner_result_%s.mat', path, t);
save_calibration(file_path, Ks, Rc_output, Tc_output, cam_ids);