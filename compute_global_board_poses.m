function global_board_poses = compute_global_board_poses(global_camera_poses, ...
                                         camerasParams, ...
                                         valid_camera_boards)
                                     
%% putting all board poses into the global coordinate system
num_cams =   size(valid_camera_boards, 1);
num_boards = size(valid_camera_boards, 2);
global_board_poses = cell(num_boards,1);

board_camera_indices = zeros(num_boards);  % index of which camera is the board pose coming from
for b = 1:num_boards
    for c = 1:num_cams
        if (valid_camera_boards(c,b))
            board_camera_indices(b) = c;
            break;
        end
    end
end

for b = 1:num_boards
    cam_index = board_camera_indices(b); % corresponding_cam_index

    board_usage = valid_camera_boards(cam_index,:);
    [val index] = find(board_usage);
    [val board_index_in_calib] = find(index==b);

    RV = camerasParams{cam_index}.RotationMatrices(:,:,board_index_in_calib)';
    TV = camerasParams{cam_index}.TranslationVectors(board_index_in_calib,:)';

    global_board_poses{b} = global_camera_poses{cam_index}*[RV TV; 0 0 0 1];
end