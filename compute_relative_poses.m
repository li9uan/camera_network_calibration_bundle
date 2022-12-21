function [rel_camera_poses] = compute_relative_poses(camerasParams, ...
                                                     camera_boards_mat)
% rel_camera_poses(i,j) is the transformation so that transforms 3D points in the j-th cam
% coordinate system to the world and then to the i-th cam coordinate

num_cams = size(camera_boards_mat, 1);
num_boards = size(camera_boards_mat, 2);
rel_camera_poses = cell(num_cams,num_cams);

%% initialize output
for c = 1:num_cams
    for cc = 1:num_cams
        if (c == cc)
            rel_camera_poses{c, cc} = eye(4);
        else
            rel_camera_poses{c, cc} = zeros(4);
        end
    end
end

is_connectivity_updated = true;

while (is_connectivity_updated)
    is_connectivity_updated = false;
    
    for c = 1:num_cams
        for cc = c+1:num_cams
            if (c==cc)
                continue;
            end
            if (sum(abs(rel_camera_poses{c,cc}(:))))
                continue;
            end
            
            disp(sprintf('%d - %d', c, cc));
            
            % if elements is seen by the same board, use the board as
            % target to find the relative pose between (c, cc) and (cc, c)
            found_transitional_board = 0;
            for b = 1:num_boards
                if (camera_boards_mat(c,b) && camera_boards_mat(cc,b))
                    found_transitional_board = b;
                    break;
                end
            end
            
            if (found_transitional_board)
                disp(sprintf('use board %d', found_transitional_board));
                rel_camera_poses{c,cc} = assign_relative_pose_from_board(c, cc, b, camerasParams, camera_boards_mat);
                rel_camera_poses{cc,c} = assign_relative_pose_from_board(cc, c, b, camerasParams, camera_boards_mat);
                is_connectivity_updated = true;
                continue;
            end
            
            % for elements in rel_board_poses that has not been assigned
            % we check if there is any other cameras associating that
            found_transitional_camera = 0;
            for ccc = 1:num_cams
                if (sum(abs(rel_camera_poses{c,ccc}(:))) && ...
                    sum(abs(rel_camera_poses{cc,ccc}(:))))
                    found_transitional_camera = ccc;
                    break;
                end
            end
            if (found_transitional_camera)
                disp(sprintf('use transition cam %d', found_transitional_camera));
                % assign RTc_cc using RT_c_ccc and RT_cc_ccc
                rel_camera_poses{c,cc} = assign_relative_pose_from_cam(c, cc, ccc, rel_camera_poses);
                rel_camera_poses{cc,c} = assign_relative_pose_from_cam(cc, c, ccc, rel_camera_poses);
                is_connectivity_updated = true;
                continue;
            end
        end
        
        % find already assigned cameras
    end
end

test = 1;