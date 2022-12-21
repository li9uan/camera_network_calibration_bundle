function global_camera_poses = compute_global_cam_poses(rel_camera_poses, starting_cam_id)

% rel_camera_poses(i,j) is the transformation so that transforms 3D points in the j-th cam
% coordinate system to the world and then to the i-th cam coordinate

num_cams = size(rel_camera_poses, 1);
global_camera_poses = cell(num_cams,1);

%% initialize output
for c = 1:num_cams
	global_camera_poses{c} = eye(4);
end

for i = 1:num_cams
    if (i == starting_cam_id)
        continue;
    end
    global_camera_poses{i} = rel_camera_poses{starting_cam_id, i}; % this stores the camera pose in i-th to starting_cam_id
end