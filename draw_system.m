function B = draw_system(global_camera_poses, global_board_poses, checkerboard_nr, checkerboard_nc, checkerboard_grid_square_length)

figure; load_colors;
camera_size = 0.06;
num_cams = size(global_camera_poses);
for i = 1:num_cams
    draw_camera_color(camera_size, 0, 0, 0, global_camera_poses{i}(1:3,1:3), global_camera_poses{i}(1:3,4), 1, colors{i}, 1); hold on;
end;

num_boards = numel(global_board_poses);
for i = 1:num_boards
    B{i} = draw_checkboard_grid(global_board_poses{i}(1:3,1:3), ...
        global_board_poses{i}(1:3,4), ...
        checkerboard_nr, checkerboard_nc, ...
        checkerboard_grid_square_length, colors{i}, 1);
end
axis tight; axis equal;