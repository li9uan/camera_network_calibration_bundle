% input requires camera poses
% draw_camera_color(camera_size, 0, 0, 0, R_pose, T_pose, 1, [0 0 1], 1);

function draw_camera_color(dX, xtranslate, ytranslate, ztranslate, Cam_R_pose, Cam_T_pose, xscale, color, line_width)

% [Cam_R_pose, Cam_T_pose] puts camera coordinate to world coordinate
% therefore input is camera poses, not camera extrinsics

% draw the cameras
P1 = [0; 0; 0; 1];
P2 = [-2*dX; -dX; 2*dX; 1];
P3 = [-2*dX; dX; 2*dX; 1];
P4 = [2*dX; dX; 2*dX; 1];
P5 = [2*dX; -dX; 2*dX; 1];
P6 = [0; 0; 2*dX; 1];
P7 = [0; -dX; 2*dX; 1];

P_transformation = [Cam_R_pose, Cam_T_pose]; % from camera coordinates to world

PP1 = P_transformation*P1/xscale+[xtranslate;ytranslate;ztranslate];
PP2 = P_transformation*P2/xscale+[xtranslate;ytranslate;ztranslate];
PP3 = P_transformation*P3/xscale+[xtranslate;ytranslate;ztranslate];
PP4 = P_transformation*P4/xscale+[xtranslate;ytranslate;ztranslate];
PP5 = P_transformation*P5/xscale+[xtranslate;ytranslate;ztranslate];
PP6 = P_transformation*P6/xscale+[xtranslate;ytranslate;ztranslate];
PP7 = P_transformation*P7/xscale+[xtranslate;ytranslate;ztranslate];

PP1 = [PP1(1) PP1(2) PP1(3)];
PP2 = [PP2(1) PP2(2) PP2(3)];
PP3 = [PP3(1) PP3(2) PP3(3)];
PP4 = [PP4(1) PP4(2) PP4(3)];
PP5 = [PP5(1) PP5(2) PP5(3)];
PP6 = [PP6(1) PP6(2) PP6(3)];
PP7 = [PP7(1) PP7(2) PP7(3)];

hold on;

line([PP1(1); PP2(1)],[PP1(2); PP2(2)],[PP1(3); PP2(3)], 'LineWidth', line_width, 'Color', color);
line([PP1(1); PP3(1)],[PP1(2); PP3(2)],[PP1(3); PP3(3)], 'LineWidth', line_width, 'Color', color);
line([PP1(1); PP4(1)],[PP1(2); PP4(2)],[PP1(3); PP4(3)], 'LineWidth', line_width, 'Color', color);
line([PP1(1); PP5(1)],[PP1(2); PP5(2)],[PP1(3); PP5(3)], 'LineWidth', line_width, 'Color', color);
line([PP2(1); PP3(1)],[PP2(2); PP3(2)],[PP2(3); PP3(3)], 'LineWidth', line_width, 'Color', color);
line([PP3(1); PP4(1)],[PP3(2); PP4(2)],[PP3(3); PP4(3)], 'LineWidth', line_width, 'Color', color);
line([PP4(1); PP5(1)],[PP4(2); PP5(2)],[PP4(3); PP5(3)], 'LineWidth', line_width, 'Color', color);
line([PP5(1); PP2(1)],[PP5(2); PP2(2)],[PP5(3); PP2(3)], 'LineWidth', line_width, 'Color', color);
line([PP6(1); PP7(1)],[PP6(2); PP7(2)],[PP6(3); PP7(3)], 'LineWidth', line_width, 'Color', color);
    
hold on;
