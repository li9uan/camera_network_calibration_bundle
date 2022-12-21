function [cam_image_extracts, valid_camera_boards] = ...
    detection_images_corners(fileList, num_cams, num_poses, ...
                             checkerboard_nr, checkerboard_nc, cam_ids)

file_id = 1;
figure;
for i = 1:num_cams
    for j = 1:num_poses
        file_name = sprintf('%s/%s', fileList(file_id).folder, fileList(file_id).name);
        
        [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(file_name, 'MinCornerMetric', 0.25);
        if (numel(imagePoints)/2 ~= checkerboard_nr * checkerboard_nc)
            imagesUsed = false;
        end
        if (imagesUsed)
            I = imread(file_name);
            imageSize = [size(I, 1), size(I, 2)];
            imshow(I);
            hold on;
            for p = 1:size(imagePoints,1)
                plot(imagePoints(p,1,1),imagePoints(p,2,1),'ro'); hold on;
            end
            plot(imagePoints(1,1,1),imagePoints(1,2,1),'bo'); 
            title(sprintf('cam %d - board %d', cam_ids(i), j));
            
            % dump detected checkerboard locations to python file
            folder_path = './calibration/calib_01_31_2019/MATLAB_checkerboard_detected_locations';
            point_file_dir = sprintf('%s/%s', folder_path, fileList(file_id).name);
            point_file_dir(end-2:end) = 'txt';
            output_points_python(point_file_dir, imagePoints, checkerboard_nr, checkerboard_nc);
            pause(0.5);
        end
        
        if (imagesUsed)
            cam_image_extracts{i,j} = imagePoints;
        else
            cam_image_extracts{i,j} = [];
        end
        valid_camera_boards(i,j) = imagesUsed;
        
        file_id = file_id + 1;
    end
end
close; %figure