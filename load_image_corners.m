function [cam_image_extracts, valid_camera_boards] = load_image_corners(detection_file_path, fileList, num_cams, num_poses, ...
    checkerboard_nr, checkerboard_nc, cam_ids)

is_debug = false;
valid_camera_boards = zeros(num_cams, num_poses);
file_id = 1;
for i = 1:num_cams
    for j = 1:num_poses
        file_name = sprintf('%s/%s', detection_file_path, fileList(file_id).name);
        % change the file_name subfix from png to txt
        file_name(end-2:end) = 'txt';
        if (exist(file_name, 'file'))
            valid_camera_boards(i,j) = 1;
            % load the image
            corners = load_detection_python(file_name);
            X = corners(1:2:end) - 1;
            Y = corners(2:2:end) - 1;
            
            X = reshape(reshape(X, [checkerboard_nc,checkerboard_nr])', checkerboard_nc*checkerboard_nr, 1);
            Y = reshape(reshape(Y, [checkerboard_nc,checkerboard_nr])', checkerboard_nc*checkerboard_nr, 1);
            
            cam_image_extracts{i,j} = [X Y];
            
            if (is_debug)
                % project on image
                image_file_name = sprintf('%s/%s', fileList(file_id).folder, fileList(file_id).name);
                I = imread(image_file_name);
                
                imageSize = [size(I, 1), size(I, 2)];
                imshow(I);
                hold on;
                imagePoints = cam_image_extracts{i,j};
                for p = 1:size(imagePoints,1)
                    plot(imagePoints(p,1,1),imagePoints(p,2,1),'ro'); hold on;
                end
                plot(imagePoints(1,1,1),imagePoints(1,2,1),'bo'); 
                title(sprintf('cam %d - board %d', cam_ids(i), j));
                pause; %(0.5); close;
            end
            
        else
            valid_camera_boards(i,j) = 0;
            cam_image_extracts{i,j} = [];
        end
        
        file_id = file_id + 1;
    end
end
