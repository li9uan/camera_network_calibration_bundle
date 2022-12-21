function reprojected_points = evaluate_reprojection(path, K, R, T, B, is_vis_reprojection, sub_id)
reprojected_points = zeros(size(B, 1)*size(B, 2), 2);
I = imread(path);
ProjectionM = K*[R' -R'*T]; % get the extrinsics from camera pose

if (is_vis_reprojection)
    imshow(I); hold on;
end
index = 1;
for i = 1:size(B, 1) 
    for j = 1:size(B, 2)
        P = zeros(4,1);
        P(1) = B(i,j,1); P(2) = B(i,j,2); P(3) = B(i,j,3); P(4) = 1;
        p = ProjectionM * P;
        p(1) = p(1)/p(3); p(2) = p(2)/p(3);
        
        reprojected_points(index,:) = [p(1) p(2)]';
        if (is_vis_reprojection)
            hold on; 

            if (i == 1 && j == 1)
                plot(p(1), p(2), 'ro');
            else
                plot(p(1), p(2), 'b+');
            end
            hold on;
        end
        index = index + 1;
    end
end

