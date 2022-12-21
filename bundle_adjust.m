% bundle adjustment
function [Rc_output, Tc_output, RV_output, TV_output] = bundle_adjust(Kc, Rc, Tc, RV, TV, cam_has_pose_detection, nx, ny, reprojection_2Ds, is_adjustK)

global x y nc np xgrid ygrid kkRadial; 

K = [];
R = [];
T = [];
kkRadial = [];
Rcc = [];
Tcc = [];

for i = 1:nc
    K = [K; Kc{i}];
    R = [R; Rc{i}'];         % Rc is the pose
    T = [T; -Rc{i}'*Tc{i}];  % Tc is the pose 
    kkRadial = [kkRadial; [0 0]];
end

frameSingleOut = find(sum(cam_has_pose_detection(:,1:np))<=-1);
disp('checkerboard poses that only one view or no view can see are not used in the bundle:');
disp(frameSingleOut);

% every 3D point (nx*ny) on every board (np) is projected to every camera (nc)
x = zeros(nc, np*nx*ny); 
y = zeros(nc, np*nx*ny);

index = 1;
for iter_board = 1:np
    skipping_camera_flag = 0;
    for i=1:size(frameSingleOut,2)
         if(iter_board==frameSingleOut(i))
             skipping_camera_flag = 1;
             break;
         end;
    end;
    if(skipping_camera_flag==1)
        continue;
    end
    
    % if more than one camera can see this pose
    % construct Rp and Tp matrices
    Rcc = [Rcc;RV{iter_board}];
    Tcc = [Tcc;TV{iter_board}];

    % construct image corner matrices
    for iter_cam = 1:nc
        if (numel(reprojection_2Ds{iter_cam, iter_board}) == 0)
            % if the board is not seen from the camera
            continue;
        end
        x(iter_cam,(index-1)*nx*ny+1:index*nx*ny) = reprojection_2Ds{iter_cam, iter_board}(:,1)';
        y(iter_cam,(index-1)*nx*ny+1:index*nx*ny) = reprojection_2Ds{iter_cam, iter_board}(:,2)';
    end;
    index=index+1;
end;

%bundle adjustment
if(is_adjustK)
    disp('adjust Kc Rc Tc Rp Tp but set radial distortions as fixed.');
    [Kc,Rc,tc,Rp,tp] = bundleRadial(K,R,T,Rcc,Tcc);
else
    disp('fix Kc and radial distortions, only adjust Rc Tc Rp Tp.');
    clear global fixK;
    global fixK;
    fixK = K;
    [Kc,Rc,tc,Rp,tp] = bundleRadialFixK(R,T,Rcc,Tcc);
end;

% output return
for i = 1:nc
    Rc_output{i} = Rc(i*3+(-2:0), :)'; 
    Tc_output{i} = -Rc_output{i} * tc(i*3+(-2:0), :); 
end
for i = 1:np
    RV_output{i} = Rp(i*3+(-2:0), :); 
    TV_output{i} = tp(i*3+(-2:0), :); 
end

% display calibration result
% load input_enforce_center_and_correct_result
% figure;
% hold on;
% axis on;
% displayCalib(K,Rc,tc,Rp,tp,'r');
% displayCalib(K,R,T,Rp,tp,'b');
% 
% %output bundle result
% path = './calibration/calibration_images/BAoutput';
% for i=1:nc
%     kkk = K(i*3-2:i*3, 1:3);
%     rrr = Rc(i*3-2:i*3, 1:3);
%     ttt = tc(i*3-2:i*3);
%     PPP{i}=kkk*[rrr ttt];
% 
%     if(is_adjustK)
%         dir = sprintf('%s/bundleResult%d.txt', path, i);
%     else
%         dir = sprintf('%s/bundleResultFixK%d.txt', path, i);
%     end;
%     
%     fp = fopen(dir, 'w');
%     
%     fprintf(fp, '%f %f %f %f\n', PPP{i}(1,1:4));
%     fprintf(fp, '%f %f %f %f\n', PPP{i}(2,1:4));
%     fprintf(fp, '%f %f %f %f', PPP{i}(3,1:4));
%     fclose(fp);
% end;