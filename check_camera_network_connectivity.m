% given camera to checkerboard detection binary matrix, figure out of
% calibration of network is possible. Namely no island of camera clusters
% existing, e.g. c1 c2 can both see b1 b2, c4 ~ c9 can all see b3-b20, then
% c1 and c2 do not have any connection with c4 ~ c9, even though the
% calibration configuration satisfies:
% (1) assume all boards can see at least two boards
% (2) and every board can be seen by at least two cameras.

function [is_connected, camera_degree_matrix, board_degree_matrix] = check_camera_network_connectivity(camera_boards_mat)

num_cams = size(camera_boards_mat, 1);
num_boards = size(camera_boards_mat, 2);
camera_degree_matrix = eye(num_cams);
board_degree_matrix = eye(num_boards);

cam_connectivity_mat = zeros(num_cams); % adjacency matrix diagnal to be zero
% initial assignment
for c = 1:num_cams
    for b = 1:num_boards
        if (camera_boards_mat(c,b)) % if the board b is seen in cam c
            for cc = 1:num_cams
                if (camera_boards_mat(cc,b)) % if the board b is also seen in cam cc
                    cam_connectivity_mat(c,cc) = 1;
                    cam_connectivity_mat(cc,c) = 1;
                end
            end
        end
    end
end

% propagate the connectivity matrix until it does not update
is_matrix_the_same = false;
A = double(cam_connectivity_mat);
B = A;
step = 1;
while (~is_matrix_the_same)
    step = step + 1;
    B = B * double(cam_connectivity_mat);
    
    for i = 1:num_cams
        for j = 1:num_cams
            if (camera_degree_matrix(i,j) == 0 && A(i,j))
                camera_degree_matrix(i,j) = step;
            end
        end
    end
    
    if ((B>0) == (A>0))
        is_matrix_the_same = true;
        break;
    end
    
    A = B
end
is_connected = ~(any(A(:) == 0));

% if (~is_connected)
%     disp('camera network is not fully connected with all checkerboard poses. There are islands in the network.');
%     return;
% end

board_connectivity_mat = zeros(num_boards); % adjacency matrix diagnal to be zero
% initial assignment
for b = 1:num_boards
    for c = 1:num_cams
    
        if (camera_boards_mat(c,b)) % if cam c sees board b
            for bb = 1:num_boards
                if (camera_boards_mat(c,bb)) % if cam c also sees board bb
                    board_connectivity_mat(b,bb) = 1;
                    board_connectivity_mat(bb,b) = 1;
                end
            end
        end
    end
end

% propagate the connectivity matrix until it does not update
is_matrix_the_same = false;
A = double(board_connectivity_mat);
B = A;
step = 1;
while (~is_matrix_the_same)
    step = step + 1;
    B = B * double(board_connectivity_mat);
    
    for i = 1:num_boards
        for j = 1:num_boards
            if (board_degree_matrix(i,j) == 0 && A(i,j))
                board_degree_matrix(i,j) = step;
            end
        end
    end
    
    if ((B>0) == (A>0))
        is_matrix_the_same = true;
        break;
    end
    
    A = B
end
