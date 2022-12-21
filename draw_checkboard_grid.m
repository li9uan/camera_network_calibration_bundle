function board = draw_checkboard_grid(RV, TV, nr, nc, d, color, line_width)
% RV and TV transforms checkerboard points from board coordinates to the
% world coordinates

[C, R] = meshgrid(0:nc-1,0:nr-1);
C = C*d;
R = R*d;

% apply rotation and translation to the grid
Ps = zeros(nr, nc, 3);
% assign camera poses
for i = 1:size(C,1)
    for j = 1:size(C,2)
        x = C(i,j); y = R(i,j);
        p = [x y 0]';
        P = RV * p + TV;
        Ps(i,j,1) = P(1);
        Ps(i,j,2) = P(2);
        Ps(i,j,3) = P(3);
        if (i==1 && j==1)
            plot3(P(1), P(2), P(3), 'o', 'Color', color); hold on;
        end
    end
end

% draw grid
for i = 1:size(Ps,1) 
    for j = 1:size(Ps,2) 
        if (i < size(Ps,1))
            line([Ps(i,j,1); Ps(i+1,j,1)], ...
                 [Ps(i,j,2); Ps(i+1,j,2)], ...
                 [Ps(i,j,3); Ps(i+1,j,3)], ...
                 'LineWidth', line_width, 'Color', color);
        end
        if (j < size(Ps,2))
            line([Ps(i,j,1); Ps(i,j+1,1)], ...
                 [Ps(i,j,2); Ps(i,j+1,2)], ...
                 [Ps(i,j,3); Ps(i,j+1,3)], ...
                 'LineWidth', line_width, 'Color', color);
        end
    end
end

board = Ps;