colors{1} = [1 0 0];
colors{2} = [0 1 0];
colors{3} = [0 0 1];
colors{4} = [0.5 0.5 0.5];
colors{5} = [0 1 1];
colors{6} = [0 0 0];
colors{7} = [1 1 0];
colors{8} = [1 0 1];
colors{9} = [0.5 0 0.5];
colors{10} = [0.0 0.5 0.5];
colors{11} = [0.0 0.0 0.5];
colors{12} = [0.5 0.0 0.0];

% for following colors
n = 100;
cc = rand(n,3);
for i = 13:n
    ccc = cc(i,:)/norm(cc(i,:));
    colors{i} = ccc;
end