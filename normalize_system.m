function [Rc_output, Tc_output, Rb_output, Tb_output] = normalize_system(Rc_input, Tc_input, Rb_input, Tb_input)

Rc_output = Rc_input;
Tc_output = Tc_input;
Rb_output = Rb_input;
Tb_output = Tb_input;

transformation = [Rb_input{1} Tb_input{1}; 0 0 0 1];
% make Z look downward
transformation = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1] * inv(transformation);

for i = 1:numel(Rc_input)
    R = Rc_input{i};
    T = Tc_input{i};
    M = [R T; 0 0 0 1];
    M_new = transformation * M;
    Rc_output{i} = M_new(1:3, 1:3);
    Tc_output{i} = M_new(1:3, 4);
end

for i = 1:numel(Rb_input)
    R = Rb_input{i};
    T = Tb_input{i};
    M = [R T; 0 0 0 1];
    M_new = transformation * M;
    Rb_output{i} = M_new(1:3, 1:3);
    Tb_output{i} = M_new(1:3, 4);
end