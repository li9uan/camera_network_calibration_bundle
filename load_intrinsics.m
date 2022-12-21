function [Ks] = load_intrinsics(intrinsics_path, num_cams)

if isfile(intrinsics_path)
    String = fileread(intrinsics_path);
    
    for i = 1:num_cams
        start_string = strfind(String, '] = [');
        end_string   = strfind(String, '1.0]');
        sub_string = String(start_string+5:end_string+2);
        Ks{i} = reshape(str2num(sub_string),3,3)';
        String = String(end_string+4:end);
    end
    
    % convert intrinsics image space from (0,0) to (1,1)
    for i = 1:num_cams
        Ks{i}(1,3) = Ks{i}(1,3) + 1;
        Ks{i}(2,3) = Ks{i}(2,3) + 1;
    end
else
    Ks = [];
end