% load corner detection

function detection = load_detection_python(dir)
if isfile(dir)
    String = fileread(dir);
    String(strfind(String, '[')) = ' ';
    String(strfind(String, ']')) = ' ';
    String(strfind(String, ',')) = ' ';
    detection = str2num(String);
else
    detection = [];
end