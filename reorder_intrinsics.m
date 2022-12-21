function newKs = reorder_intrinsics(Ks, cam_ids)

for i = 1:numel(cam_ids)
    fprintf('new %d - loaded %d\n', i, cam_ids(i));
    newKs{i} = Ks{cam_ids(i)};
end