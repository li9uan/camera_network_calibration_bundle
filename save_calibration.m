% save calibration results - intrinsics and camera poses
function save_calibration(file_path, Ks, Rc, Tc, cam_ids)
save(file_path, 'Ks', 'Rc', 'Tc', 'cam_ids');
