function decomp = SFC_3D(path, obs)
    % Initialize MultipleSegments
	local_bbox = [2 2]; offset_x = 0;
    decomp = MultipleSegments(local_bbox);
    decomp.set_obs(obs);
    decomp.dilate(path, offset_x);
end

