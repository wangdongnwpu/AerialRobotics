function R = rotationMatrix(v1col, v2col)
    v1 = v1col';
    v2 = v2col';
    nv1 = v1/norm(v1);
    nv2 = v2/norm(v2);

    if norm(nv1+nv2)==0
        q = [0 0 0 0];
    else
        u = cross(nv1,nv2);         
        u = u/norm(u);

        theta = acos(sum(nv1.*nv2))/2;
        q = [cos(theta) sin(theta)*u];
    end

    R=[2*q(1).^2-1+2*q(2)^2  2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
        2*(q(2)*q(3)-q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2 2*(q(3)*q(4)+q(1)*q(2));
        2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2];
	R = R';
end
