function [X, new_waypts, ts]= quadraticProgramming(waypts, total_time)

%% condition
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
T = total_time;
n_order = 7;

%% sample mid points
r = 1.63;  %% corridor r
step = 4;
new_waypts = waypts(:,1);
for i=2:size(waypts,2)
    x1 = waypts(1,i-1);
    y1 = waypts(2,i-1);
    z1 = waypts(3,i-1);
    x2 = waypts(1,i);
    y2 = waypts(2,i);
    z2 = waypts(3,i);
    n = ceil( sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2)/step )+1;
    sample_pts = [linspace(x1,x2,n);linspace(y1,y2,n);linspace(z1,z2,n)];
    new_waypts = [new_waypts sample_pts(:,2:end)];
end
ts = arrangeT(new_waypts,T);
    
%% trajectory plan
[polys_x, minValueX] = minimum_snap_single_axis_corridor(new_waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1),r);
[polys_y, minValueY] = minimum_snap_single_axis_corridor(new_waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2),r);
[polys_z, minValueZ] = minimum_snap_single_axis_corridor(new_waypts(3,:),ts,n_order,v0(3),a0(3),v1(3),a1(3),r);

disp(['minValueX is : ',num2str(minValueX)]);
disp(['minValueY is : ', num2str(minValueY)]);
disp(['minValueZ is : ', num2str(minValueZ)]);
disp(['minValue(X + Y + Z) is : ', num2str(minValueX + minValueY + minValueZ)]);

polys_x = changeArrayShape(polys_x);
polys_y = changeArrayShape(polys_y);
polys_z = changeArrayShape(polys_z);
X = [polys_x polys_y polys_z];
new_waypts = new_waypts';

end

function [polys, minValue] = minimum_snap_single_axis_corridor(waypts,ts,n_order,v0,a0,ve,ae,corridor_r)
	p0 = waypts(1);
	pe = waypts(end);
	
	n_poly = length(waypts)-1;
	n_coef = n_order+1;
	
	% compute Q
	Q_all = [];
	for i=1:n_poly
	    Q_all = blkdiag(Q_all,computeQ(n_order,4,ts(i),ts(i+1)));
	end
	b_all = zeros(size(Q_all,1),1);
	

	neq = 8;  %% (8 equations)
	Aeq = zeros(7*(n_poly-1)+neq, n_coef*n_poly);
	beq = zeros(7*(n_poly-1)+neq, 1);
	
	% start/terminal pva constraints  (8 equations)
	Aeq(1:4,1:n_coef) = [calc_tvec(ts(1),n_order,0);
	                     calc_tvec(ts(1),n_order,1);
	                     calc_tvec(ts(1),n_order,2);
	                     calc_tvec(ts(1),n_order,3)];
	Aeq(5:8,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
	                    [calc_tvec(ts(end),n_order,0);
	                     calc_tvec(ts(end),n_order,1);
	                     calc_tvec(ts(end),n_order,2);
	                     calc_tvec(ts(end),n_order,3)];
	beq(1:8,1) = [p0,v0,a0,0,pe,ve,ae,0]';
	
	% continuous constraints  ((n_poly-1)*7 equations)
	for i=1:n_poly-1
		tvec_p = calc_tvec(ts(i+1),n_order,0);
        tvec_v = calc_tvec(ts(i+1),n_order,1);
        tvec_a = calc_tvec(ts(i+1),n_order,2);
        tvec_j = calc_tvec(ts(i+1),n_order,3);  % jerk
        tvec_4 = calc_tvec(ts(i+1),n_order,4);
        tvec_5 = calc_tvec(ts(i+1),n_order,5);
        tvec_6 = calc_tvec(ts(i+1),n_order,6);
        Aeq(neq+1,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
        Aeq(neq+2,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
        Aeq(neq+3,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
        Aeq(neq+4,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_j,-tvec_j];
        Aeq(neq+5,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_4,-tvec_4];
        Aeq(neq+6,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_5,-tvec_5];
        Aeq(neq+7,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_6,-tvec_6];
        neq = neq + 7;
	end
	
	% corridor constraints (n_ploy-1 iequations)
	Aieq = zeros(2*(n_poly-1),n_coef*n_poly);
	bieq = zeros(2*(n_poly-1),1);
	for i=1:n_poly-1
	    tvec_p = calc_tvec(ts(i+1),n_order,0);
	    Aieq(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_p;-tvec_p];
	    bieq(2*i-1:2*i) = [waypts(i+1)+corridor_r corridor_r-waypts(i+1)];
	end
	
	p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);
    
    minValue = p'*Q_all*p;
	
	polys = reshape(p,n_coef,n_poly);
    
    
end


function new_polys = changeArrayShape(oldArray)
	[~, mm] = size(oldArray);
    new_polys = [];
    for i = 1:mm
        new_polys = [new_polys; flip(oldArray(:,i))];
    end
end

