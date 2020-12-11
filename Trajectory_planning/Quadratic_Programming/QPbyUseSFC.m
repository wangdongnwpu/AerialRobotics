function X = QPbyUseSFC(waypts, ts, decomp)

%% condition
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
n_order = 7;

%% trajectory plan
[minSnapValue, px, py, pz] = minimum_snap_single_axis_corridor(waypts,ts,n_order,v0,a0,v1,a1, decomp);

disp(['minSnapValue is : ',num2str(minSnapValue)]);


polys_x = changeArrayShape(px);
polys_y = changeArrayShape(py);
polys_z = changeArrayShape(pz);

X = [polys_x polys_y polys_z];

end

% waypts is a [3*N] matrix,
% v0 = [1*3]
function [minValue, px, py, pz] = minimum_snap_single_axis_corridor(waypts,ts,n_order,v0,a0,ve,ae, decomp)
    % Dimension
    dim = 3;
	p0 = waypts(:,1);
	pe = waypts(:,end);
	
	n_poly = size(waypts, 2) - 1;
	n_coef = n_order+1;
	
	%% compute Q, because Q_x == Q_y == Q_z
    % Q_all = [Q_x  0     0]
    %         [0    Q_y   0]
    %         [0    0   Q_z]
    Q_x = [];
	for i=1:n_poly
	    Q_x = blkdiag(Q_x,computeQ(n_order,4,ts(i),ts(i+1)));
    end
    zeroM = zeros(size(Q_x));
    Q_all = [Q_x zeroM zeroM; zeroM Q_x zeroM; zeroM zeroM Q_x];
    xlen = size(Q_all,1);
	b_all = zeros(xlen,1);
	
    %% compute Aeq x = beq
    % beacuse Aeq_x == Aeq_y == Aeq_z
    % Aeq = [Aeq_x  0      0]
    %       [0    Aeq_y    0]
    %       [0      0  Aeq_z] 
	neq = 8;  %% (8 equations)
	Aeq_x = zeros(7*(n_poly-1)+neq, n_coef*n_poly);
    eqNum_x = (7*(n_poly-1)+neq);
	beq = zeros(dim*eqNum_x, 1);
	
	% start/terminal pva constraints  (8 equations)
	Aeq_x(1:4,1:n_coef) = [calc_tvec(ts(1),n_order,0);
	                     calc_tvec(ts(1),n_order,1);
	                     calc_tvec(ts(1),n_order,2);
	                     calc_tvec(ts(1),n_order,3)];
	Aeq_x(5:8,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
	                    [calc_tvec(ts(end),n_order,0);
	                     calc_tvec(ts(end),n_order,1);
	                     calc_tvec(ts(end),n_order,2);
	                     calc_tvec(ts(end),n_order,3)];
	beq(1:8,1) = [p0(1),v0(1),a0(1),0,pe(1),ve(1),ae(1),0]';
    beq((eqNum_x   + 1):(eqNum_x   + 8),1) = [p0(2),v0(2),a0(2),0,pe(2),ve(2),ae(2),0]';
    beq((eqNum_x*2 + 1):(eqNum_x*2 + 8),1) = [p0(3),v0(3),a0(3),0,pe(3),ve(3),ae(3),0]';
	
	% continuous constraints  ((n_poly-1)*7 equations)
	for i=1:n_poly-1
		tvec_p = calc_tvec(ts(i+1),n_order,0);
        tvec_v = calc_tvec(ts(i+1),n_order,1);
        tvec_a = calc_tvec(ts(i+1),n_order,2);
        tvec_j = calc_tvec(ts(i+1),n_order,3);  % jerk
        tvec_4 = calc_tvec(ts(i+1),n_order,4);
        tvec_5 = calc_tvec(ts(i+1),n_order,5);
        tvec_6 = calc_tvec(ts(i+1),n_order,6);
        Aeq_x(neq+1,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
        Aeq_x(neq+2,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
        Aeq_x(neq+3,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
        Aeq_x(neq+4,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_j,-tvec_j];
        Aeq_x(neq+5,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_4,-tvec_4];
        Aeq_x(neq+6,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_5,-tvec_5];
        Aeq_x(neq+7,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_6,-tvec_6];
        neq = neq + 7;
    end
	
  
    zeroM = zeros(size(Aeq_x)); 
    Aeq = [Aeq_x zeroM zeroM; zeroM Aeq_x zeroM; zeroM zeroM Aeq_x];
    
    %% compute Aieq x <= bieq
    Aieq = [];
    bieq = [];
    ieq = 1;
    xll = xlen / dim;  % 1/3 of xlen,  = n_coef * (n_poly-1)
    for i = 1 : n_poly-1
        planesCell = decomp.lines_{i}.polyhedron_.polys_;
        [~, lenj] = size(planesCell);
        tvec_p = calc_tvec(ts(i+1),n_order,0);   % [1*8]vector
        tvec_f = calc_tvec(ts(i),n_order,0); 
        for j = 1 : lenj
            n = planesCell{j}.n_;
            p = planesCell{j}.p_;
            nx = zeros(1, xlen);
            % add back point on the poly's Ax<b 
            nx(1, 8*(i-1) + 1 : 8*(i-1) + 8)                = tvec_p.*n(1);
            nx(1, 8*(i-1) + 1 + xll : 8*(i-1) + 8 + xll)    = tvec_p.*n(2);
            nx(1, 8*(i-1) + 1 + 2*xll : 8*(i-1) + 8 + 2*xll)= tvec_p.*n(3);
            Aieq(ieq,:) = nx;
            bieq(ieq,:) = dot(n, p);
            ieq = ieq + 1;
            
            % --------------------------------------------
            % add front point on the poly's Ax<b 
            nx(1, 8*(i-1) + 1 : 8*(i-1) + 8)                = tvec_f.*n(1);
            nx(1, 8*(i-1) + 1 + xll : 8*(i-1) + 8 + xll)    = tvec_f.*n(2);
            nx(1, 8*(i-1) + 1 + 2*xll : 8*(i-1) + 8 + 2*xll)= tvec_f.*n(3);
            Aieq(ieq,:) = nx;
            bieq(ieq,:) = dot(n, p);
            ieq = ieq + 1;
            % --------------------------------------------
        end
    end
    
	p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);
    
    minValue = p'*Q_all*p;
	
	px = reshape(p(1:xll,1),n_coef,n_poly);
    py = reshape(p(xll + 1:2*xll,1),n_coef,n_poly);
    pz = reshape(p(xll*2 + 1:3*xll,1),n_coef,n_poly);
end


function new_polys = changeArrayShape(oldArray)
	[~, mm] = size(oldArray);
    new_polys = [];
    for i = 1:mm
        new_polys = [new_polys; flip(oldArray(:,i))];
    end
end

