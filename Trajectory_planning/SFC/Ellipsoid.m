classdef Ellipsoid < handle
    properties               
        C_;    
        d_;    
        axes_;
    end
    methods                   
        function obj = Ellipsoid (a,b)   
            obj.C_ = a;
            obj.d_ = b;
        end

        % Calculate distance to the center
        function dis = dist(obj, pt)
            % return (C_.inverse() * (pt - d_)).norm();  
            dd = inv(obj.C_) * (pt - obj.d_)';
            dis = dot(dd, dd);   % dot(A,B) 
        end

        % Check if the point is inside
        function ret = inside(obj, pt)
            ret = -1;
            if (obj.dist(pt) <= 1)
                    ret = 1;
            end
        end

        % Calculate points inside ellipsoid
        function pinside = points_inside(obj, points)
            pinside = [];
            [len, ~]=size(points);
            for i = 1 : len
                if(obj.inside(points(i,:)))
                    pinside = [pinside; points(i,:)];
                end
            end
        end

        % Find the closest point
        function p = closest_point(obj, points)
            p = points(1,:);
            % decimal_t min_dist = std::numeric_limits<decimal_t>::max();
            min_dist = 1e8;
            [len, ~]=size(points);
            for i = 1 : len
                d = obj.dist(points(i,:));
                if(d < min_dist)
                    min_dist = d;
                    p = points(i,:);
                end
            end						
        end

        % Find the closest hyperplane from the closest point, 
        function plane = closest_hyperplane(obj, points)
            closest_pt = obj.closest_point(points);
            % const auto  n = C_.inverse() * C_.inverse().transpose() * (closest_pt - d_);
            n = (inv(obj.C_)*(inv(obj.C_))')*(closest_pt - obj.d_)';
            plane = Hyperplane(closest_pt, n);
        end
    end
end
