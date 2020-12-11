function plot_path(map, path, decomp)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

figure('Name','Animation');


hold on;
for i = 1:size(map.blocks,1)
    block = map.blocks(i, :);
    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [ones(2,1) * block(5); ones(2,1) * block(2); ones(2,1) * block(5); ones(2,1) * block(2)];
    z = [block(3);block(6);block(3);block(6);block(3);block(6);block(3);block(6)];


    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c,'FaceAlpha',.2);

    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [block(2);block(5);block(2);block(5);block(2);block(5);block(2);block(5)];
    z = [ones(2,1) * block(3); ones(2,1) * block(6); ones(2,1) * block(3); ones(2,1) * block(6)];

    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c,'FaceAlpha',.2);
end

if size(path,1) > 0
    % pcshow(path, [0,1,0],'MarkerSize', 0.1);
    set(gcf,'color',[0,0.5,0.5]);
    set(gca,'color','w');
    xlabel('x');
    ylabel('y');
    zlabel('z');

end
axis([map.boundary(1)-1, map.boundary(4)-1, map.boundary(2)-1,map.boundary(5)+1,map.boundary(3)+1,map.boundary(6)+1])

path1 = path;
plot3(path1(:,1), path1(:,2), path1(:,3),'-*','color','k','LineWidth',3);
set(gca,'DataAspectRatio',[1 1 1])
%SFC(path1, map);
% decomp.drawEllipsoids();
% decomp.drawpolyhedron();
% decomp.drawPlanePoint();
   
%view(3); 
view(30, 10);
hold off;

end