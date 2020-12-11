 hold on;
 len = length(obs);
 for i = 1 : len
     plot3(obs(i,1), obs(i,2), obs(i,3), '-o', 'Color', 'b');
 end
 hold off;