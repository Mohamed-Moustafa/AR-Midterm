%% First try
data = zeros(4, 4, 4);
Force = [0; 0; 100; 0; 0; 0];
pointsize = 20;
for z = 1: 7
    for y = 1: 7
        for x = 1: 7
        p_global = [x* 0.1, y*0.1, z*0.1,0];
        pre_data = Compute_Deflection_Scara( p_global, Force);
        
        deflection = sqrt(pre_data(1)^2 + pre_data(2)^2 + pre_data(3)^2);
        data(x, y, z) = deflection; 
        scatter3(x*0.1, y*0.1, z*0.1,pointsize, data(x,y,z));
        hold on
        end
    end
end
colorbar
