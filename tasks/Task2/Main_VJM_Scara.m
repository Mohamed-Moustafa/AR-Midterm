%% Main Scara VJM

data = zeros(3, 3, 3);
data2 = zeros(3, 3, 3);
Force = [0; 0; 100; 0; 0; 0];
pointsize = 45;
theta = zeros(1,4);

for z = 3: 6
    for y = 3: 6
        for x = 3:6
        p_global = [x* 0.1, y*0.1, z*0.1,0];
        
        q=IK_Scara(p_global);
        Jt = Jt_Scara(q,theta);
        
        Kc = Kc_Scara_VJM( Jt);
        
        
        dt_VJM = pinv(Kc)*Force;
        
        D_VJM = sqrt(dt_VJM(1)^2 + dt_VJM(2)^2 + dt_VJM(3)^2);
        data(x, y, z) = D_VJM; 
        
        
        scatter3(x*0.1, y*0.1, z*0.1,pointsize, data(x,y,z));
        hold on
        end
    end
end

colorbar


    