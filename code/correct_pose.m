function [R, t] = correct_pose(R4,t4)

ind = [];

%% Case One -  Z should always be positive - with the assumption that car is always moving forward
for i = 1: length(t4)
    if (t4{i}(3,1) > 0)
        ind = [ind;i];
    end
end

%% Case two-rotational matrix 
new_Rset = {};
j = 1;
ind_2 = [];
if (size(ind,1) > 0)
    for i=1:size(ind,1)
        R_test = R4{ind(i)};
       if R_test(2,2) > 0.9 && abs(R_test(1,2)) < 0.1 && abs(R_test(2,1)) < 0.1 && ...
          abs(R_test(2,3)) < 0.1 && abs(R_test(3,2)) < 0.1
          new_Rset{j} = R_test;
          j = j + 1;
          ind_2 = [ind_2;ind(i)];
       end
    end
%% case three - pick the matrix with minimum y translation
    if (size(ind_2,1) > 0)
        R = new_Rset{1};
        t = [t4{ind_2(1)}(1);0;t4{ind_2(1)}(3)];
        min_y = abs(t4{ind_2(1)}(2));
        for i = 2: size(ind_2,1)
            curr_min_y = abs(t4{ind_2(i)}(2));
            if curr_min_y < min_y
                min_y = curr_min_y;
                R = new_Rset{i};
                t = [t4{ind_2(i)}(1);0;t4{ind_2(i)}(3)];
            end
        end
 %% case four -> Restrict rotation only about the Y axis
        R(1,2) = 0;
        R(2,1) = 0;
        R(2,3) = 0;
        R(3,2) = 0;
        
 %% Filtering the Noises- 
        if abs(R(1,3)) < 0.005
            R(1,3) = 0;
        end
        if abs(R(3,1)) < 0.005
            R(3,1) = 0;
        end  
        
 %% case five -> Limiting the side movement
        if abs(t(1)) < 0 || R(1,1) > 0.90
            t = [0;0;t(3)];
        end
    else
        R = eye(3);
        t = [0;0;0];
    end
else
    R = eye(3);
    t = [0;0;0];
end