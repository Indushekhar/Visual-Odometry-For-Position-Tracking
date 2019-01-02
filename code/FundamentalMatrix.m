function F = FundamentalMatrix(x1, x2)

point_size = size(x1,1);
%% Normalizing the Image points
x1_x = x1(:,1);
x2_x = x2(:,1);
x1_y = x1(:,2);
x2_y = x2(:,2);

%% First Transform
mean_x1_x = mean(x1_x);
mean_x1_y = mean(x1_y);
x1_x = x1_x - mean_x1_x * ones(point_size,1);
x1_y = x1_y - mean_x1_y * ones(point_size,1);
avg_d = sqrt(sum(x1_x.^2  + x1_y.^2)) / point_size;
scaling_factor = sqrt(2) / avg_d;
x1(:,1) = scaling_factor * x1_x;
x1(:,2) = scaling_factor * x1_y;
dx = (-scaling_factor*mean_x1_x);
dy = (-scaling_factor*mean_x1_y);

T_1 = [scaling_factor,0,dx;0,scaling_factor,dy;0,0,1];  

%% Second Transform
mean_x2_x = mean(x2_x);
mean_x2_y = mean(x2_y);
x2_x = x2_x - mean_x2_x * ones(point_size,1);
x2_y = x2_y - mean_x2_y * ones(point_size,1);
avg_d = sqrt(sum(x2_x.^2  + x2_y.^2)) / point_size;
scaling_factor = sqrt(2) / avg_d;
x2(:,1) = scaling_factor * x2_x;
x2(:,2) = scaling_factor * x2_y;
T_2 = [scaling_factor 0 -scaling_factor*mean_x2_x;
       0 scaling_factor -scaling_factor*mean_x2_y;
       0 0 1];


A_Mat = [x1(:,1).* x2(:,1) x1(:,1).* x2(:,2) x1(:,1) x1(:,2).* x2(:,1) x1(:,2) .* x2(:,2) x1(:,2) x2(:,1) x2(:,2) ones(size(x1,1),1)];
[~,~,v] = svd(A_Mat);
F_lin = v(:,end);
F_rank3 =reshape(F_lin,3,3);
F_rank3 = F_rank3 / norm(F_rank3);
[u,d,v] = svd(F_rank3);
d(3,3) = 0;
F_rank2 = u * d * v';
F = T_2' * F_rank2 * T_1;
end

