function [R4,t4] = get_pose(E)

W = [0 -1 0;1 0 0;0 0 1];
[U,~,V] = svd(E);
R = U*W*V';
C = U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
R4{1} = R;
t4{1} = -R'*C;

R = U*W*V';
C = -U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
R4{2} = R;
t4{2} = -R'*C;

R = U*W'*V';
C = U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
R4{3} = R;
t4{3} = -R'*C;

R = U*W'*V';
C = -U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
R4{4} = R;
t4{4} = -R'*C;
