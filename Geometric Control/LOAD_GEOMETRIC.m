% Trajectory data
load('POS_40.mat')
load('VEL_40.mat')
load('ACC_40.mat')

T = 40; %trajDuration

% References definition
xx = PP(1,:);
yy = PP(2,:); 
zz = PP(3,:); 
xv = VV(1,:);
yv = VV(2,:); 
zv = VV(3,:); 
xa = AA(1,:);
ya = AA(2,:); 
za = AA(3,:); 
tt = PP(4,:);

% PSI Computation
PSI = zeros(1, length(PP));
for i = 1:length(PP)-1
    PSI(1,i) = atan2(PP(2,i+1) - PP(2,i), PP(1,i+1) - PP(1,i));
end
PSI(1,end) = PSI(1,end-1);

for i = 579:609
    PSI(1,i) = PSI(1,i) - 2*pi;
end

for i = 1177:4001
    PSI(1,i) = PSI(1,i) + 2*pi;
end

for i = 1914:1943
    PSI(1,i) = PSI(1,i) - 2*pi;
end

for i = 2510:4001
    PSI(1,i) = PSI(1,i) + 2*pi;
end

for i = 3247:3277
    PSI(1,i) = PSI(1,i) - 2*pi;
end

for i = 3842:4001
    PSI(1,i) = PSI(1,i) + 2*pi;
end


