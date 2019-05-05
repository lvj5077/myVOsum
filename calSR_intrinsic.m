clear
clc
d = load('/Users/lingqiujin/Data/test/frm_0089.dat');

% z = d(1:144,:);
% x = d(1*144+1:2*144,:);
% y = d(2*144+1:3*144,:);
% 
% % 
% % c = d(3*144+1:4*144,:);
% % c = im2uint8(uint16(d(3*144+1:4*144,:)));
% % c = imadjust(c);
% % imshow(c)
% % conf = d(4*144+1:5*144,:);
% % 
% % CC = reshape(d(3*144+1:4*144,:),[1,144*176]);
% % z = reshape(z,[1,144*176]);
% % x = reshape(x,[1,144*176]);
% % y = reshape(y,[1,144*176]);
% % pcshow(pointCloud([x;y;z]','Intensity',double(CC)))
% 
% % x = A\B solves the system of linear equations A*x = B
% T = [ 1 0 0 0;0 1 0 0; 0 0 1 0];

% Files1=dir("/Users/lingqiujin/Data/RV_Data/in/*.dat");
% Files2=dir("/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/*.dat");
Files=dir("/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_0044.dat");
% Files = [Files2;Files3];
% Files = Files1;
fx = 0;
fy = 0;
cx = 0;
cy = 0;
v=0;
for k=1:1:1
    v = v+1;
    d = load([Files(k).folder '/' Files(k).name]);
    
    z = d(1:144,:);
    x = d(1*144+1:2*144,:);
    y = d(2*144+1:3*144,:);

    Ax = [];
    Bx = [];
    Ay = [];
    By = [];
    for ww = 0:175
        for hh = 0:143
            px = x(hh+1,ww+1);
            py = y(hh+1,ww+1);
            pz = -z(hh+1,ww+1);
%             if pz>-8 && pz<-.2
            Ax = [Ax;px/pz 1];
            Bx = [Bx;ww];
            Ay = [Ay;py/pz 1];
            By = [By;hh];  
%             end
        end
    end
    fx_cx = Ax\Bx;
    fy_cy = Ay\By;
    
    fx = fx+ fx_cx(1);
    fy = fy+ fy_cy(1);
    cx = cx+ fx_cx(2);
    cy = cy+ fy_cy(2);
    
end

f_x = fx/v
f_y = fy/v
c_x = cx/v
c_y = cy/v
%%
A=[0.4101, -0.3008, 1.3448;0.142, -0.1288, 1.6359;0.38, -0.3082, 1.3334;0.1792, 0.4181, 1.8994;.3637, 0.0712, 1.8127;0.3342, 0.011, 1.7862;0.1838, 0.2002, 2.0183;0.4546, 0.0427, 1.8469;0.2893, -0.2951, 1.3472;0.1341, 0.3049, 1.9906]
B=[0.3713, -0.3253, 1.4972;0.0695, -0.2084, 1.6318;0.3347, -0.3217, 1.4966;0.0943, 0.4807, 2.1551;0.25, 0.074, 1.7314;0.2452, 0.0117, 1.9254;0.0725, 0.1762, 1.7137;0.3294, 0.046, 1.7185;0.2384, -0.3211, 1.4957;0.0259, 0.2844, 1.8128]

A= [0.0174, -0.3023, 1.6942;0.0036, -0.43, 1.7142;0.261, -0.351, 1.3752;0.3328, -0.3808, 1.324;0.3328, -0.3808, 1.324;0.299, -0.2293, 1.3584;0.299, -0.2293, 1.3584]
B= [0.2337, -0.4637, 1.653;0.2337, -0.4637, 1.653;0.2757, -0.1978, 1.5589;0.3467, -0.2343, 1.4551;0.3467, -0.2343, 1.4551;0.3184, -0.0835, 1.483;0.3184, -0.0835, 1.483]

% %%
% 
% Files=dir('/Users/lingqiujin/Data/test/*.dat');
% %     Files=dir([char(dir_name),'/color/*.jpg']);
% sumImage = zeros;
% for k=1:6
%     d = load([Files(k).folder '/' Files(k).name]);
% %     c = uint16(d(3*144+1:4*144,:));
%     c = uint8(d(3*144+1:4*144,:)/(2^8));
% %     c = im2uint8(uint16(d(3*144+1:4*144,:)));
%     c = imadjust(c);
%     ffname = [Files(k).folder '/dark_' Files(k).name];
%     ffname(end-2:end)='png';
%     imwrite(c, ffname)
% 
% %     imwrite(ind2rgb(im2uint8(mat2gray(c)), parula(256)), ffname)
% end