clear
% clc
% d = load('/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_0080.dat');
d= load('/Users/lingqiujin/Data/RV_Data2/d1_0302.dat');
z = d(1:144,:);
x = d(1*144+1:2*144,:);
y = d(2*144+1:3*144,:);

% x(1:70,80:170)=0;
% z(1:70,80:170)=0;
% y(1:70,80:170)=0;

c = d(3*144+1:4*144,:);
% c = imadjust(c);
% imshow(c)
conf = d(4*144+1:5*144,:);

CC = reshape(d(3*144+1:4*144,:),[1,144*176]);
z = reshape(z,[1,144*176]);
x = reshape(x,[1,144*176]);
y = reshape(y,[1,144*176]);

pc1 = pointCloud([x;y;z]');
[pc1,indices] = removeInvalidPoints(pc1);
figure,pcshow(pc1)

fx =222.6132;
fy = 225.6439;
cx = 88.5320;
cy = 72.5102;
s =1000;
imdepth = unit16(1000*d(1:144,:));
imdepth = double(imdepth);
Z = imdepth/s;
[h,w] = size(imdepth);
u=repmat(1:w,[h,1]);
v=repmat(1:h,[w,1])';

u=u( (cter(1)-roi):(cter(1)+roi),(cter(2)-roi):(cter(2)+roi));
v=v( (cter(1)-roi):(cter(1)+roi),(cter(2)-roi):(cter(2)+roi));
Z = Z( (cter(1)-roi):(cter(1)+roi),(cter(2)-roi):(cter(2)+roi));

X=(Z(:).*(u(:)-cx))/fx;
Y=(Z(:).*(v(:)-cy))/fy;

xyzPoints = [X(:),Y(:),Z(:)];
xyzPoints = xyzPoints(Z(:)~=0,:);

figure,pcshow(pointCloud(xyzPoints))

%%

d = load('/Users/lingqiujin/Data/RV_Data/Translation/Y2/frm_0080.dat');

z = d(1:144,:);
x = d(1*144+1:2*144,:);
y = d(2*144+1:3*144,:);

% x(1:70,80:170)=0;
% z(1:70,80:170)=0;
% y(1:70,80:170)=0;


c = d(3*144+1:4*144,:);
base = ceil(log(max(max(c)))/log(2))-8;
% c = uint16(c);
c = uint8(c / (2^base));
% c = imadjust(c);
imshow(c)
conf = d(4*144+1:5*144,:);

CC = reshape(d(3*144+1:4*144,:),[1,144*176]);
z = reshape(z,[1,144*176]);
x = reshape(x,[1,144*176]);
y = reshape(y,[1,144*176]);

pc2 = pointCloud([x;y;z]');
[pc2,indices] = removeInvalidPoints(pc2);

% figure,pcshow(pc2)

%%
tform = pcregistericp(pc1,pc2);

disp(tform.T');
%%
% 
% % x = A\B solves the system of linear equations A*x = B
% T = [ 1 0 0 0;0 1 0 0; 0 0 1 0];

Files=dir("/Users/lingqiujin/Data/RV_Data/in/*.dat");
fx = 0;
fy = 0;
cx = 0;
cy = 0;
for k=1: 50
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
            if pz>-5 && pz<-.2
                Ax = [Ax;px/pz 1];
                Bx = [Bx;ww+1];
                Ay = [Ay;py/pz 1];
                By = [By;hh+1];  
            end
        end
    end
    fx_cx = Ax\Bx;
    fy_cy = Ay\By;
    
    fx = fx+ fx_cx(1);
    fy = fy+ fy_cy(1);
    cx = cx+ fx_cx(2);
    cy = cy+ fy_cy(2);
    
end

f_x = fx/k
f_y = fy/k
c_x = cx/k
c_y = cy/k


%%

Files=dir('/Users/lingqiujin/Data/RV_Data2/*.dat');
%     Files=dir([char(dir_name),'/color/*.jpg']);
sumImage = zeros;
for k=1:1100
    d = load([Files(k).folder '/' Files(k).name]);
    c = uint8(d(3*144+1:4*144,:)/(32));
%     c = im2uint8(uint16(d(3*144+1:4*144,:)));
%     c = imadjust(c);
%     imshow(c)
    ffname = ['/Users/lingqiujin/Data/RV_Data2/color/' int2str(k) '.png'];
%     ffname(end-2:end)='png';
    imwrite(c, ffname)
    
    z = 1000*d(1:144,:);
    z = uint16(z);
    ffname = ['/Users/lingqiujin/Data/RV_Data/Pitch/37/depth/' int2str(k) '.png'];
%     ffname(end-2:end)='png';
    imwrite(z, ffname)
end

%%
    
pc1= pcread('/Users/lingqiujin/Q_MAC/work/myVOsum/build/pc1.pcd');
pc2= pcread('/Users/lingqiujin/Q_MAC/work/myVOsum/build/pc2.pcd');
pc= pcread('/Users/lingqiujin/Q_MAC/work/myVOsum/build/pc_12.pcd');
pcshowpair(pc1,pc)
figure,pcshowpair(pc2,pc)