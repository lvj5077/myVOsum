% clc
A = importdata('/Users/lingqiujin/Desktop/data.txt');
% A = A(all(~isnan(A),2),:);
A = A(:,1:3);
A= A(A(:,3)>2,:);
% A= A(A(:,3)<400,:);
mean(A)
std(A)