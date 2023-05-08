%% 机器人末端位姿
T61=Trans(0.9489,0.0182,0.6877)*Rot('z',-152.5)*Rot('y',23.9)*Rot('x',-159.6);
T62=Trans(0.9501,0.0147,0.7066)*Rot('z',-164.6)*Rot('y',21.0)*Rot('x',-170.6);
T63=Trans(1.0053,-0.1001,0.7052)*Rot('z',169.9)*Rot('y',23.4)*Rot('x',170.9);
T64=Trans(0.9962,-0.0944,0.6747)*Rot('z',176.6)*Rot('y',28.4)*Rot('x',170.6);
%% 手眼矩阵
%H_E= inv(Trans(0.0061,0.3730,-0.0976)*Rot('z',74.53)*Rot('y',304.15)*Rot('x',11.79));
%% 图像点
p1 = [482;430;1]; p2 = [351;334;1];  p3 = [386;359;1];  p4 = [392;292;1]; 
%% 相机内参矩阵
C_in = (cameraParams.IntrinsicMatrix)';
%% 图像归一化点
[pn1x,pn1y] = Normalized(C_in,p1);
[pn2x,pn2y] = Normalized(C_in,p2);
[pn3x,pn3y] = Normalized(C_in,p3);
[pn4x,pn4y] = Normalized(C_in,p4);
%% 归一化点在摄像机坐标系下坐标
% 1
xc1 = -pn1x/(a*pn1x+b*pn1y+c);
yc1 = -pn1y/(a*pn1x+b*pn1y+c);
zc1 = -1/(a*pn1x+b*pn1y+c);
% 2
xc2 = -pn2x/(a*pn2x+b*pn2y+c);
yc2 = -pn2y/(a*pn2x+b*pn2y+c);
zc2 = -1/(a*pn2x+b*pn2y+c);
% 3
xc3 = -pn3x/(a*pn3x+b*pn3y+c);
yc3 = -pn3y/(a*pn3x+b*pn3y+c);
zc3 = -1/(a*pn3x+b*pn3y+c);
% 4
xc4 = -pn4x/(a*pn4x+b*pn4y+c);
yc4 = -pn4y/(a*pn4x+b*pn4y+c);
zc4 = -1/(a*pn4x+b*pn4y+c);
%% 计算空间坐标位置
pw1 = T61*HandEye*[xc1;yc1;zc1;1]
pw2 = T62*HandEye*[xc2;yc2;zc2;1]
pw3 = T63*HandEye*[xc3;yc3;zc3;1]
pw4 = T64*HandEye*[xc4;yc4;zc4;1]




