%% ----------------说明：此程序用于线激光平面标定---------------------
% 结合Matlab Camera Calibration模块,对激光平面方程ax+by+cz+1=0进行求解。
%使用说明：
    % 1.采集足够多的棋盘格-线激光图像；
    % 2.用Camera Calibration模块对摄像机进行标定，并导出结果cameraParams；
    % 3.设置图片的总张数img_num；
    % 4.指定图像存在的文件夹路径；
    % 5.运行程序，得到参数a,b,c。
%%--------------------------------------------------------------------------
clearvars -except  cameraParams  estimationErrors REs H_E Angle;
%% 求激光条纹与标定板外围交点
img_num = 20;     %图像张数
img_p=[];         %交点的图像坐标
itstpc=[];        %交点三维坐标在摄像机坐标系下存储空间
qr = [];
laseri_l_eq = [];
for i=1:img_num
    src = imread(['F:\JZW\1.TestFiles\20191026LY/0',num2str(i,'%d'),'.jpg']);
    [img_q1,img_q2,qr1,qr2,laser_l_eq] = solveLaserIntesectingPoints(src,cameraParams,i);
    img_p=[img_p;img_q1;img_q2];  qr = [qr;qr1,qr2];  laseri_l_eq = [laseri_l_eq;laser_l_eq];
    %% 求外参矩阵 
    R = cameraParams.RotationMatrices(:,:,i);
    T = cameraParams.TranslationVectors(i,:);
    Mc_en = [R',(T/1000)'];     %外参矩阵
    %% 求交点在摄像机坐标系下坐标
    Pc1 = Mc_en*[qr1(1);qr1(2);0;1];
    Pc2 = Mc_en*[qr2(1);qr2(2);0;1];
    itstpc=[itstpc,Pc1,Pc2];
end
%% 求解激光平面方程参数
laser_pc = itstpc';
%-------构建E矩阵-------
E=[];
for j=1:2*img_num
    e=-1;
    E=[E;e];
end
%----------------------
X=pinv(laser_pc)*E;
a=X(1)
b=X(2)
c=X(3)
%% 误差分析
D = [];
for k=1:2*img_num
    d = (abs(a*itstpc(1,k)+b*itstpc(2,k)+c*itstpc(3,k)+1))/(sqrt(a^2+b^2+c^2));
    D = [D;d]; 
end
   DM = mean(D)    %平均误差
clearvars -except  cameraParams  estimationErrors REs H_E Angle img_p qr laseri_l_eq a b c DM;