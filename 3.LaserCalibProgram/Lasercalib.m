%% ----------------˵�����˳��������߼���ƽ��궨---------------------
% ���Matlab Camera Calibrationģ��,�Լ���ƽ�淽��ax+by+cz+1=0������⡣
%ʹ��˵����
    % 1.�ɼ��㹻������̸�-�߼���ͼ��
    % 2.��Camera Calibrationģ�����������б궨�����������cameraParams��
    % 3.����ͼƬ��������img_num��
    % 4.ָ��ͼ����ڵ��ļ���·����
    % 5.���г��򣬵õ�����a,b,c��
%%--------------------------------------------------------------------------
clearvars -except  cameraParams  estimationErrors REs H_E Angle;
%% �󼤹�������궨����Χ����
img_num = 20;     %ͼ������
img_p=[];         %�����ͼ������
itstpc=[];        %������ά���������������ϵ�´洢�ռ�
qr = [];
laseri_l_eq = [];
for i=1:img_num
    src = imread(['F:\JZW\1.TestFiles\20191026LY/0',num2str(i,'%d'),'.jpg']);
    [img_q1,img_q2,qr1,qr2,laser_l_eq] = solveLaserIntesectingPoints(src,cameraParams,i);
    img_p=[img_p;img_q1;img_q2];  qr = [qr;qr1,qr2];  laseri_l_eq = [laseri_l_eq;laser_l_eq];
    %% ����ξ��� 
    R = cameraParams.RotationMatrices(:,:,i);
    T = cameraParams.TranslationVectors(i,:);
    Mc_en = [R',(T/1000)'];     %��ξ���
    %% �󽻵������������ϵ������
    Pc1 = Mc_en*[qr1(1);qr1(2);0;1];
    Pc2 = Mc_en*[qr2(1);qr2(2);0;1];
    itstpc=[itstpc,Pc1,Pc2];
end
%% ��⼤��ƽ�淽�̲���
laser_pc = itstpc';
%-------����E����-------
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
%% ������
D = [];
for k=1:2*img_num
    d = (abs(a*itstpc(1,k)+b*itstpc(2,k)+c*itstpc(3,k)+1))/(sqrt(a^2+b^2+c^2));
    D = [D;d]; 
end
   DM = mean(D)    %ƽ�����
clearvars -except  cameraParams  estimationErrors REs H_E Angle img_p qr laseri_l_eq a b c DM;