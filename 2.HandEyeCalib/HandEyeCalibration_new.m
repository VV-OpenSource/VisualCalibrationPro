%% ----------------˵�����˳����������۱궨---------------------
clearvars -except  cameraParams  estimationErrors  REs;  clc;
%% ����д����
img_num = 20;           %ͼ������
calib_num = 20;         %����ѭ���궨����
%% ���۱궨
robotEffectorPose;      %������ĩ��λ��
order_all= orderhe(img_num,calib_num);
HE = [];  %�洢ѭ���궨λ��
for order_num = 1:calib_num       
    order = order_all(order_num,:);
    %% ��ĩ�˺������ת��kl, kr.
    kl=[]; kr=[]; theta_r=[]; theta_l=[];
    for i = 1:img_num-2
        %% �������ת��Kr
        Rr1 = cameraParams.RotationMatrices(:,:,order(i));
        Tr1 = cameraParams.TranslationVectors(order(i),:);
        Rr2 = cameraParams.RotationMatrices(:,:,order(i+1));
        Tr2 = cameraParams.TranslationVectors(order(i+1),:);
        Rr3 = cameraParams.RotationMatrices(:,:,order(i+2));
        Tr3 = cameraParams.TranslationVectors(order(i+2),:);
        Rrt1 = [Rr1' (Tr1/1000)';0,0,0,1]/[Rr2' (Tr2/1000)';0,0,0,1];
        Rrt2 = [Rr2' (Tr2/1000)';0,0,0,1]/[Rr3' (Tr3/1000)';0,0,0,1];
        [fr1, thetar1] = InvRot(Rrt1);
        [fr2, thetar2] = InvRot(Rrt2);
        kr = [kr,fr1,fr2,cross(fr1,fr2)];  theta_r = [theta_r;thetar1;thetar2];
        %% ��ĩ��ת��Kl
        Re1 = REs(4*order(i)-3:4*order(i),:);
        Re2 = REs(4*(order(i+1))-3:4*(order(i+1)),:); 
        Re3 = REs(4*(order(i+2))-3:4*(order(i+2)),:); 
        Ret1 = Re1\Re2;
        Ret2 = Re2\Re3;
        Rlt1 = Ret1(1:3,1:3);
        Rlt2 = Ret2(1:3,1:3);
        [fl1, thetal1] = InvRot(Rlt1);
        [fl2, thetal2] = InvRot(Rlt2);
        kl = [kl,fl1,fl2,cross(fl1,fl2)];  theta_l = [theta_l;thetal1;thetal2];
    end
    %% �����۾�����ת��ϵ
    Rm = kl*pinv(kr);      %-----------------------------------------
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% �����۹�ϵƽ������
    Prs=[]; Pls=[];
    for i = 1:img_num-1
    %% ��E-Rli)*Pm = Pli-Rm*Pri ----------�����Ҷ�    
        %% ĩ��ƽ������
        TRl1 = REs(4*order(i)-3:4*order(i),:); 
        TRl2 = REs(4*(order(i+1))-3:4*(order(i+1)),:);   
        Rl = TRl1\TRl2;
        Pli = Rl(1:3,4);            %--------------Pli
        %% ������ƽ������
        TRr1 = cameraParams.RotationMatrices(:,:,order(i));
        TTr1 = cameraParams.TranslationVectors(order(i),:);
        TRr2 = cameraParams.RotationMatrices(:,:,order(i+1));
        TTr2 = cameraParams.TranslationVectors(order(i+1),:);
        TRlt = [TRr1' (TTr1/1000)';0,0,0,1]/[TRr2' (TTr2/1000)';0,0,0,1];
        Pri = TRlt(1:3,4);
        Pr = Pli - Rm*Pri;
        Prs = [Prs;Pr];    %--------------�洢
    %% ��E-Rli)*Pm = Pli-Rm*Pri ----------������� 
        E=[1,0,0;0,1,0;0,0,1];
        Rli = Rl(1:3,1:3);
        Pl = E - Rli;
        Pls = [Pls;Pl];   %--------------�洢
    end
    %% �����۾���ƽ������
    Pm = pinv(Pls)*Prs;  %-----------------------------------------
    H_E = [Rm,Pm;0,0,0,1];
    HE = [HE;H_E];
end
%% �������۾���ƽ������
He(4,4)=0;
for i = 1:calib_num
    he = HE(4*i-3:4*i,:);
    He = He+he;
end
HandEye = He/calib_num
%% �Ƕ���� 
Angle = (theta_r-theta_l)*180/3.1415926;
%% 
clearvars -except  cameraParams  estimationErrors REs HandEye theta_r theta_l Angle order_all;