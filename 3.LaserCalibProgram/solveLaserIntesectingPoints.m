function [img_q1,img_q2,qr1,qr2,laser_l_eq] = solveLaserIntesectingPoints(src,cameraParams,i)
    %%% q1,q2�������꣬  qr1,qr2�����ڱ������ϵʵ����ά����,   iΪͼ������
    %% ��ȡ��������
    %src = imread('E:/Matlab/6.JIGUANG/20181128/02.bmp');
    figure(i);subplot(1,2,1),imshow(src); title('ԭͼ��');
    G=src(:,:,2); %��ȡ����ɫͨ��
    img_G = G==255; %��ȡ����������Ϣ
    %subplot(2,2,2),imshow(img_R); title('��������');
    kernal = [1,1,1;1,1,1;1,1,1];
    img_G=imdilate(img_G,kernal);
    %subplot(1,3,2),imshow(img_R); title('���ͼ�������');
    %% �󼤹����Ʒ���
    [w,h]=size(img_G);
    [y,x]=find(img_G ==1);
    l_eq=polyfit(x,y,1);
    laser_l_eq = l_eq;
    klaser=l_eq(1);     blaser=l_eq(2);
    x0=0; y0=blaser; xn=h; yn = klaser*xn+blaser;
    %subplot(1,2,2),imshow(src); title('����������ȡ');
    %line([x0 xn],[y0 yn],'color','g','LineWidth',1);
    %% ��ȡ�궨��4���������꼰�䷽��
    p1=cameraParams.ReprojectedPoints(1,:,i);
    aa1=cameraParams.ReprojectedPoints(2,:,i);
    aa2=cameraParams.ReprojectedPoints(3,:,i);
    aa3=cameraParams.ReprojectedPoints(4,:,i);
    p2=cameraParams.ReprojectedPoints(5,:,i);
    bb01=cameraParams.ReprojectedPoints(10,:,i);%bb01Ϊp2��bb1�м�ĵ�
    bb1=cameraParams.ReprojectedPoints(15,:,i); 
    bb02=cameraParams.ReprojectedPoints(20,:,i);%bb02Ϊbb1��bb2�м�ĵ�
    bb2=cameraParams.ReprojectedPoints(25,:,i);
    bb03=cameraParams.ReprojectedPoints(30,:,i);%bb03Ϊbb2��bb3�м�ĵ�
    bb3=cameraParams.ReprojectedPoints(35,:,i);
    p4=cameraParams.ReprojectedPoints(36,:,i);
    cc1=cameraParams.ReprojectedPoints(37,:,i);
    cc2=cameraParams.ReprojectedPoints(38,:,i);
    cc3=cameraParams.ReprojectedPoints(39,:,i);
    p3=cameraParams.ReprojectedPoints(40,:,i);
    dd01=cameraParams.ReprojectedPoints(6,:,i);%dd01Ϊp1��dd1�м�ĵ�
    dd1=cameraParams.ReprojectedPoints(11,:,i);
    dd02=cameraParams.ReprojectedPoints(16,:,i);%dd02Ϊdd1��dd2�м�ĵ�
    dd2=cameraParams.ReprojectedPoints(21,:,i);
    dd03=cameraParams.ReprojectedPoints(26,:,i);%dd03Ϊdd2��dd3�м�ĵ�
    dd3=cameraParams.ReprojectedPoints(31,:,i);
    [k1,k2,k3,k4,b1,b2,b3,b4] = lineEqua(p1,p2,p3,p4);
    subplot(1,2,2),imshow(src); title('����');
    line([0 h],[b1 k1*h+b1],'color','r','LineWidth',1);
    line([0 h],[b2 k2*h+b2],'color','m','LineWidth',1);
    line([0 h],[b3 k3*h+b3],'color','b','LineWidth',1);
    line([0 h],[b4 k4*h+b4],'color','y','LineWidth',1);
    line([x0 xn],[y0 yn],'color','g','LineWidth',1);
    %% ������Χ���ΰ���
    xv=[p1(1) p2(1) p3(1) p4(1) p1(1)];
    yv=[p1(2) p2(2) p3(2) p4(2) p1(2)];
    %% �󼤹����ƽ���q1,q2
    q1=[0,0];  q2=[0,0]; 
    x1=(b1-blaser)/(klaser-k1);    y1=(klaser*b1-k1*blaser)/(klaser-k1);  
    if(inpolygon(x1,y1,xv,yv))     
        q1=[x1,y1];
        l_q1=sqrt( (q1(1)-p1(1))^2+(q1(2)-p1(2))^2 );
        l_q1r=pixAugment(p1,aa1,aa2,aa3,[0,0],[0,0],[0,0],l_q1,'s');%��̱�ֻ��3���㣬����3��[0,0]�������
        qr1 = [0,l_q1r,1];
    end
     %//////////////////////
    x2=(b2-blaser)/(klaser-k2);    y2=(klaser*b2-k2*blaser)/(klaser-k2);
    if(inpolygon(x2,y2,xv,yv)&&q1(1)&&q1(2))
         q2=[x2,y2];
         l_q2=sqrt( (q2(1)-p2(1))^2+(q2(2)-p2(2))^2 );
         l_q2r = pixAugment(p2,bb1,bb2,bb3,bb01,bb02,bb03,l_q2,'l');
         qr2 = [l_q2r,0.020,1];
    else  if(inpolygon(x2,y2,xv,yv))
            q1=[x2,y2];
            l_q2=sqrt( (q1(1)-p2(1))^2+(q1(2)-p2(2))^2 );
            l_q2r = pixAugment(p2,bb1,bb2,bb3,bb01,bb02,bb03,l_q2,'l');
            qr1 = [l_q2r,0.020,1];
          end
    end
     %/////////////////////
    x3=(b3-blaser)/(klaser-k3);    y3=(klaser*b3-k3*blaser)/(klaser-k3);
    if(inpolygon(x3,y3,xv,yv)&&q1(1)&&q1(2)) 
         q2=[x3,y3];
         l_q3=sqrt( (q2(1)-p4(1))^2+(q2(2)-p4(2))^2 );
         l_q3r = pixAugment(p4,cc1,cc2,cc3,[0,0],[0,0],[0,0],l_q3,'s');
         qr2 = [0.035,l_q3r,1];
    else if(inpolygon(x3,y3,xv,yv)) 
         q1=[x3,y3];
         l_q3=sqrt( (q1(1)-p4(1))^2+(q1(2)-p4(2))^2 );
         l_q3r = pixAugment(p4,cc1,cc2,cc3,[0,0],[0,0],[0,0],l_q3,'s');
         qr1 = [0.035,l_q3r,1];
         end
    end
      %////////////////////
    x4=(b4-blaser)/(klaser-k4);    y4=(klaser*b4-k4*blaser)/(klaser-k4);
    if(inpolygon(x4,y4,xv,yv))
         q2=[x4,y4];
         l_q1=sqrt( (q2(1)-p1(1))^2+(q2(2)-p1(2))^2 );
         l_q2r = pixAugment(p1,dd1,dd2,dd3,dd01,dd02,dd03,l_q1,'l');
         qr2 = [l_q2r,0,1];
    end
    img_q1 = q1; img_q2 = q2;
end