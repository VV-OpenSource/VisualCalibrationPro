  i=2;
    %% ��ȡ��������
    src = imread('D:/Matlab/13.VisualCalibMatlabPro/1.IMG/20190314/012.jpg');
    figure(i);subplot(1,2,1),imshow(src); title('ԭͼ��');
    G=src(:,:,2); %��ȡ����ɫͨ��
    img_G = G>245; %��ȡ����������Ϣ
    subplot(2,2,1),imshow(img_G); title('��������');
    kernal = [1,1,1;1,1,1;1,1,1];
    img_R=imdilate(img_G,kernal);
    subplot(2,2,2),imshow(img_G); title('���ͼ�������');
    %% �󼤹����Ʒ���
    [w,h]=size(img_R);
    [y,x]=find(img_R ==1);
    l_eq=polyfit(x,y,1);
    laser_l_eq = l_eq;
    klaser=l_eq(1);     blaser=l_eq(2);
    x0=0; y0=blaser; xn=h; yn = klaser*xn+blaser;
    subplot(2,2,3),imshow(src); title('����������ȡ');
    line([x0 xn],[y0 yn],'color','g','LineWidth',1);
    %% ��ȡ�궨��4���������꼰�䷽��
    p1=cameraParams.ReprojectedPoints(1,:,i);
    aa1=cameraParams.ReprojectedPoints(2,:,i);
    aa2=cameraParams.ReprojectedPoints(3,:,i);
    aa3=cameraParams.ReprojectedPoints(4,:,i);
    p2=cameraParams.ReprojectedPoints(5,:,i);
    bb1=cameraParams.ReprojectedPoints(15,:,i);
    bb2=cameraParams.ReprojectedPoints(25,:,i);
    bb3=cameraParams.ReprojectedPoints(35,:,i);
    p4=cameraParams.ReprojectedPoints(36,:,i);
    cc1=cameraParams.ReprojectedPoints(37,:,i);
    cc2=cameraParams.ReprojectedPoints(38,:,i);
    cc3=cameraParams.ReprojectedPoints(39,:,i);
    p3=cameraParams.ReprojectedPoints(40,:,i);
    dd1=cameraParams.ReprojectedPoints(11,:,i);
    dd2=cameraParams.ReprojectedPoints(21,:,i);
    dd3=cameraParams.ReprojectedPoints(31,:,i);
    [k1,k2,k3,k4,b1,b2,b3,b4] = lineEqua(p1,p2,p3,p4);
    subplot(2,2,4),imshow(src); title('����');
    line([0 h],[b1 k1*h+b1],'color','r','LineWidth',1);
    line([0 h],[b2 k2*h+b2],'color','m','LineWidth',1);
    line([0 h],[b3 k3*h+b3],'color','b','LineWidth',1);
    line([0 h],[b4 k4*h+b4],'color','y','LineWidth',1);
    line([x0 xn],[y0 yn],'color','g','LineWidth',1);
   