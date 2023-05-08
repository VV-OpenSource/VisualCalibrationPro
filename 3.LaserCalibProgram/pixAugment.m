function [lqr] = pixAugment(p,c1,c2,c3,cl1,cl2,cl3,lq,charter)
    %pΪ��ʼ�㣬p,c1,c2,c3���ߣ� c1Ϊ��1�㣬c2Ϊ��2�㣬c3Ϊ��3��
    %lqΪ���㵽��ʼ�㳤�ȣ�charterΪ���̱��жϣ�'s'Ϊ�̣�'l'Ϊ����
    %cl1,cl2,cl3�ֱ�Ϊ������c1,c2,c3���м��
    
    l_q1=sqrt( (c1(1)-p(1))^2+(c1(2)-p(2))^2 );
    l_q2=sqrt( (c2(1)-p(1))^2+(c2(2)-p(2))^2 );
    l_q3=sqrt( (c3(1)-p(1))^2+(c3(2)-p(2))^2 );
    l_qc1=sqrt( (cl1(1)-p(1))^2+(cl1(2)-p(2))^2 );
    l_qc2=sqrt( (cl2(1)-p(1))^2+(cl2(2)-p(2))^2 );
    l_qc3=sqrt( (cl3(1)-p(1))^2+(cl3(2)-p(2))^2 );
    
    if (charter=='s')
        A=[l_q1,0.005;l_q2,0.010;l_q3,0.015];
        B=[1;1;1];
    end
    
    if (charter=='l')
        A=[l_q1,0.010;l_q2,0.020;l_q3,0.030;l_qc1,0.005;l_qc2,0.015;l_qc3,0.025];
        B=[1;1;1;1;1;1];
    end
    
    t = pinv(A)*B;
    lqr = 1/t(2)-t(1)*lq/t(2);
end