function [f,theta] = InvRot(R)
  h2j = 3.1415926/180;
    %R = cameraParams.RotationMatrices(:,:,i);
    m = sqrt( (R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1)-R(1,2))^2 );
    n = ( R(1,1) + R(2,2) + R(3,3) - 1 );
    theta = atan(m/n);
    
    fx = (R(3,2)-R(2,3))/(2*sin(theta));
    fy = (R(1,3)-R(3,1))/(2*sin(theta));
    fz = (R(2,1)-R(1,2))/(2*sin(theta));
    f = [fx;fy;fz];
end

