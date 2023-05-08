
T6=Trans(0.8032,-0.0514,0.6280)*Rot('z',-178.3)*Rot('y',38.2)*Rot('x',-178.9);
T6Pose = T6;
for i=1:133  
    tx = T6(1,4)+0.0020;
    T6(1,4) = tx;
    T6Pose = [T6Pose;T6];
end

C_in = (cameraParams.IntrinsicMatrix)';
PW =[];
for j=1:134
    p = weldingpoint(j,:)'; 
    [pnx,pny] = Normalized(C_in,p);

    xc = -pnx/(a*pnx+b*pny+c);
    yc = -pny/(a*pnx+b*pny+c);
    zc = -1/(a*pnx+b*pny+c);
    
    T6j = T6Pose(4*j-3:4*j,:);
    
    pw = T6j*HandEye*[xc;yc;zc;1];
    PW = [PW; pw'];
end