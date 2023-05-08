function [ Normalizedx,Normalizedy ] = Normalized(c_in,p)
%น้าปปฏ
B=[p(1);
   p(2);
   1];

A=c_in\B;
Normalizedx=A(1,1);
Normalizedy=A(2,1);
end