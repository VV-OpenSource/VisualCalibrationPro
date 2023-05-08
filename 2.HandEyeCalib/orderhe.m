function [order] = orderhe(img_num,he_num)
%产生手眼标定的读取顺序，he_num应小于或等于img_num,或者会产生重复顺序

   order(he_num,img_num) = 0;
 
    for num_i = 1:he_num
        number = num_i-1;
        for num_j = 1:img_num
            number = number+1;
            if number>img_num
                number = number-img_num;
                order(num_i,num_j) =  number;
            else
                order(num_i,num_j) =  number;
            end
        end
    end           
end