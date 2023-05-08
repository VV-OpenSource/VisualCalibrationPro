src = imread('E:/Matlab/6.JIGUANG/20181123/src/03.bmp');
dst = undistortImage(src,cameraParams);
imwrite(dst,'E:/Matlab/6.JIGUANG/20181123/src/dst/undistorimg_03.bmp');
