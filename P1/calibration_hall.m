function [ A ] = calibration_hall( xy, xyz )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    NUMPTS = size(xy,2);
    Q = zeros(2*NUMPTS, 11);
    B = zeros(2*NUMPTS, 1);

    for r=1 : NUMPTS

        idx = 2*r-1;
        xu=xy(1, r);  yu=xy(2, r);
        xw=xyz(1, r); yw=xyz(2, r);  zw=xyz(3, r);

        Q(idx  , :) = [ xw yw zw  1  0  0  0 0 -xu*xw  -xu*yw  -xu*zw ];
        Q(idx+1, :) = [  0  0  0  0 xw yw zw 1 -yu*xw  -yu*yw  -yu*zw ];

        B(idx)   = xu;
        B(idx+1) = yu;
    end

    %Get the calibration matrix using Least Squares
    A = (Q'*Q) \ (Q'*B);
    A = [A; 1];
    A = reshape(A, 4, 3)';

end

