clear; clc; close all;

im = imread('images/chessboard03.png'); 
im_orig = im;

if size(im,3)>1 
    im=rgb2gray(im); 
end % Derivative masks

dx = [  -1 0 1;
        -1 0 1;
        -1 0 1];
dy = dx';

% Image derivatives
Ix = conv2(double(im), dx, 'same'); 
Iy = conv2(double(im), dy, 'same');
sigma=2;

% Generate Gaussian filter of size 9x9 and std. dev. sigma. 
g = fspecial('gaussian',9, sigma);

% Smoothed squared image derivatives 
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

figure(1);
subplot(1,3,1), imshow(im);
subplot(1,3,2), imshow(Iy);
subplot(1,3,3), imshow(Iy);

[M, N] = size(im);
MN = M*N;

% 
im = im(:);
Ix2 = Ix2(:);
Iy2 = Iy2(:);
Ixy = Ixy(:);

figure(2);

%% Part 1
% Modify the code above to compute a matrix E which contains 
% for every point the value of the smaller eigenvalue of M. 
E = zeros(MN,1);
tic
for i=1:length(im)
    E(i) = min( eig( [Ix2(i) Ixy(i); Ixy(i) Iy2(i)] ) );
end
toc

% Once computed, display this matrix by means of:
subplot(2,2,1), imshow( mat2gray( reshape(E,M,N) ) )

%% Part 2
% Modify the code above to compute a matrix R which contains 
% for every point the result of Equation (3). What is the difference 
% with respect to E?
% \$R = det(M)- k\dottr(M)^2\$

k = 0.04;

tic
detM = Ix2 .* Iy2 - 2.*Ixy.*Ixy;
trcM2 = Ix2 + Iy2;
trcM2 = trcM2 .* trcM2;

R = detM - k.*trcM2;
toc

% Once computed, display this matrix by means of:
subplot(2,2,2), imshow( mat2gray( reshape(R,M,N) ) )

%% Part 3
% Select for E and R the 81 most salient points. Is this the result 
% you expected?
% Hint for part 3: Build a structure named features with 2 fields
% to store the x coordinate (p_x) and the y coordinate (p_y) of the
% 81 points with the highest the cornerness value given by E or R. 
% Then, display the corners by means of:
MAXPTS = 81;
[~, ixE] = sort(E);

maxE = zeros(MN,1);
maxE( ixE(end-MAXPTS+1:end) ) = 1;


subplot(2,2,3), imshow(im_orig); hold on;
plot( ixE(end-MAXPTS+1:end) , 'r+')

[~, ixR] = sort(R);

maxR = zeros(MN,1);
maxR( ixR(end-MAXPTS+1:end) ) = 1;
subplot(2,2,4), imshow( mat2gray( reshape(maxR,M,N) ) )

%% Part 4
% Build a function to carry out non-maximal suppression for E and R. 
% Again, the 81 most salient points using a non-maximal suppression 
% of 11×11 pixels. Is this result better than that ofnl Part 3?
funmin = @(x) min(x(:))




