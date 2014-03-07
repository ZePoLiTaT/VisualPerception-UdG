clc; clear; close all;

%% Step 1. 
% Define the intrinsic parameters and extrinsic 
% parameters with the following values:
au=557.0943; av=712.9824; u0=326.3819; v0=298.6679;
f=80;
Tx=100; Ty=0; Tz=1500;
Phix=0.8*pi/2; Phiy=-1.8*pi/2; Phix1=pi/5;

%The intrinsec matrix is given by:
intrinsec = [au  0 u0 0;
              0 av v0 0;
              0  0  1 0];


%The extrinsec matrix is given by the rotation and translation
%of the points in the world coordinate system expressed with respect
%to the camera reference system:

%First I define the rotation matrices for X and Y (generator functions)
Rx = @(phi) [1 0 0;  0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = @(phi) [cos(phi) 0 sin(phi); 0 1 0; -sin(phi) 0 cos(phi)];

%Then I calculate the Euler rotation matrix of the form XYX
Reuler = Rx(Phix)*Ry(Phiy)*Rx(Phix1);

T = [Tx Ty Tz]';

%Now that we have the rotation matrix and translation vector we
%define the extrinsec matrix
extrinsec = [ Reuler        T;
              zeros(1,3)    1];
          

%% Step 2. 
% Get the intrinsic and extrinsic transformation matrices
disp('intrinsec: '); intrinsec
disp('extrinsec: '); extrinsec
disp('calibration matrix: '); 
calib_matrix = intrinsec * extrinsec;
          

%% Step 3.
% Define a set of 3D points in the rang [-480:480;-480:480;-480:480]. 
% Note the points should be non-linear and non-coplanar. 
% At least you need to define a set of 6 points. 
% It?s not necessary to demonstrate mathematically the 
% non-linearity/non-coplanarity, just define 6 points randomly 
% in the 3D space.

LIM = 480;
NUMPTS = 50; %10; %6;

%TODO: Remove this (only for testing)
rng(2);
%generate six 3D random points
xyz = rand(3, NUMPTS)*(LIM*2) - LIM;

%since we are dealing with homogeneous coordinates, we add an extra 1
xyz = [xyz; ones(1, NUMPTS)];

S = rand(size(xyz,2), 1);
%figure(2); scatter3( xyz(1,:),  xyz(2,:), xyz(3,:), [], S);


%% Step 4. 
% Compute the projection on the image plane by using the camera 
% transformation matrix. Do not remove the subpixel precision of 
% the obtained points.

xy_unormalized = calib_matrix * xyz;


%% Step 5.
% Open a window in matlab which will be used as the image plane and 
% draw the 2D points. Are the points well spread in the image plane?
% Will the distribution of points in the image affect the accuracy 
% in the computation?

%In order to plot the obtained homogeneous coordinates we first
%need to normalize them.
xy = bsxfun(@rdivide, xy_unormalized, xy_unormalized(3,:));

figure(1); scatter( xy(1,:),  xy(2,:), 'c.');
rectangle('Position',[0, 0, 640, 480],'LineStyle','--');

%% Step 6.
% By using the points of Step 3 and their projection obtained in Step 5, 
% compute the 3x4 transformation matrix by using the method of Hall.

A = calibration_hall(xy, xyz)


%% Step 7.
% Compare the matrix obtained in Step 6 to the one defined in step 2.
calib_matrix_normalized = calib_matrix / calib_matrix(3,4)
error = A(:) - calib_matrix_normalized(:);
disp(sprintf('Error of Hall method: %E', norm(error,2)));


%% Step 8.
% Add some Gaussian noise to all the 2D points producing discrepancies
% between the range [-1,+1] pixels for the 95% of points. 
% Again repeat step 6 with the noisy 2D points and the ones defined in step 3. 
% Compare the obtained matrix to the one you got in step 6 with the
% non- noisy points. 

%Add gaussian noise with 0 mean and std = \$\frac{1-(-1)}{4}=\frac{1}{2}\$
xy_noisy = xy(1:2,:) + randn(2,NUMPTS)*0.5;
Anoisy = calibration_hall(xy_noisy, xyz)
hold on;
figure(1); scatter( xy_noisy(1,:),  xy_noisy(2,:), 'mo');

%Error with the noisy measurements
error = Anoisy(:) - A(:);
disp(sprintf('Error of Hall method with noisy measurements: %E', norm(error,2)));


%% Now compute the 2D points with the obtained matrix
% and compare them to those obtained in step 4 (you can check accuracy 
% computing the discrepancy between points)?
xy_projection = Anoisy * xyz;
%In order to plot the obtained homogeneous coordinates we first
%need to normalize them.
xy_projection_norm = bsxfun(@rdivide, xy_projection, xy_projection(3,:));
hold on;
figure(1); scatter( xy_projection_norm(1,:),  xy_projection_norm(2,:), 'bx');

