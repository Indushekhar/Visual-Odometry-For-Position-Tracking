clc 
clear all
close all
%% Video 
Vid = VideoWriter('VisualOdometry_Comparison_output','MPEG-4');
Vid.FrameRate = 20;
open(Vid);
%% Get the Camera Parameters 
image_dir = 'input\Oxford_dataset\stereo\centre';
[fx, fy, cx, cy, ~, LUT] = ReadCameraModel('input\Oxford_dataset\stereo\centre','input\Oxford_dataset\model');
K = [fx 0 cx;0 fy cy; 0 0 1];

cameraParams = cameraParameters('IntrinsicMatrix',K');
%% Getting the first frame
D = 'input/Oxford_dataset/stereo/centre';
S = dir(fullfile(D,'*.png')); 
file1 = fullfile(D,S(1).name);
I1 = imread(file1);
I1_col = demosaic(I1,'gbrg');
I1_col = UndistortImage(I1_col, LUT);
I1_col= imgaussfilt(I1_col,1.2);
I1_gray = rgb2gray(I1_col);
points1 = detectSURFFeatures(I1_gray);

%% Extracting the feature from frame1
[features1, points1] = extractFeatures(I1_gray,points1, 'Upright', true);

%% Initialization of camera pose
R_1 = eye(3);
t_1 = [0; 0 ;0];
pos_cam=[0,0];
pos_cam_matlab=[0,0];
R_1_mf = eye(3);
t_1_mf = [0;0;0];

%% Main Loop
for i = 15:3850%numberOfImageFiles
    file2 = fullfile(D,S(i).name);
    I2 = imread(file2);
    I2_col = demosaic(I2,'gbrg');
    I2_col = UndistortImage(I2_col, LUT);
    I2_col= imgaussfilt(I2_col,1.2);
    I2_gray = rgb2gray(I2_col);
    points2 = detectSURFFeatures(I2_gray);
    [features2, points2] = extractFeatures(I2_gray,points2, 'Upright', true);
    indexPairs = matchFeatures(features1,features2);
    matchedPoints1 = points1(indexPairs(:,1),:);
    matchedPoints2 = points2(indexPairs(:,2),:);

    %% Compute the Fundamental Matrix using RANSAC
    F = FundamentalMatrixRANSAC(matchedPoints1, matchedPoints2);
    
    %% Calculating the essential matrix
   E = K'* F * K;
   [U,~,V] = svd(E);
   E = U * [1 0 0;0 1 0;0 0 0] * V';
   E = E / norm(E);
   
    %% Get the Pose from Essential Matrix
    [R4, t4] = get_pose(E);
    
    %% Getting the correct pose out of four poses
    [R, t] = correct_pose(R4,t4);
    
    %% Updating poses
    t_1 = t_1 + R_1*t;
    R_1 =  R_1*R;
   
   pos_cam=[pos_cam;[t_1(1), t_1(3)]];
    
  %% Calculation using MATLAB function
    
[F_matlab,inlierIndex] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','RANSAC','NumTrials',2000,'DistanceThreshold',1e-3);
     m1X = matchedPoints1.Location(:,1);
    m1Y = matchedPoints1.Location(:,2);
    inliers1 = [m1X(inlierIndex) m1Y(inlierIndex)];
    
    m2X = matchedPoints2.Location(:,1);
    m2Y = matchedPoints2.Location(:,2);
    inliers2 = [m2X(inlierIndex) m2Y(inlierIndex)];
    
   [R_mf,t_mf] = relativeCameraPose(F_matlab,cameraParams,inliers1,inliers2);
    t_1_mf = t_1_mf + R_1_mf * t_mf';
    R_1_mf = R_1_mf* R_mf';
   
    pos_cam_matlab=[pos_cam_matlab;[t_1_mf(1), t_1_mf(3)]];
    
%% comment out the line 89-92 to see the plot only( without frames) in proper size
figure(1)
subplot(2,1,1)
imshow(I2_col)
subplot(2,1,2)
 plot(-pos_cam(:,1),pos_cam(:,2),'g',pos_cam_matlab(:,1), pos_cam_matlab(:,2),'r','Linewidth',1.5)
 xlabel('X')
 ylabel('Z')
 legend('Location','northoutside')
legend('cameraPostion','cameraPostion(MATLAB In-built Function)')

    %% Change for next iteration
    features1 = features2;
    points1 = points2;
    frame = getframe(gcf);
    writeVideo(Vid,frame);
    frame_No=i
   
end
close(Vid)
