%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - PIV project (Part 2)                                %
%                                                                        %
%  The function just receives the images as input and must compute both  %
%  the transformations(R1,T1, R2,T2 in the figure)  and the object       %
%  trajectories.                                                         %
%                                                                        %
%  Autores: nº 78486, Luís Rei                                           %
%           nº 78675, Gonçalo Duarte                                     %
%           nº 78761, João Girão                                         %
%                                                                        %
%  Data: 22/12/2017                                                      %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [objects, cam1toW, cam2toW ] = track3D_part2(imgseq1, imgseq2, cam_params)

% --------------------------------------------------------------------
%                           Variables' declarations and instantiations
% --------------------------------------------------------------------

% Define global variables
global depth_threshold
global label_threshold
global seq_threshold
global gradient_threshold
global peak_thresh
global alpha
global beta
global gamma
global gridStep
global maxPointDistance

% Define thresholds
seq_threshold = 30; % maximum of 30 images to perform background removal
depth_threshold = 50; % Able to deal with small objects
label_threshold = 500; % Minimum object has 500 pixels
gradient_threshold = 150; % Assumed object is never > 20cm in depth
peak_thresh = 0.03; % SIFT peak threshold parameter
gridStep = 0.005; % Pointcloud downsample grid size parameter
maxPointDistance = 0.200; % Maximum of 20 cm between centroids
alpha = 0.6; % Weight given to the #matches/#features
beta = 0.2;  % Weight given to the color histogram difference
gamma = 0.2; % Weight given to the object eccentricity match
n = 2000; % Number of RANSAC iterations
ransac_threshold = 0.4; % Inlier threshold for RANSAC

% Initialize camera parameters
Kdepth = cam_params.Kdepth; % 3x3 matrix
Krgb = cam_params.Krgb;
R = cam_params.R; % Rotation matrix from depth to RGB (extrinsic params)
T = cam_params.T; % Translation from depth to RGB

% Camera 1 is the world reference
cam1toW.R = [1 0 0; 0 1 0; 0 0 1];
cam1toW.T = [0; 0; 0];


% --------------------------------------------------------------------
%                                                           Background
% --------------------------------------------------------------------

[bg1, bgd1] = get_bg(imgseq1); % camera 1
[bg2, bgd2] = get_bg(imgseq2); % camera 2

% --------------------------------------------------------------------
%                                                  Combine depth + rgb
% --------------------------------------------------------------------
% camera 1
xyz1 = get_xyzasus(bgd1(:),[length(bgd1(:,1)) length(bgd1(1,:))],1:length(bgd1(1,:))*length(bgd1(:,1)),Kdepth,1,0);
rgbd1 = get_rgbd(xyz1, bg1, R, T, Krgb);
% camera 2
xyz2 = get_xyzasus(bgd2(:),[length(bgd2(:,1)) length(bgd2(1,:))],1:length(bgd2(1,:))*length(bgd2(:,1)),Kdepth,1,0);
rgbd2 = get_rgbd(xyz2, bg2, R, T, Krgb);

% --------------------------------------------------------------------
%                            Define cam2 transformation to world frame
% --------------------------------------------------------------------

[R2, T2] = get_world_transform(n, bg1, bg2, bgd1, bgd2, xyz1, xyz2, ...
    rgbd1, rgbd2, ransac_threshold);

cam2toW.R = R2;
cam2toW.T = T2;

% --------------------------------------------------------------------
%                                        Object detection and tracking
% --------------------------------------------------------------------

keepvars = {'imgseq1', 'imgseq2', 'cam_params', 'cam1toW', 'cam2toW',...
    'bgd1', 'bgd2'};
clearvars('-except', keepvars{:});

% Similar to track3D_part1 but already receives previously calculated
% backgrounds for each camera
objects = detect_and_track(imgseq1, imgseq2, cam_params, cam1toW, cam2toW, bgd1, bgd2);

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        AUXILIARY FUNCTIONS                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - detect_and_track                                    %
%                                                                        %
%  The transformations (R1,T1,R2,T2) are known, the function             %
%  receives the images, the transformations, the cameras' backgrounds    %
%  and as inputs and returns the 8 points describing the time            %
%  trajectories of the enclosing box of the objects in world coordinates %                                      %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function objects = detect_and_track( imgseq1, imgseq2, cam_params, cam1toW, cam2toW, bgd1, bgd2 )

% --------------------------------------------------------------------
%                           Variables' declarations and instantiations
% --------------------------------------------------------------------
% Define global variables
global depth_threshold
global label_threshold
global seq_threshold
global gradient_threshold
global peak_thresh
global alpha
global beta
global gamma
global gridStep
global maxPointDistance

% Initializes 'objects' structure
objects = [];

% Initialize camera parameters
Kdepth = cam_params.Kdepth; % 3x3 matrix
Krgb = cam_params.Krgb;
R = cam_params.R; % Rotation matrix from depth to RGB (extrinsic params)
T = cam_params.T; % Translation from depth to RGB

% Initialize world frame parameters
Rw1 = cam1toW.R; % 3x3  Rotation matrix between 1st camera and world frame
Tw1 = cam1toW.T; % 3x1 Translation vector from camera 1 to the world frame
Rw2 = cam2toW.R; % 3x3  Rotation matrix between 2nd camera and world frame
Tw2 = cam2toW.T; % 3x1 Translation vector from camera 2 to the world frame

% --------------------------------------------------------------------
%                                                Object tracking cycle
% --------------------------------------------------------------------
for i = 1:numel(imgseq1)
    
    % Load Depth and RGB from specified index array structure
    im1 = imread(imgseq1(i).rgb);
    load(imgseq1(i).depth, 'depth_array'); depth1 = depth_array;
    
    im2 = imread(imgseq2(i).rgb);
    load(imgseq2(i).depth, 'depth_array'); depth2 = depth_array;
    clear depth_array;
    
    % --------------------------------------------------------------------
    %                                                           Foreground
    % --------------------------------------------------------------------
    % Camera 1
    [fg1_gray, fg1_label] = get_fg(bgd1, im1, depth1);
    % Camera 2
    [fg2_gray, fg2_label] = get_fg(bgd2, im2, depth2);
    
    % --------------------------------------------------------------------
    %                                          Combine depth + rgb cameras
    % --------------------------------------------------------------------
    if max(fg1_label(:))~=0
        % camera 1
        xyz1 = get_xyzasus(depth1(:),[length(depth1(:,1)) length(depth1(1,:))],1:length(depth1(1,:))*length(depth1(:,1)),Kdepth,1,0);
        rgbd1 = get_rgbd(xyz1, im1, R, T, Krgb);
    end
    if max(fg2_label(:))~=0
        % camera 2
        xyz2 = get_xyzasus(depth2(:),[length(depth2(:,1)) length(depth2(1,:))],1:length(depth2(1,:))*length(depth2(:,1)),Kdepth,1,0);
        rgbd2 = get_rgbd(xyz2, im2, R, T, Krgb);
    end
    
    
    %%
    % Compute pointcloud in 3D to assess objects' similarities in both
    % cameras. Reduce pointcloud size to easen the computational burden.
    
    % List of explored object regions in current camera frame.
    % Assures all objects are scanned and checked their "box" set.
    explored1 = zeros(max(fg1_label(:)),1);
    explored2 = zeros(max(fg2_label(:)),1);
    
    % Auxiliary structures. objs_aux0 for objects in both camera, and aux1
    % and 2 for objects solely in camera 1 and 2, respectively.
    objs_aux0 = []; objs_aux1 = []; objs_aux2 = [];
    count = 0; % Auxiliary variable to index objects
    
    % --------------------------------------------------------------------
    %                 Compare objects identity through centroid comparison
    % --------------------------------------------------------------------
    for j = 1:max(fg1_label(:))
        % For each label at each frame in each camera verify object closeness
        
        % Reset auxiliary variables
        index1 = 0;
        new_xyz1 = [];
        
        % Compute object's pointcloud based on label j
        [row,col] = find(fg1_label == j);
        temp_xyz1 = reshape(xyz1,length(depth1(:,1)),length(depth1(1,:)),3);
        for l = 1:length(row)
            if all(xyz1((col(l)-1)*length(depth1(:,1)) + row(l), :) == [0 0 0])
            else
                index1 = index1 + 1;
                new_xyz1(index1, :) = temp_xyz1(row(l), col(l),:);
            end
        end
        
        % Build translation vector
        TW1 = cat(2, ones(length(new_xyz1),1).*Tw1(1,:), ones(length(new_xyz1),1).*Tw1(2,:), ones(length(new_xyz1),1).*Tw1(3,:));
        
        % Translate xyz from camera 1 to World Frame
        new_xyz1 = (Rw1*new_xyz1' + TW1')';
        
        % Project objects in 3D. Downsampled pointcloud
        p1 =  pcdownsample(pointCloud(new_xyz1),'gridAverage',gridStep);
        
        % Object centroid in camera 1
        centroid1(1) = mean(p1.Location(:,1));
        centroid1(2) = mean(p1.Location(:,2));
        centroid1(3) = mean(p1.Location(:,3));
        
        for k = 1:max(fg2_label(:))
            % Same as before but for camera 2
            
            index2 = 0;
            new_xyz2 = [];
            
            % Compute object's pointcloud based on label k
            [row,col] = find(fg2_label == k);
            temp_xyz2 = reshape(xyz2,length(depth2(:,1)),length(depth2(1,:)),3);
            for l = 1:length(row)
                if all(xyz2((col(l)-1)*length(depth2(:,1)) + row(l), :) == [0 0 0])
                else
                    index2 = index2 + 1;
                    new_xyz2(index2, :) = temp_xyz2(row(l), col(l),:);
                end
            end
            
            % Build translation vectors
            TW2 = cat(2, ones(length(new_xyz2),1).*Tw2(1,:), ones(length(new_xyz2),1).*Tw2(2,:), ones(length(new_xyz2),1).*Tw2(3,:));
            
            % Translate xyz from camera 2 to World Frame
            new_xyz2 = (Rw2*new_xyz2' + TW2')';
            
            % Project objects in 3D. Downsampled pointcloud
            p2 =  pcdownsample(pointCloud(new_xyz2),'gridAverage',gridStep);
            
            % Object centroid in camera 2
            centroid2(1) = mean(p2.Location(:,1));
            centroid2(2) = mean(p2.Location(:,2));
            centroid2(3) = mean(p2.Location(:,3));
            
            % Check pointcloud centroids' distance
            if norm([centroid1(1)-centroid2(1), centroid1(2)-centroid2(2), centroid1(3)-centroid2(3)]) < maxPointDistance
                count = count+1;
                explored1(j) = 1;
                explored2(k) = 1;
                
                if max(p1.XLimits) > max(p2.XLimits)
                    max_coords(1) = max(p2.XLimits)+(max(p1.XLimits)-max(p2.XLimits))/2;
                    max_coords(2) = max(p2.YLimits)+(max(p1.YLimits)-max(p2.YLimits))/2;
                    max_coords(3) = max(p2.ZLimits)+(max(p1.ZLimits)-max(p2.ZLimits))/2;
                else
                    max_coords(1) = max(p1.XLimits)+(max(p2.XLimits)-max(p1.XLimits))/2;
                    max_coords(2) = max(p1.YLimits)+(max(p2.YLimits)-max(p1.YLimits))/2;
                    max_coords(3) = max(p1.ZLimits)+(max(p2.ZLimits)-max(p1.ZLimits))/2;
                end
                
                if min(p1.XLimits) < min(p2.XLimits)
                    min_coords(1) = min(p2.XLimits)+(min(p1.XLimits)-min(p2.XLimits))/2;
                    min_coords(2) = min(p2.YLimits)+(min(p1.YLimits)-min(p2.YLimits))/2;
                    min_coords(3) = min(p2.ZLimits)+(min(p1.ZLimits)-min(p2.ZLimits))/2;
                else
                    min_coords(1) = min(p1.XLimits)+(min(p2.XLimits)-min(p1.XLimits))/2;
                    min_coords(2) = min(p1.YLimits)+(min(p2.YLimits)-min(p1.YLimits))/2;
                    min_coords(3) = min(p1.ZLimits)+(min(p2.ZLimits)-min(p1.ZLimits))/2;
                end
                found = 0;
                
                if found == 0
                    objs_aux0(count).X = [min_coords(1), min_coords(1), max_coords(1), max_coords(1), min_coords(1), min_coords(1), max_coords(1), max_coords(1)]; %#ok<*AGROW> %%#ok<*AGROW>
                    objs_aux0(count).Y = [min_coords(2), max_coords(2), max_coords(2), min_coords(2), min_coords(2), max_coords(2), max_coords(2), min_coords(2)];
                    objs_aux0(count).Z = [max_coords(3), max_coords(3), max_coords(3), max_coords(3), min_coords(3), min_coords(3), min_coords(3), min_coords(3)];
                    objs_aux0(count).frames_tracked = i;
                    objs_aux0(count).label1 = j;
                    objs_aux0(count).label2 = k;
                    objs_aux0(count).ind = count;
                    break
                end
            end
        end
    end
    
    clear new_xyz1; clear new_xyz2; clear temp_xyz1; clear temp_xyz2;
    clear new_rgbd1; clear new_rgbd2;
    clear TW1; clear TW2;
    
    % Unexplored objects are individual to each camera. Set them as
    % different objects (two boxes instead of one).
    if max(fg1_label(:)) ~=0
        [objs_aux1, count] = set_explored(count, i,1,explored1,fg1_label,xyz1, depth1, Rw1, Tw1);
    end
    if max(fg2_label(:)) ~=0
        [objs_aux2, ~] = set_explored(count, i,2,explored2,fg2_label,xyz2, depth2, Rw2, Tw2);
    end
    
    
    %%
    % Calculates object movement through frames
    
    % --------------------------------------------------------------------
    %                              SIFT matches and output structure build
    % --------------------------------------------------------------------
    if (i > 1 && (max(fg1_label(:)) ~= 0 && max(fg1_label_ant(:)) ~= 0) )
        % Retrieve features match matrix.
        matches_matrix1 = get_match_matrix(fg1_label, fg1_label_ant, fg1_gray, fg1_gray_ant);
        
        % Retrieve color similarity matrix. Retrieve eccentricity matching between objects
        [color_matrix1, ecc_matrix1] = get_info_objects(fg1_label_ant,...
            fg1_label, xyz1, depth1, rgbd1);
        
        % Compute stochastic matrix to access object tracking through frames
        objs_match1 = get_object_track(alpha, beta, gamma,...
            matches_matrix1, color_matrix1, ecc_matrix1);
        
        [objects,objs_aux0,objs_aux1] = get_association_frame_cam1(objects, objs_match1,...
            objs_aux0_ant,objs_aux1_ant,objs_aux2_ant, objs_aux0,objs_aux1);
    end
    
    % Same for camera 2
    if (i > 1 && (max(fg2_label(:)) ~= 0 && max(fg2_label_ant(:)) ~= 0) )
        % Retrieve features match matrix.
        matches_matrix2 = get_match_matrix(fg2_label, fg2_label_ant, fg2_gray, fg2_gray_ant);
        
        % Compute stochastic matrix to access object tracking through frames
        [color_matrix2, ecc_matrix2] = get_info_objects(fg2_label_ant,...
            fg2_label, xyz2, depth2, rgbd2);
        
        % Retrieve color similarity matrix. Retrieve eccentricity matching between objects
        objs_match2 = get_object_track(alpha, beta, gamma,...
            matches_matrix2, color_matrix2, ecc_matrix2);
        
        [objects,objs_aux0,objs_aux1,objs_aux2] = get_association_frame_cam2(objects, objs_match2,...
            objs_aux0_ant,objs_aux1_ant,objs_aux2_ant, objs_aux0,objs_aux1,objs_aux2);
    end
    
    % Construct initial objects' structure
    if i > 1
        if ~isempty(objs_aux0) && max(fg1_label_ant(:)) == 0 && max(fg2_label_ant(:)) == 0
            last = length(objects) + 1;
            for k = 1:length(objs_aux0)
                objs_aux0(k).ind = last;
                last = last + 1;
            end
            objects = cat(2, objects, objs_aux0);
        end
        if ~isempty(objs_aux1) && max(fg1_label_ant(:)) == 0
            last = length(objects) + 1;
            for k = 1:length(objs_aux1)
                objs_aux1(k).ind = last;
                last = last + 1;
            end
            objects = cat(2, objects, objs_aux1);
        end
        if ~isempty(objs_aux2) && max(fg2_label_ant(:)) == 0
            last = length(objects) + 1;
            for k = 1:length(objs_aux2)
                objs_aux2(k).ind = last;
                last = last + 1;
            end
            objects = cat(2, objects, objs_aux2);
        end
    else
        if ~isempty(objs_aux0)
            objects = cat(2, objects, objs_aux0);
        end
        if ~isempty(objs_aux1)
            objects = cat(2, objects, objs_aux1);
        end
        if ~isempty(objs_aux2)
            objects = cat(2, objects, objs_aux2);
        end
    end
    
    % --------------------------------------------------------------------
    %                                                    Update foreground
    % --------------------------------------------------------------------
    fg1_gray_ant = fg1_gray; fg2_gray_ant = fg2_gray;       % update gray
    fg1_label_ant = fg1_label; fg2_label_ant = fg2_label;   % update label
    objs_aux0_ant = objs_aux0; % update objects seen in last frame
    objs_aux1_ant = objs_aux1;
    objs_aux2_ant = objs_aux2;
    
end

% Trim final objects structure. Ignore objects seen in only one frame.
objects = trim_objects(objects);

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Retrieve background from a series of images         %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [bgim, bgimd] = get_bg(imgseq)
% imgseq - sequence of images up to seq_threshold

global seq_threshold

% Subsample sequence of images if neccessary
if (length(imgseq) > seq_threshold)
    new_imgseq = imgseq(1:seq_threshold);
else
    new_imgseq = imgseq;
end

% Pre-allocate memory
load(new_imgseq(1).depth);
ims = zeros(length(depth_array(:,1))*length(depth_array(1,:)),3,numel(new_imgseq));
imsd = zeros(length(depth_array(:,1))*length(depth_array(1,:)),numel(new_imgseq));

% Construct array with rgb and depth info
for i = 1:numel(new_imgseq)
    im = imread(new_imgseq(i).rgb);
    im = reshape(im, length(im(:,1))*length(im(1,:))/3,3);
    load(new_imgseq(i).depth);
    ims(:,:,i) = im;
    imsd(:,i) = depth_array(:);
end

% Calculate median
medim = median(double(ims),3);
meddep = median(double(imsd),2);

% Reshape into matrix with desirable size
bgim = (uint8(reshape(medim,[length(depth_array(:,1)) length(depth_array(1,:)) 3])));
bgimd = reshape(meddep,[length(depth_array(:,1)) length(depth_array(1,:))]);

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Retrieve foreground from an image                   %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [fg_gray, fg_depth_label] = get_fg(bgd, im_rgb, im_depth)
% im_rgb - rgb image matrix (:,:,3)
% im_depth - depth_array matrix
% bg - previously calculated image background
% bg - depth background

% Define thresholds
global depth_threshold
global label_threshold
global gradient_threshold

% make grayscale
if size(im_rgb,3) > 1,
    im_gray = rgb2gray(im_rgb) ;
else
    im_gray = im_rgb ;
end

fg_depth = abs(double(im_depth) - double(bgd))*0.001; % Translate depth foreground to meters
fg_gray = uint8(zeros(length(bgd(:,1)),length(bgd(1,:)))); % initialize fg matrix

% Deal with depth foreground
fg_depth_mask = bwareafilt(abs(double(im_depth) - double(bgd)) > depth_threshold,...
    [label_threshold length(bgd(:,1))*length(bgd(1,:))]);

% Gradient to filter depth discrepancies
[fx, fy] = gradient(double(im_depth));
gradient_mask = (fx.*fx + fy.*fy) < (gradient_threshold)^2;

% Filter foreground through masks
fg_depth_mask(im_depth(:) == 0) = 0; % Ignore invalid values
fg_depth_mask(gradient_mask(:) == 0) = 0; % Ignore depth discrepancies
fg_depth_label = bwlabel(bwareafilt(imopen(imfill(bwareafilt(fg_depth_mask,...
    [label_threshold length(bgd(:,1))*length(bgd(1,:))]),'holes'),strel('disk',4)),...
    [label_threshold length(bgd(:,1))*length(bgd(1,:))]));

% Construct final foreground containing only labelled objects
for m = 1:length(bgd(:,1))
    for n = 1:length(bgd(1,:))
        if fg_depth_label(m, n) > 0
            fg_gray(m, n) = im_gray(m, n);
        end
    end
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Index different objects that were not seen in both  %
%  cameras                                                               %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [objects, count] = set_explored(count, frame,cam_number, explored, label, xyz, depth, Rw, Tw)
% Returns objects' structure with fields specified as in the project guide

objects = []; % Initialize objects' structure
o = 0;
% For each unexplored object, add a line in the structure
for e = 1:length(explored)
    if explored(e) == 0
        index = 0;
        
        % Resize object
        [r,c] = find(label == e);
        temp_xyz = reshape(xyz,length(depth(:,1)),length(depth(1,:)),3);
        for l = 1:length(r)
            if all(xyz((c(l)-1)*length(depth(:,1)) + r(l), :) == [0 0 0])
            else
                index = index + 1;
                new_xyz(index, :) = temp_xyz(r(l), c(l),:);
            end
        end
        
        % Build translation vector
        TW = cat(2, ones(length(new_xyz),1).*Tw(1,:), ones(length(new_xyz),1).*Tw(2,:), ones(length(new_xyz),1).*Tw(3,:));
        
        % Translate xyz from camera 1 to World Frame
        new_xyz = (Rw*new_xyz' + TW')';
        
        % Get XYZ coordinates for the box
        max_coords = max(new_xyz, [], 1);
        min_coords = min(new_xyz, [], 1);
        o = o + 1;
        count = count+1;
        objects(o).X = [min_coords(1), min_coords(1), max_coords(1), max_coords(1), min_coords(1), min_coords(1), max_coords(1), max_coords(1)];
        objects(o).Y = [min_coords(2), max_coords(2), max_coords(2), min_coords(2), min_coords(2), max_coords(2), max_coords(2), min_coords(2)];
        objects(o).Z = [max_coords(3), max_coords(3), max_coords(3), max_coords(3), min_coords(3), min_coords(3), min_coords(3), min_coords(3)];
        objects(o).frames_tracked = frame;
        objects(o).label1 = 0;
        objects(o).label2 = 0;
        objects(o).ind = count;
        if cam_number == 1
            objects(o).label1 = e;
        else
            objects(o).label2 = e;
        end
    end
end

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Gather information about eccentricity and color of  %
%    the objects in two consecutive frames                               %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [color_matrix, ecc_matrix] = get_info_objects( label_ant, label, xyz, depth, rgbd)

color_matrix = zeros(max(label_ant(:))+1, max(label(:))+1);
ecc_matrix = zeros(max(label_ant(:))+1, max(label(:))+1);

for j = 1:max(label_ant(:))
    index1 = 0;
    new_rgbd_ant = [];
    
    new_xyz_ant = [];
    
    [r,c] = find(label_ant == j);
    temp_xyz = reshape(xyz,length(depth(:,1)),length(depth(1,:)),3);
    for l = 1:length(r)
        if all(xyz((c(l)-1)*length(depth(:,1)) + r(l), :) == [0 0 0])
        else
            index1 = index1 + 1;
            new_rgbd_ant(index1,:) = rgbd(r(l), c(l),:);
            new_xyz_ant(index1, :) = temp_xyz(r(l), c(l),:);
        end
    end
    
    for k = 1:max(label(:))
        index2 = 0;
        new_rgbd = [];
        new_xyz = [];
        
        [r,c] = find(label == k);
        
        temp_xyz = reshape(xyz,length(depth(:,1)),length(depth(1,:)),3);
        for l = 1:length(r)
            if all(xyz((c(l)-1)*length(depth(:,1)) + r(l), :) == [0 0 0])
            else
                index2 = index2 + 1;
                new_rgbd(index2,:) = rgbd(r(l), c(l),:);
                new_xyz(index2, :) = temp_xyz(r(l), c(l),:);
            end
        end
        
        color_matrix(j,k) = get_color_histogram(new_rgbd_ant, new_rgbd);
    end
    ecc_matrix = get_eccentricity(label_ant, label);
end

for ind1 = 1:size(color_matrix,1)-1
    color_matrix(ind1,:) = color_matrix(ind1,:)/sum(color_matrix(ind1,:));
end

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Compare color histograms of objects                 %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function coef = get_color_histogram(rgbd_ant, rgbd)
% Transforms color histogram differences into qualitative coefficient to be
% used in the feature matching problem

% Color normalization to lower luminosity differences impact
hsvImage = rgb2hsv(rgbd_ant./255);
hsvImage(hsvImage(:) == 1) = 254/255; % Correct black histogram bin going over to white
hueImage = hsvImage(:,1);
c1 = imhist(hueImage);

hsvImage = rgb2hsv(rgbd./255);
hsvImage(hsvImage(:) == 1) = 254/255;
hueImage = hsvImage(:,1);
c2 = imhist(hueImage);

% Negative exponential penalizes big differences.
% Biggest value (1) when the color histograms are the same.
c = abs(c1-c2)/(0.5*(sum(c1) + sum(c2)));
aux = sum(c);
coef = exp(-aux);

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Retrieve object eccentricity                        %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ecc = get_eccentricity( label_ant, label)
% Outputs eccentricity parameters for object comparison.
aux = regionprops(label_ant,'eccentricity');
s1 = zeros(size(aux,1),1);
for ind_k = 1:size(aux,1)
    s1(ind_k,1) = aux(ind_k).Eccentricity;
end
aux = regionprops(label,'eccentricity');

s2 = zeros(size(aux,1),1);
for ind_k = 1:size(aux,1)
    s2(ind_k,1) = aux(ind_k).Eccentricity;
end

% Build matrix
if length(s1) == 1 && length(s2) == 1
    ecc = [1 0; 0 0];
else
    ecc = ones(length(s1)+1, length(s2)+1);
    for ind_i = 1:length(s1)
        for ind_j = 1:length(s2)
            ecc(ind_i, ind_j) = abs(s1(ind_i)-s2(ind_j));
        end
    end
    for ind_i = 1:length(s1)
        ecc(ind_i, 1:end-1) = ecc(ind_i, 1:end-1)./sum(ecc(ind_i, 1:end-1));
    end
    ecc = ones(length(s1)+1, length(s2)+1) - ecc;
    for ind_i = 1:length(s1)
        ecc(ind_i, 1:end-1) = ecc(ind_i, 1:end-1)./sum(ecc(ind_i, 1:end-1));
    end
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Compute matching matrix                             %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function mm_ret = get_match_matrix(labels, labels_prev, Ia, Ib)

global peak_thresh

% Execute sift on the foreground of the grayscaled images
% Obtain frames and descriptors
[fa, da] = vl_sift(im2single(Ia), 'PeakThresh', peak_thresh);
[fb, db] = vl_sift(im2single(Ib), 'PeakThresh', peak_thresh);

% Obtain matches and scores
[matches, scores] = vl_ubcmatch(da, db);
[~, perm] = sort(scores, 'descend');

% Sort by greatest score
matches = matches(:, perm);
scores  = scores(perm) ;

f1 = round(fb(1:2,matches(2,:))); % Coordinates of the features of the 1st image
f2 = round(fa(1:2,matches(1,:))); % Coordinates of the features of the 2nd image

new_mm = zeros(max(labels_prev(:))+1, max(labels(:))+1);

for i = 1:size(matches,2)
    x = labels_prev(f1(2,i), f1(1,i));
    y = labels(f2(2,i), f2(1,i));
    if x ~= 0 && y ~= 0
        new_mm(x, y) = new_mm(x, y) + 1;
    end
end

for i = 1:size(new_mm, 1)-1
    if sum(new_mm(i,:)) ~= 0
        new_mm(i,1:end-1) = new_mm(i,1:end-1)./sum(new_mm(i,1:end-1));
    end
    new_mm(i, end) = 1 - sum(new_mm(i,1:end-1));
end

mm_ret = new_mm;
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Compute object tracking matrix                      %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function track = get_object_track(alpha, beta, gamma, match, color, ecc)

track = zeros(size(match));

% For each region in previous frame, calculate percentage of being certain
% region in the next frame
for i = 1:length(match(:,1))
    track(i,:) = (alpha*match(i,:) + beta*color(i,:) + gamma*ecc(i,:));
    if ~all(track(i,:) == zeros(size(track(i,:))))
        track(i,end) = 1 - sum(track(i,1:end-1));
    end
end

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Transformation from cam2 to cam1                    %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R, T] = get_world_transform(n, bg1, bg2, bgd1, bgd2, xyz1, xyz2, ...
    rgbd1, rgbd2, inlier_threshold)
% n - number of RANSAC iterations
% bg1/bg2 - rgb background images
% bgd1/bgd2 - background depth measurements
% xyz1/xyz2 - 3D depth perception
% rgbd1/rgbd1 - combined depth+rgb
% inlier_threshold - transformation error threshold

% make grayscale
if size(rgbd1,3) > 1,
    rgbd1 = rgb2gray(rgbd1);
end
if size(rgbd2,3) > 1,
    rgbd2 = rgb2gray(rgbd2);
end

% make single
bg1_gray = im2single(rgbd1);
bg2_gray = im2single(rgbd2);

% --------------------------------------------------------------------
%                                                         SIFT matches
% --------------------------------------------------------------------
[fa,da] = vl_sift(bg1_gray);
[fb,db] = vl_sift(bg2_gray);

[matches, ~] = vl_ubcmatch(da,db) ;

f1 = round(fa(1:2,matches(1,:))); % Coordinates of the features of the 1st image
f2 = round(fb(1:2,matches(2,:))); % Coordinates of the features of the 2nd image

% Clean possible invalid matches
c = 1;
for i = 1:size(matches,2)
    if bgd1(f1(2,i), f1(1,i)) ~= 0 && bgd2(f2(2,i), f2(1,i)) ~= 0
        matchedPoints1(:,c) = f1(:,i);
        matchedPoints2(:,c) = f2(:,i);
        c = c + 1;
    end
end

% --------------------------------------------------------------------
%                                                               RANSAC
% --------------------------------------------------------------------
inliers = [];
nb = 4;

xyz1_points = zeros(nb, 3);
xyz2_points = zeros(nb, 3);

for i = 1:n
    ind = randperm(length(matchedPoints1),nb);
    xyz1_points(:, :) = xyz1(sub2ind(size(bg1), matchedPoints1(2,ind), matchedPoints1(1,ind)), :);
    xyz2_points(:, :) = xyz2(sub2ind(size(bg2), matchedPoints2(2,ind), matchedPoints2(1,ind)), :);
    
    [~,~,tr] = procrustes(xyz1_points,xyz2_points,'scaling',false,'reflection',false);
    
    % Apply tranform to all SIFT matches
    xyz1_Allpoints(:, :) = xyz1(sub2ind(size(bg1), matchedPoints1(2,:), matchedPoints1(1,:)), :);
    xyz2_Allpoints(:, :) = xyz2(sub2ind(size(bg2), matchedPoints2(2,:), matchedPoints2(1,:)), :);
    
    % Access transform error on all of the SIFT matches
    ptsTransformed = xyz2_Allpoints*tr.T + ones(length(xyz2_Allpoints),1)*tr.c(1,:);
    distances = (ptsTransformed - xyz1_Allpoints).^2;
    distances = sqrt(distances(:,1)+distances(:,2)+distances(:,3));

	% Update inliers and best tranformation
    if (sum(distances < inlier_threshold) >= length(inliers))
        ind_best = ind;
        inliers = find(distances < inlier_threshold);
    end
end

xyz1_Allpoints(:, :) = xyz1(sub2ind(size(bg1), matchedPoints1(2,:), matchedPoints1(1,:)), :);
xyz2_Allpoints(:, :) = xyz2(sub2ind(size(bg2), matchedPoints2(2,:), matchedPoints2(1,:)), :);
    
[~,~,tr] = procrustes(xyz1_Allpoints(inliers,:),xyz2_Allpoints(inliers,:),'scaling',false,'reflection',false);
if isempty(tr), R = eye(3); T = [0;0;0]; 
else R = tr.T'; T = tr.c(1,:)'; 
end

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Gather information about eccentricity and color of  %
%    the objects in two consecutive frames                               %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [objects,objs_aux0,objs_aux1] = get_association_frame_cam1(objects, objs_match1,...
    objs_aux0_ant,objs_aux1_ant,objs_aux2_ant, objs_aux0,objs_aux1)

free1 = ones(size(objs_match1(1,:))); % Restriction vector

track1 = zeros(length(objs_match1(:,1))-1,1);
% Associate object of previous frame through greedy search
for h = 1:length(objs_match1(:,1))-1
    
    % Run through matrix
    aux = free1.*objs_match1(h,:); % Available matches
    if ~all(aux(:) == 0)
        [~, ind_aux] = max(aux);
    else
        ind_aux = length(objs_match1(1,:));
    end
    % Take match from available ones
    if ind_aux ~= length(objs_match1(1,:))
        track1(h) = ind_aux;
        
        % Restrict that object unless it died
        free1(track1(h)) = 0;
        found1 = 0;
        for ind0 = 1:length(objs_aux0)
            if objs_aux0(ind0).label1 == track1(h)
                for ind01 = 1:length(objs_aux0_ant)
                    if objs_aux0_ant(ind01).label1 == h
                        % Annex new points to final structure
                        for ind02 = 1:length(objects)
                            if objects(ind02).ind == objs_aux0_ant(ind01).ind
                                r = ind02;
                                break
                            end
                        end
                        objects(r).X = cat(1, objects(r).X, objs_aux0(ind0).X);
                        objects(r).Y = cat(1, objects(r).Y, objs_aux0(ind0).Y);
                        objects(r).Z = cat(1, objects(r).Z, objs_aux0(ind0).Z);
                        objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux0(ind0).frames_tracked);
                        objs_aux0(ind0).ind = objects(r).ind;
                        found1 = 1;
                        break
                    end
                end
                if found1 == 0;
                    for ind01 = 1:length(objs_aux1_ant)
                        if objs_aux1_ant(ind01).label1 == h
                            % Annex new points to final structure
                            for ind02 = 1:length(objects)
                                if objects(ind02).ind == objs_aux1_ant(ind01).ind
                                    r = ind02;
                                    break
                                end
                            end
                            objects(r).X = cat(1, objects(r).X, objs_aux0(ind0).X);
                            objects(r).Y = cat(1, objects(r).Y, objs_aux0(ind0).Y);
                            objects(r).Z = cat(1, objects(r).Z, objs_aux0(ind0).Z);
                            objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux0(ind0).frames_tracked);
                            objs_aux0(ind0).ind = objects(r).ind;
                            found1 = 1;
                            break
                        end
                    end
                end
                if found1 == 0;
                    for ind01 = 1:length(objs_aux2_ant)
                        if objs_aux2_ant(ind01).label1 == h
                            % Annex new points to final structure
                            for ind02 = 1:length(objects)
                                if objects(ind02).ind == objs_aux2_ant(ind01).ind
                                    r = ind02;
                                    break
                                end
                            end
                            objects(r).X = cat(1, objects(r).X, objs_aux0(ind0).X);
                            objects(r).Y = cat(1, objects(r).Y, objs_aux0(ind0).Y);
                            objects(r).Z = cat(1, objects(r).Z, objs_aux0(ind0).Z);
                            objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux0(ind0).frames_tracked);
                            objs_aux0(ind0).ind = objects(r).ind;
                            found1 = 1;
                            break
                        end
                    end
                end
                break
            end
        end
        if found1 == 0;
            for ind1 = 1:length(objs_aux1)
                if objs_aux1(ind1).label1 == track1(h)
                    for ind11 = 1:length(objs_aux1_ant)
                        if objs_aux1_ant(ind11).label1 == h
                            % Annex new points to final structure
                            for ind12 = 1:length(objects)
                                if objects(ind12).ind == objs_aux1_ant(ind11).ind
                                    r = ind12;
                                    break
                                end
                            end
                            objects(r).X = cat(1, objects(r).X, objs_aux1(ind1).X);
                            objects(r).Y = cat(1, objects(r).Y, objs_aux1(ind1).Y);
                            objects(r).Z = cat(1, objects(r).Z, objs_aux1(ind1).Z);
                            objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux1(ind1).frames_tracked);
                            objs_aux1(ind1).ind = objects(r).ind;
                            found1 = 1;
                            break
                        end
                    end
                    if found1 == 0;
                        for ind11 = 1:length(objs_aux0_ant)
                            if objs_aux0_ant(ind11).label1 == h
                                % Annex new points to final structure
                                for ind12 = 1:length(objects)
                                    if objects(ind12).ind == objs_aux0_ant(ind11).ind
                                        r = ind12;
                                        break
                                    end
                                end
                                objects(r).X = cat(1, objects(r).X, objs_aux1(ind1).X);
                                objects(r).Y = cat(1, objects(r).Y, objs_aux1(ind1).Y);
                                objects(r).Z = cat(1, objects(r).Z, objs_aux1(ind1).Z);
                                objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux1(ind1).frames_tracked);
                                objs_aux1(ind1).ind = objects(r).ind;
                                found1 = 1;
                                break
                            end
                        end
                    end
                    if found1 == 0;
                        for ind11 = 1:length(objs_aux2_ant)
                            if objs_aux2_ant(ind11).label1 == h
                                % Annex new points to final structure
                                for ind12 = 1:length(objects)
                                    if objects(ind12).ind == objs_aux2_ant(ind11).ind
                                        r = ind12;
                                        break
                                    end
                                end
                                objects(r).X = cat(1, objects(r).X, objs_aux1(ind1).X);
                                objects(r).Y = cat(1, objects(r).Y, objs_aux1(ind1).Y);
                                objects(r).Z = cat(1, objects(r).Z, objs_aux1(ind1).Z);
                                objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux1(ind1).frames_tracked);
                                objs_aux1(ind1).ind = objects(r).ind;
                                break
                            end
                        end
                    end
                    break
                end
            end
        end
    end
end

free1(end) = 0; % Not needed anymore. We will add newborn objects.
c = find(free1(:) ~= 0);

% Annex newborn objects in final structure
for h = 1:length(c)
    last = length(objects) + 1;
    foundit = 0;
    for ind0 = 1:length(objs_aux0)
        if objs_aux0(ind0).label1 == c(h)
            objects(last).X = objs_aux0(ind0).X;
            objects(last).Y = objs_aux0(ind0).Y;
            objects(last).Z = objs_aux0(ind0).Z;
            objects(last).frames_tracked = objs_aux0(ind0).frames_tracked;
            objects(last).label1 = objs_aux0(ind0).label1;
            objects(last).label2 = objs_aux0(ind0).label2;
            objects(last).ind = last;
            objs_aux0(ind0).ind = last;
            foundit = 1;
            break
        end
    end
    if foundit == 0
        for ind1 = 1:length(objs_aux1)
            if objs_aux1(ind1).label1 == c(h)
                objects(last).X = objs_aux1(ind1).X;
                objects(last).Y = objs_aux1(ind1).Y;
                objects(last).Z = objs_aux1(ind1).Z;
                objects(last).frames_tracked = objs_aux1(ind1).frames_tracked;
                objects(last).label1 = objs_aux1(ind1).label1;
                objects(last).label2 = objs_aux1(ind1).label2;
                objects(last).ind = last;
                objs_aux1(ind1).ind = last;
                break
            end
        end
    end
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Gather information about eccentricity and color of  %
%    the objects in two consecutive frames                               %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [objects,objs_aux0,objs_aux1,objs_aux2] = get_association_frame_cam2(objects, objs_match2,...
    objs_aux0_ant,objs_aux1_ant,objs_aux2_ant, objs_aux0,objs_aux1,objs_aux2)

free2 = ones(size(objs_match2(1,:))); % Restriction vector

% Associate object of previous frame through greedy search
for h = 1:length(objs_match2(:,1))-1
    skip = 0;
    % Run through matrix
    track2 = zeros(length(objs_match2(:,1))-1,1);
    aux = free2.*objs_match2(h,:); % Available matches
    if ~all(aux(:) == 0)
        [~, ind_aux] = max(aux);
    else
        ind_aux = length(objs_match2(1,:));
    end
    % Make objects seen by both cameras unavailable for selection
    for del_ind = 1:length(objs_aux0)
        if ind_aux == objs_aux0(del_ind).label2
            skip = 1; % Object seen by both cameras, no need to repeat it
            free2(ind_aux) = 0;
            break
        end
    end
    if skip == 0
        % Take match from available ones
        if ind_aux ~= length(objs_match2(1,:))
            track2(h) = ind_aux;
            found2 = 0;
            % Restrict that object unless it died
            free2(track2(h)) = 0;
            for ind2 = 1:length(objs_aux2)
                if objs_aux2(ind2).label2 == track2(h)
                    for ind21 = 1:length(objs_aux2_ant)
                        if objs_aux2_ant(ind21).label2 == h
                            % Annex new points to final structure
                            for ind22 = 1:length(objects)
                                if objects(ind22).ind == objs_aux2_ant(ind21).ind
                                    r = ind22;
                                    break
                                end
                            end
                            objects(r).X = cat(1, objects(r).X, objs_aux2(ind2).X);
                            objects(r).Y = cat(1, objects(r).Y, objs_aux2(ind2).Y);
                            objects(r).Z = cat(1, objects(r).Z, objs_aux2(ind2).Z);
                            objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux2(ind2).frames_tracked);
                            objs_aux2(ind2).ind = objects(r).ind;
                            found2 = 1;
                            break
                        end
                    end
                    if found2 == 0
                        for ind21 = 1:length(objs_aux1_ant)
                            if objs_aux1_ant(ind21).label2 == h
                                % Annex new points to final structure
                                for ind22 = 1:length(objects)
                                    if objects(ind22).ind == objs_aux1_ant(ind21).ind
                                        r = ind22;
                                        break
                                    end
                                end
                                objects(r).X = cat(1, objects(r).X, objs_aux2(ind2).X);
                                objects(r).Y = cat(1, objects(r).Y, objs_aux2(ind2).Y);
                                objects(r).Z = cat(1, objects(r).Z, objs_aux2(ind2).Z);
                                objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux2(ind2).frames_tracked);
                                objs_aux2(ind2).ind = objects(r).ind;
                                found2 = 1;
                                break
                            end
                        end
                    end
                    if found2 == 0
                        for ind21 = 1:length(objs_aux0_ant)
                            if objs_aux0_ant(ind21).label2 == h
                                % Annex new points to final structure
                                for ind22 = 1:length(objects)
                                    if objects(ind22).ind == objs_aux0_ant(ind21).ind
                                        r = ind22;
                                        break
                                    end
                                end
                                if objects(r).frames_tracked(end) == objs_aux2(ind2).frames_tracked
                                    for ind_rep = 1:length(objs_aux1)
                                        if objs_aux1(ind_rep).ind == r
                                            m1 = min(objs_aux1(ind_rep).X(1), objs_aux2(ind2).X(1));
                                            M1 = max(objs_aux1(ind_rep).X(3), objs_aux2(ind2).X(3));
                                            m2 = min(objs_aux1(ind_rep).Y(1), objs_aux2(ind2).Y(1));
                                            M2 = max(objs_aux1(ind_rep).Y(2), objs_aux2(ind2).Y(2));
                                            m3 = min(objs_aux1(ind_rep).Z(5), objs_aux2(ind2).Z(5));
                                            M3 = max(objs_aux1(ind_rep).Z(1), objs_aux2(ind2).Z(1));
                                            last = length(objs_aux0) + 1;
                                            objs_aux0(last).X = [m1, m1, M1, M1, m1, m1, M1, M1];
                                            objs_aux0(last).Y = [m2, M2, M2, m2, m2, M2, M2, m2];
                                            objs_aux0(last).Z = [M3, M3, M3, M3, m3, m3, m3, m3];
                                            objs_aux0(last).frames_tracked = objs_aux2(ind2).frames_tracked;
                                            objs_aux0(last).label1 = objs_aux1(ind_rep).label1;
                                            objs_aux0(last).label2 = objs_aux2(ind2).label2;
                                            objs_aux0(last).ind = objects(r).ind;
                                            objs_aux1(ind_rep) = [];
                                            objs_aux2(ind2) = [];
                                            break
                                        else
                                            last = length(objects) + 1;
                                            objects(last).X = objs_aux2(ind2).X;
                                            objects(last).Y = objs_aux2(ind2).Y;
                                            objects(last).Z = objs_aux2(ind2).Z;
                                            objects(last).frames_tracked = objs_aux2(ind2).frames_tracked;
                                            objects(last).label1 = objs_aux2(ind2).label1;
                                            objects(last).label2 = objs_aux2(ind2).label2;
                                            objects(last).ind = last;
                                            objs_aux2(ind2).ind = last;
                                            break
                                        end
                                    end
                                    break
                                end
                                objects(r).X = cat(1, objects(r).X, objs_aux2(ind2).X);
                                objects(r).Y = cat(1, objects(r).Y, objs_aux2(ind2).Y);
                                objects(r).Z = cat(1, objects(r).Z, objs_aux2(ind2).Z);
                                objects(r).frames_tracked = cat(1, objects(r).frames_tracked, objs_aux2(ind2).frames_tracked);
                                objs_aux2(ind2).ind = objects(r).ind;
                                break
                            end
                        end
                    end
                    break
                end
            end
        end
    end
end

free2(end) = 0; % Not needed anymore. We will add newborn objects.
c = find(free2(:) ~= 0);

% Annex newborn objects in final structure
for h = 1:length(c)
    last = length(objects) + 1;
    for ind2 = 1:length(objs_aux2)
        if objs_aux2(ind2).label2 == c(h)
            objects(last).X = objs_aux2(ind2).X;
            objects(last).Y = objs_aux2(ind2).Y;
            objects(last).Z = objs_aux2(ind2).Z;
            objects(last).frames_tracked = objs_aux2(ind2).frames_tracked;
            objects(last).label1 = objs_aux2(ind2).label1;
            objects(last).label2 = objs_aux2(ind2).label2;
            objects(last).ind = last;
            objs_aux2(ind2).ind = last;
            break
        end
    end
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Check nested structure structure                    %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function isFieldResult = myIsField(inStruct, fieldName)
% inStruct is the name of the structure or an array of structures to search
% fieldName is the name of the field for which the function searches

isFieldResult = 0;
f = fieldnames(inStruct(1));

for i=1:length(f)
    if(strcmp(f{i},strtrim(fieldName)))
        isFieldResult = 1;
        return;
    elseif isstruct(inStruct(1).(f{i}))
        isFieldResult = myIsField(inStruct(1).(f{i}), fieldName);
        if isFieldResult
            return;
        end
    end
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Set object indexes to organize 'objects' structure  %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function objects = trim_objects(objects)

if myIsField(objects, 'label1')
    objects = rmfield(objects,'label1');
end
if myIsField(objects, 'label2')
    objects = rmfield(objects,'label2');
end
if myIsField(objects, 'ind')
    objects = rmfield(objects,'ind');
end

n = length(objects);
m = 1;
for k = 1:n
    if length(objects(m).frames_tracked) < 3
        objects(m) = [];
        m = m-1;
    end
    m = m+1;
end
end
