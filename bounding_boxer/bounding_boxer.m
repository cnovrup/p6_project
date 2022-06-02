clear
clc
X = [751;-10;116;1]; % middle of bounding box - 3D coordinate - world coordinates
w = 60; % width of bounding box
h = 138; % height of bounding box
imgwidth = 1920; % pixel width of image
imgheight = 1080; % pixel height of image
savebox = false; % save images with bounding box - only set true with few images, or else script is very slow
filetype = '.png';

imagedir = 'images'; % sample images folder name
datadir = 'data'; % data folder name
bboxdir = 'bounding_box'; % folder name for bounding box text data
bb_images = 'bb_images'; % folder for images with bounding box

% intrinsic parameters from camera - obtained via. the camera API
% https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html#pyrealsense2.intrinsics
% high res intrinsics
fx = 1366.22; % Focal length of the image plane, as a multiple of pixel width
fy = 1366.62; % % Focal length of the image plane, as a multiple of pixel height
u0 = 956.246; % Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
v0 = 557.477; % Vertical coordinate of the principal point of the image, as a pixel offset from the top edge
% low res intrinsics 
% fx = 607.137386191766; % Focal length of the image plane, as a multiple of pixel width
% fy = 607.084913879017; % % Focal length of the image plane, as a multiple of pixel height
% u0 = 318.577798089646; % Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
% v0 = 247.839100800778; % Vertical coordinate of the principal point of the image, as a pixel offset from the top edge

images = dir([imagedir '/*.png']);
imagenames = struct2cell(images);
imagenames = imagenames(1,:);
imagenumber = numel(images);

% calculate 3D world coordinates for bounding box
[p1,p2,p3,p4,p5,p6,p7,p8] = get3dBox(X,w,h);
P_world = [p1 p2 p3 p4 p5 p6 p7 p8];

% intrinsic camera matrix
K = [fx 0 u0;
     0 fy v0;
     0 0 1]; 

for i=1:2
    imgname = char(imagenames(i));
    imgname = imgname(1:end-4);
    datafile = fopen([datadir '/' imgname '.txt'],'r');
    bboxfile = fopen([bboxdir '/' imgname '.txt'],'w');
    data = textscan(datafile, '%s', 'Delimiter', '\n');
    data = data{1};
    data = data{2};
    data = split(data, ' ');
    q = str2double(data);

    T = base2camera(q(1), q(2), q(3), q(4), q(5), q(6));
    R = T(1:3, 1:3); % rotation matrix
    t = T(1:3,4); % translation vector

    for j=1:length(P_world)
        u(:,j) = get2dPoint(K,R,t,P_world(:,j));
        %plot(u(1,i),u(2,i), '*')
    end
    max_x = max(u(1,:));
    min_x = min(u(1,:));
    max_y = max(u(2,:));
    min_y = min(u(2,:));
    b_box = [min_x,min_y,(max_x-min_x),(max_y-min_y)];
    b_box_norm = [((((max_x-min_x)/2)+min_x)/imgwidth),((((max_y-min_y)+min_y)/2)/imgheight),(max_x-min_x)/imgwidth,(max_y-min_y)/imgheight];
    fprintf(bboxfile, '%i %f %f %f %f', [0, b_box_norm]);
    fclose(datafile);
    fclose(bboxfile);
    
    if(savebox==true)
        im = imread([imagedir '/' imgname filetype]);
        set(gcf,'Visible','off');
        imshow(im)
        rectangle('Position', b_box, EdgeColor=[0 1 0], LineWidth=1)
        exportgraphics(gcf, [bb_images '/' imgname filetype],'Resolution',300)
    end
end
disp('done');