clc;
clear all;
close all;
vid = videoinput('winvideo',2,'YUY2_320x240');

% Set the properties of the video object
set(vid, 'FramesPerTrigger', 3000);
set(vid, 'ReturnedColorspace', 'RGB')
vid.FrameGrabInterval = 10;

%start the video aquisition here
start(vid)



% Set a loop that stop after 100 frames of aquisition

while(vid.FramesAcquired<=100)
face_original = getsnapshot(vid);
%im2=double(im);
size_img = size(face_original);
r = 1; g = 2; b = 3;
y = 1; u = 2; v = 3;

%%YUV Conversion%%
face_yuv = face_original;
for i = 1:size_img(1)
    for j = 1:size_img(2)
     
        face_yuv(i, j, y) = (face_original(i, j, r) + 2*face_original(i, j, g) + face_original(i, j, b)) / 4;
        face_yuv(i, j, u) = face_original(i, j, r) - face_original(i, j, g);
        face_yuv(i, j, v) = face_original(i, j, b) - face_original(i, j, g);
     
        end
end

%%Skin segmentation%%
face_detected = face_original;
for i = 1:size_img(1)
    for j = 1:size_img(2)
        %U range was found based on experiments
        if face_yuv(i, j, u) > 20 && face_yuv(i, j, u) < 74
            %Set suspected face regions to 1
            face_detected(i, j, r) = 255;
            face_detected(i, j, g) = 255;
            face_detected(i, j, b) = 255;
        else
            %Set non-face regions to 0
            face_detected(i, j, r) = 0;
            face_detected(i, j, g) = 0;
            face_detected(i, j, b) = 0;
        end
    end
end

%%Convert image into the BW format; the image itself stays the same, only the format is changed%%
face_detected = im2bw(face_detected);

%%Erode noise%%
face_imerode = imerode(face_detected, strel('square', 3));

%%Fill in holes to get fully connected face regions%%
face_imfill = imfill(face_imerode, 'holes');

%%Label each connected region in the BW image%%
[L, n] = bwlabel(face_imfill); %n gives us #connected objects

face_region = regionprops(L, 'Area', 'BoundingBox');
face_area = [face_region.Area]; %contains areas of all the filled regions

%%Filter out regions whose areas are less than 26% largest area (supposedly a face area)%%
face_idx = find(face_area > (.26)*max(face_area)); %only shows indices of regions that are faces
face_shown = ismember(L, face_idx);

%%Face areas vs. box areas%%
% fprintf('Ratio between face area and box area \n');
for n = 1:length(face_idx)
    idx = face_idx(n);
    area_face(n) = face_region(idx).Area;
    area_box(n) = face_region(idx).BoundingBox(3)*face_region(idx).BoundingBox(4);
    ratio(n) = area_face/area_box;
end
% ratio

%%Compute the coordinates of each face region%%
for n = 1:length(face_idx)
    idx = face_idx(n);
    face_region(idx).BoundingBox;
    xmin = round(face_region(idx).BoundingBox(1));
    ymin = round(face_region(idx).BoundingBox(2));
    xmax = round(face_region(idx).BoundingBox(1)+face_region(idx).BoundingBox(3));
    ymax = round(face_region(idx).BoundingBox(2)+face_region(idx).BoundingBox(4));
    %%Draw a box around each face found%%
    for i = xmin:xmax
        face_original(ymin, i, r) = 255;
        face_original(ymin, i, g) = 0;
        face_original(ymin, i, b) = 0;
    end
    
    for i = xmin:xmax
        face_original(ymax, i, r) = 255;
        face_original(ymax, i, g) = 0;
        face_original(ymax, i, b) = 0;
    end
    
    for j = ymin:ymax
        face_original(j, xmin, r) = 255;
        face_original(j, xmin, g) = 0;
        face_original(j, xmin, b) = 0;
    end
    
    for j = ymin:ymax
        face_original(j, xmax, r) = 255;
        face_original(j, xmax, g) = 0;
        face_original(j, xmax, b) = 0;
    end
end

%imshow(face_original), title ('Original Image')
% imshow(face_detected), title ('Raw Segmentation Result')
% figure, imshow(face_imerode), title ('Eroded Result')
% figure, imshow(face_imfill), title ('Filled Regions')
% figure, imshow(face_shown), title ('Final Result')

%%Compute centroids of the face regions%%
s  = regionprops(face_shown, 'Centroid');
centroids = cat(1, s.Centroid); %cat puts all the s.Centroid values into a matrix
imshow(face_original), title('Detection Result'), xlabel (['#faces found: ', num2str(length(face_idx))])
hold on
plot(centroids(:), centroids(:), 'b*', 'MarkerSize', 10)
end
stop(vid);

% Flush all the image data stored in the memory buffer.
flushdata(vid);

% Clear all variables
clear all
