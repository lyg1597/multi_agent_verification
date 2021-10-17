function map = generate_obstacles(fn)
%GENERATE_OBSTACLES 此处显示有关此函数的摘要
%   此处显示详细说明
image = imread(fn);
image_bw = rgb2gray(image);
image_contrast = imadjust(image_bw, [0.99, 1]);
image_filtered = medfilt2(image_contrast,[3,3]);
% subplot(2,1,1);
% imshow(image_contrast);
% subplot(2,1,2);
% imshow(image_filtered);
% map = 0;
% imshow(image_contrast)
image_binary = (image_filtered < 100)*255;

map = robotics.BinaryOccupancyGrid(image_binary);
show(map);
% points = [];
% for i=1:100000
%     x = randi(map.XWorldLimits);
%     y = randi(map.YWorldLimits);
%     res = getOccupancy(map, [x, y]);
%     if res ==1 
%         points = [points;x,y];
%     end
% end
% plot(points(:,1),points(:,2),'k.');
end

