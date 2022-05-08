clear;clc;close all;

 for img_idx = 181:446
     fid = fopen(sprintf('C:/KITTI/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/%010d.bin',img_idx),'rb');
     velo = fread(fid,[4 inf],'single')';
     velo = velo(:,1:3); % 取前三列x y z
     a = pointCloud(velo);
     pcloud(img_idx-180).ptCloud = a;
     fclose(fid);
 end


%%选择要显示的点云
% 为了突出周围的环境, 车辆, 集中在一个地区的利益, 横跨20米左右的车辆, 40 米的前面和后面的车辆。
pc = pcloud(1).ptCloud;

% 设置感兴趣区域(单位米)
xBound  = 40; 
yBound  = 20; 
xlimits = [-xBound, xBound];
ylimits = [-yBound, yBound];
zlimits = pc.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% 裁剪指定范围内的点云
indices = find(pc.Location(:, 2) >= -yBound ...
             & pc.Location(:, 2) <=  yBound ...
             & pc.Location(:, 1) >= -xBound ...
             & pc.Location(:, 1) <=  xBound);

% 将裁剪到的点显示出来
pc = select(pc, indices);
view(player, pc)

%% 分割地平面和附近障碍物
% 找到地面平面并移除地面平面点。使用RANSAC算法检测和匹配地面平面。
% 平面的法线方向应大致沿 Z 轴向上指向。所有 inlier 点必须在地面平面的20厘米以内。

maxDistance = 0.2; % in meters
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);

%%
% 将颜色标签附加到点云中的每个点。使用绿色显示地面平面和红色的障碍, 在10米的激光雷达传感器。
labelSize   = [pc.Count, 1];
colorLabels = zeros(labelSize, 'single');

% 设置用于标记点的颜色表。
colors = [0 0 1; ...  % 蓝色为未标记的点(0 0 1); 指定为[R，G，B]
          0 1 0; ...  % 绿色的为地面平面点(0 1 0)
          1 0 0; ...  % 红色的为障碍点(1 0 0)
          0 0 0];     % 黑色的为自我车辆点(0 0 0)

blueIdx  = 0; % 整个点云最初是蓝色的
greenIdx = 1;
redIdx   = 2;
blackIdx = 3;

% 标出地平面点。
colorLabels(inPlanePointIndices) = greenIdx;

% 选择不属于地平面一部分的点。
pcWithoutGround = select(pc, outliers);

%% 检索半径在15米以内的点, 并将它们标记为障碍物。
sensorLocation   = [0,0,0]; % 将激光雷达传感器放在坐标系的中心
radius           = 15;      % in meters

nearIndices  = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
nearPointIndices = outliers(nearIndices);

% 标记障碍物点
colorLabels(nearPointIndices) = redIdx;

%% 分割自我车辆
% 激光雷达通常安装在车辆的顶部, 数据可能包含车辆本身的点, 如屋顶和引擎盖。
% 这些都不是障碍物，但是最接近激光雷达传感器。在包含车辆的小半径内检索点。
% 使用这些点来形成一个轴对齐的边界立方体来表示自我车辆。
radius      = 2; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle           = select(pcWithoutGround, nearIndices);

%% 形成一个包围立方体和标签的自我车辆点。
delta = 0.1;
selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
            pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
            pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];

colorLabels(vehiclePointIndices) = blackIdx;

%% 将所有标记的点绘制到点云播放器中。使用前面设置的数字颜色标签。
colormap(player.Axes, colors)
points1 = pc.Location;
% points1(inPlanePointIndices, :) = [];
view(player, points1, colorLabels);
title(player.Axes, 'Segmented Point Cloud');

%% 处理点云序列
% for k = 2:length(pcloud)
%     pc = pcloud(k).ptCloud;    
%
%     % Crop the data to ROI.
%     indices = find(pc.Location(:, 2) >= -yBound ...
%                  & pc.Location(:, 2) <=  yBound ...
%                  & pc.Location(:, 1) >= -xBound ...    
%                  & pc.Location(:, 1) <=  xBound);
%     pc = select(pc, indices);
%     
%     colorLabels = zeros(pc.Count, 1, 'single'); % create label array
%     
%     % Find the ground plane.
%     [~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);    
%     colorLabels(inPlanePointIndices) = greenIdx;
% 
%     pcWithoutGround = select(pc, outliers);
%     
%     % Find the points corresponding to obstacles
%     radius           = 10; % in meters
%     nearIndices      = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
%     nearPointIndices = outliers(nearIndices);
%     
%     colorLabels(nearPointIndices) = redIdx;
% 
%     % Find the ego-vehicle points.
%     nearIndices         = findPointsInROI(pcWithoutGround, selfCube);
%     vehiclePointIndices = outliers(nearIndices);
%     
%     colorLabels(vehiclePointIndices) = blackIdx;
%     
%     % Plot the results.
%     view(player, pc.Location, colorLabels);
% end2
colorLabels2=colorLabels;
colorLabels2(colorLabels==1)=[];
X=points1(:,1);
Y=points1(:,2);
Z=points1(:,3);
X(colorLabels==1)=[];
Y(colorLabels==1)=[];
Z(colorLabels==1)=[];
points2=[X,Y,Z];
colormap(player.Axes, colors)
view(player, points2, colorLabels2);
title(player.Axes, 'Segmented Point Cloud');

colorLabels3=colorLabels2;
colorLabels3(colorLabels2==0)=[];
X=points2(:,1);
Y=points2(:,2);
Z=points2(:,3);
X(colorLabels2==0)=[];
Y(colorLabels2==0)=[];
Z(colorLabels2==0)=[];
points3=[X,Y,Z];
colormap(player.Axes, colors)
view(player, points3, colorLabels3);
title(player.Axes, 'Segmented Point Cloud');



%----------DBSCAN聚类---------------%
figure
scatter(points3(:,1),points3(:,2),'.'); % 在向量x和y指定的位置创建一个二维散点图
annotation('ellipse',[0.48 0.48 .1 .1],'Color','red') %画一个红色椭圆，是聚类的初始中心
hold on


% 利用pdist2函数预先计算观测值之间成对距离矩阵D。

D = pdist2(points3,points3);

% 通过使用dbscan和成对距离来对数据进行聚类。指定epsilon值为0.9，min值为50。
[idx, corepts] = dbscan(D,0.9,50,'Distance','precomputed');

% 可视化结果并注释图以突出显示特定的集群。
figure
gscatter(points3(:,1),points3(:,2),idx);
annotation('ellipse',[0.54 0.41 .07 .07],'Color','red')
grid

%获取聚类索引
cluster_idx = unique(idx);

%聚类的个数

numClusters = length(cluster_idx);


for num=1:numClusters
    idxPoints = find(idx == cluster_idx(num));        % 根据聚类标签查找点
    segmented = select(a,idxPoints); % 根据索引提取 
    filename = strcat('DNSCAN_cluster_',num2str(num),'.pcd');
    pcwrite(segmented,filename,'Encoding','binary'); % 保存结果到本地文件夹
end

idxPoints = find(idx~= -1);        % 根据聚类标签查找非噪点(标签为-1的点为噪点)
segmented = select(a,idxPoints); % 根据索引提取 
pcwrite(segmented,'lidar_denoising.pcd','Encoding','binary'); % 保存结果到本地文件夹

figure
subplot(1,2,1)
pcshow(a.Location)
title('原始点云','Color',[1 1 1]);
subplot(1,2,2)
pcshow(segmented.Location)
title('密度去噪','Color',[1 1 1]);
