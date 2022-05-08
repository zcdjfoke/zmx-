clear;clc;close all;

 for img_idx = 181:446
     fid = fopen(sprintf('C:/KITTI/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/%010d.bin',img_idx),'rb');
     velo = fread(fid,[4 inf],'single')';
     velo = velo(:,1:3); % ȡǰ����x y z
     a = pointCloud(velo);
     pcloud(img_idx-180).ptCloud = a;
     fclose(fid);
 end


%%ѡ��Ҫ��ʾ�ĵ���
% Ϊ��ͻ����Χ�Ļ���, ����, ������һ������������, ���20�����ҵĳ���, 40 �׵�ǰ��ͺ���ĳ�����
pc = pcloud(1).ptCloud;

% ���ø���Ȥ����(��λ��)
xBound  = 40; 
yBound  = 20; 
xlimits = [-xBound, xBound];
ylimits = [-yBound, yBound];
zlimits = pc.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% �ü�ָ����Χ�ڵĵ���
indices = find(pc.Location(:, 2) >= -yBound ...
             & pc.Location(:, 2) <=  yBound ...
             & pc.Location(:, 1) >= -xBound ...
             & pc.Location(:, 1) <=  xBound);

% ���ü����ĵ���ʾ����
pc = select(pc, indices);
view(player, pc)

%% �ָ��ƽ��͸����ϰ���
% �ҵ�����ƽ�沢�Ƴ�����ƽ��㡣ʹ��RANSAC�㷨����ƥ�����ƽ�档
% ƽ��ķ��߷���Ӧ������ Z ������ָ������ inlier ������ڵ���ƽ���20�������ڡ�

maxDistance = 0.2; % in meters
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);

%%
% ����ɫ��ǩ���ӵ������е�ÿ���㡣ʹ����ɫ��ʾ����ƽ��ͺ�ɫ���ϰ�, ��10�׵ļ����״ﴫ������
labelSize   = [pc.Count, 1];
colorLabels = zeros(labelSize, 'single');

% �������ڱ�ǵ����ɫ��
colors = [0 0 1; ...  % ��ɫΪδ��ǵĵ�(0 0 1); ָ��Ϊ[R��G��B]
          0 1 0; ...  % ��ɫ��Ϊ����ƽ���(0 1 0)
          1 0 0; ...  % ��ɫ��Ϊ�ϰ���(1 0 0)
          0 0 0];     % ��ɫ��Ϊ���ҳ�����(0 0 0)

blueIdx  = 0; % ���������������ɫ��
greenIdx = 1;
redIdx   = 2;
blackIdx = 3;

% �����ƽ��㡣
colorLabels(inPlanePointIndices) = greenIdx;

% ѡ�����ڵ�ƽ��һ���ֵĵ㡣
pcWithoutGround = select(pc, outliers);

%% �����뾶��15�����ڵĵ�, �������Ǳ��Ϊ�ϰ��
sensorLocation   = [0,0,0]; % �������״ﴫ������������ϵ������
radius           = 15;      % in meters

nearIndices  = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
nearPointIndices = outliers(nearIndices);

% ����ϰ����
colorLabels(nearPointIndices) = redIdx;

%% �ָ����ҳ���
% �����״�ͨ����װ�ڳ����Ķ���, ���ݿ��ܰ�����������ĵ�, ���ݶ�������ǡ�
% ��Щ�������ϰ��������ӽ������״ﴫ�������ڰ���������С�뾶�ڼ����㡣
% ʹ����Щ�����γ�һ�������ı߽�����������ʾ���ҳ�����
radius      = 2; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle           = select(pcWithoutGround, nearIndices);

%% �γ�һ����Χ������ͱ�ǩ�����ҳ����㡣
delta = 0.1;
selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
            pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
            pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];

colorLabels(vehiclePointIndices) = blackIdx;

%% �����б�ǵĵ���Ƶ����Ʋ������С�ʹ��ǰ�����õ�������ɫ��ǩ��
colormap(player.Axes, colors)
points1 = pc.Location;
% points1(inPlanePointIndices, :) = [];
view(player, points1, colorLabels);
title(player.Axes, 'Segmented Point Cloud');

%% �����������
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



%----------DBSCAN����---------------%
figure
scatter(points3(:,1),points3(:,2),'.'); % ������x��yָ����λ�ô���һ����άɢ��ͼ
annotation('ellipse',[0.48 0.48 .1 .1],'Color','red') %��һ����ɫ��Բ���Ǿ���ĳ�ʼ����
hold on


% ����pdist2����Ԥ�ȼ���۲�ֵ֮��ɶԾ������D��

D = pdist2(points3,points3);

% ͨ��ʹ��dbscan�ͳɶԾ����������ݽ��о��ࡣָ��epsilonֵΪ0.9��minֵΪ50��
[idx, corepts] = dbscan(D,0.9,50,'Distance','precomputed');

% ���ӻ������ע��ͼ��ͻ����ʾ�ض��ļ�Ⱥ��
figure
gscatter(points3(:,1),points3(:,2),idx);
annotation('ellipse',[0.54 0.41 .07 .07],'Color','red')
grid

%��ȡ��������
cluster_idx = unique(idx);

%����ĸ���

numClusters = length(cluster_idx);


for num=1:numClusters
    idxPoints = find(idx == cluster_idx(num));        % ���ݾ����ǩ���ҵ�
    segmented = select(a,idxPoints); % ����������ȡ 
    filename = strcat('DNSCAN_cluster_',num2str(num),'.pcd');
    pcwrite(segmented,filename,'Encoding','binary'); % �������������ļ���
end

idxPoints = find(idx~= -1);        % ���ݾ����ǩ���ҷ����(��ǩΪ-1�ĵ�Ϊ���)
segmented = select(a,idxPoints); % ����������ȡ 
pcwrite(segmented,'lidar_denoising.pcd','Encoding','binary'); % �������������ļ���

figure
subplot(1,2,1)
pcshow(a.Location)
title('ԭʼ����','Color',[1 1 1]);
subplot(1,2,2)
pcshow(segmented.Location)
title('�ܶ�ȥ��','Color',[1 1 1]);
