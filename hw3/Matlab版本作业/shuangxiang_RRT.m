%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all;
x_I=1; y_I=1;           % 设置初始点
x_G=300; y_G=700;       % 设置目标点
Thr=50;                 %设置目标点阈值
Delta= 30;              % 设置扩展步长
%% 建树初始化
T1.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T1.v(1).y = y_I; 
T1.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T1.v(1).yPrev = y_I;
T1.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T1.v(1).indPrev = 0;     %

T2.v(1).x = x_G;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T2.v(1).y = y_G; 
T2.v(1).xPrev = x_G;     % 起始节点的父节点仍然是其本身
T2.v(1).yPrev = y_G;
T2.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T2.v(1).indPrev = 0;     %
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count1=1;
count2=1;
for iter = 1:3000
    rng('shuffle');
    x_rand=[rand()*xL,rand()*yL];
    
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    
    short_dist1=1e+10;
    short_dist2=1e+10;
    for i=1:count1
        now_dist=(T1.v(i).x-x_rand(1))^2+(T1.v(i).y-x_rand(2))^2;
        if(now_dist<short_dist1)
            short_dist1=now_dist;
            flag1=i;
        end
    end
    
    for i=1:count2
        now_dist=(T2.v(i).x-x_rand(1))^2+(T2.v(i).y-x_rand(2))^2;
        if(now_dist<short_dist2)
            short_dist2=now_dist;
            flag2=i;
        end
    end
    
    
    x_near1=[T1.v(flag1).x, T1.v(flag1).y];
    x_near2=[T2.v(flag2).x, T2.v(flag2).y];
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    x_new1=x_rand;
    x_new2=x_rand;
    if(short_dist1^0.5>Delta)
        x_new1=(1.0/(short_dist1^0.5)*Delta)*(x_rand-x_near1)+x_near1;
    end
    if(short_dist2^0.5>Delta)
        x_new2=(1.0/(short_dist2^0.5)*Delta)*(x_rand-x_near2)+x_near2;
    end
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    
    %检查节点是否是collision-free
    add_flag1=1;
    add_flag2=1;
    if (~collisionChecking(x_near1,x_new1,Imp))
        add_flag1=0;
    end
    if (~collisionChecking(x_near2,x_new2,Imp))
        add_flag2=0;
    end
    
    if (x_new1(1)<0||x_new1(1)>xL||x_new1(2)<0||x_new1(2)>yL)
        add_flag1=0;
    end
    
    if (x_new2(1)<0||x_new2(1)>xL||x_new2(2)<0||x_new2(2)>yL)
        add_flag2=0;
    end
    
    flag_end2=0;
    flag_end1=0;
                
    if(add_flag1)
        count1=count1+1;

        %Step 4: 将x_new插入树T 
        %提示：新节点x_new的父节点是x_near
        T1.v(count1).x = x_new1(1);         
        T1.v(count1).y = x_new1(2); 
        T1.v(count1).xPrev = x_near1(1);     
        T1.v(count1).yPrev = x_near1(2);
        T1.v(count1).dist=T1.v(flag1).dist+Delta;          
        T1.v(count1).indPrev = flag1; 

        %Step 5:检查是否到达目标点附近 
        %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
       %Step 6:将x_near和x_new之间的路径画出来
       %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
       %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
       hold on;
       line([x_near1(1),x_new1(1)],[x_near1(2),x_new1(2)]);%使用line时，前面放的是两点的x，后面是两点的y
       for i=1:count2
            now_dist=(T2.v(i).x-x_new1(1))^2+(T2.v(i).y-x_new1(1))^2;
            if(now_dist<Thr^2&&(collisionChecking([T2.v(i).x,T2.v(i).y],x_new1,Imp)))
                flag_end2=i;
                flag_end1=count1;
                
                break;
            end
       end
       
    end
    
    if(add_flag2)
        count2=count2+1;

        %Step 4: 将x_new插入树T 
        %提示：新节点x_new的父节点是x_near
        T2.v(count2).x = x_new2(1);         
        T2.v(count2).y = x_new2(2); 
        T2.v(count2).xPrev = x_near2(1);     
        T2.v(count2).yPrev = x_near2(2);
        T2.v(count2).dist=T2.v(flag2).dist+Delta;          
        T2.v(count2).indPrev = flag2; 

        %Step 5:检查是否到达目标点附近 
        %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
       %Step 6:将x_near和x_new之间的路径画出来
       %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
       %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
       hold on;
       line([x_near2(1),x_new2(1)],[x_near2(2),x_new2(2)]);%使用line时，前面放的是两点的x，后面是两点的y
       for i=1:count1
            now_dist=(T1.v(i).x-x_new2(1))^2+(T1.v(i).y-x_new2(1))^2;
            if(now_dist<Thr^2&&(collisionChecking([T1.v(i).x,T1.v(i).y],x_new2,Imp)))
                flag_end1=i;
                flag_end2=count2;
                break;
            end
       end
    end
   if(flag_end1~=0)
       break;
   end
   pause(0.01); %暂停0.1s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 2000
    path1.pos(1).x = T2.v(flag_end2).x; path1.pos(1).y = T2.v(flag_end2).y;
    path1.pos(2).x = T1.v(flag_end1).x; path1.pos(2).y = T1.v(flag_end1).y;
    path1Index = T1.v(flag_end1).indPrev; % 终点加入路径
    j=0;
    while 1
        path1.pos(j+3).x = T1.v(path1Index).x;
        path1.pos(j+3).y = T1.v(path1Index).y;
        path1Index = T1.v(path1Index).indPrev;
        if path1Index == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path1.pos(end+1).x = x_I; path1.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path1.pos)
        plot([path1.pos(j).x; path1.pos(j-1).x], [path1.pos(j).y; path1.pos(j-1).y], 'b', 'Linewidth', 3);
    end
    
    
    path2.pos(1).x = T1.v(flag_end1).x; path2.pos(1).y = T1.v(flag_end1).y;
    path2.pos(2).x = T2.v(flag_end2).x; path2.pos(2).y = T2.v(flag_end2).y;
    path2Index = T2.v(flag_end2).indPrev; % 终点加入路径
    j=0;
    while 1
        path2.pos(j+3).x = T2.v(path2Index).x;
        path2.pos(j+3).y = T2.v(path2Index).y;
        path2Index = T2.v(path2Index).indPrev;
        if path2Index == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path2.pos(end+1).x = x_G; path2.pos(end).y = y_G; % 起点加入路径
    for j = 2:length(path2.pos)
        plot([path2.pos(j).x; path2.pos(j-1).x], [path2.pos(j).y; path2.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


