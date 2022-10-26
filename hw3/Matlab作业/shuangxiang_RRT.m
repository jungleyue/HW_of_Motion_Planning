%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=300; y_G=700;       % ����Ŀ���
Thr=50;                 %����Ŀ�����ֵ
Delta= 30;              % ������չ����
%% ������ʼ��
T1.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T1.v(1).y = y_I; 
T1.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T1.v(1).yPrev = y_I;
T1.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T1.v(1).indPrev = 0;     %

T2.v(1).x = x_G;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T2.v(1).y = y_G; 
T2.v(1).xPrev = x_G;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T2.v(1).yPrev = y_G;
T2.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T2.v(1).indPrev = 0;     %
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count1=1;
count2=1;
for iter = 1:3000
    rng('shuffle');
    x_rand=[rand()*xL,rand()*yL];
    
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    
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
    %Step 2: ���������������ҵ�����ڽ���x_near 
    %��ʾ��x_near�Ѿ�����T��
    x_new1=x_rand;
    x_new2=x_rand;
    if(short_dist1^0.5>Delta)
        x_new1=(1.0/(short_dist1^0.5)*Delta)*(x_rand-x_near1)+x_near1;
    end
    if(short_dist2^0.5>Delta)
        x_new2=(1.0/(short_dist2^0.5)*Delta)*(x_rand-x_near2)+x_near2;
    end
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
    
    %���ڵ��Ƿ���collision-free
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

        %Step 4: ��x_new������T 
        %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
        T1.v(count1).x = x_new1(1);         
        T1.v(count1).y = x_new1(2); 
        T1.v(count1).xPrev = x_near1(1);     
        T1.v(count1).yPrev = x_near1(2);
        T1.v(count1).dist=T1.v(flag1).dist+Delta;          
        T1.v(count1).indPrev = flag1; 

        %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
        %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
       %Step 6:��x_near��x_new֮���·��������
       %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
       %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
       hold on;
       line([x_near1(1),x_new1(1)],[x_near1(2),x_new1(2)]);%ʹ��lineʱ��ǰ��ŵ��������x�������������y
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

        %Step 4: ��x_new������T 
        %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
        T2.v(count2).x = x_new2(1);         
        T2.v(count2).y = x_new2(2); 
        T2.v(count2).xPrev = x_near2(1);     
        T2.v(count2).yPrev = x_near2(2);
        T2.v(count2).dist=T2.v(flag2).dist+Delta;          
        T2.v(count2).indPrev = flag2; 

        %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
        %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
       %Step 6:��x_near��x_new֮���·��������
       %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
       %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
       hold on;
       line([x_near2(1),x_new2(1)],[x_near2(2),x_new2(2)]);%ʹ��lineʱ��ǰ��ŵ��������x�������������y
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
   pause(0.01); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 2000
    path1.pos(1).x = T2.v(flag_end2).x; path1.pos(1).y = T2.v(flag_end2).y;
    path1.pos(2).x = T1.v(flag_end1).x; path1.pos(2).y = T1.v(flag_end1).y;
    path1Index = T1.v(flag_end1).indPrev; % �յ����·��
    j=0;
    while 1
        path1.pos(j+3).x = T1.v(path1Index).x;
        path1.pos(j+3).y = T1.v(path1Index).y;
        path1Index = T1.v(path1Index).indPrev;
        if path1Index == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path1.pos(end+1).x = x_I; path1.pos(end).y = y_I; % ������·��
    for j = 2:length(path1.pos)
        plot([path1.pos(j).x; path1.pos(j-1).x], [path1.pos(j).y; path1.pos(j-1).y], 'b', 'Linewidth', 3);
    end
    
    
    path2.pos(1).x = T1.v(flag_end1).x; path2.pos(1).y = T1.v(flag_end1).y;
    path2.pos(2).x = T2.v(flag_end2).x; path2.pos(2).y = T2.v(flag_end2).y;
    path2Index = T2.v(flag_end2).indPrev; % �յ����·��
    j=0;
    while 1
        path2.pos(j+3).x = T2.v(path2Index).x;
        path2.pos(j+3).y = T2.v(path2Index).y;
        path2Index = T2.v(path2Index).indPrev;
        if path2Index == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path2.pos(end+1).x = x_G; path2.pos(end).y = y_G; % ������·��
    for j = 2:length(path2.pos)
        plot([path2.pos(j).x; path2.pos(j-1).x], [path2.pos(j).y; path2.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


