% 多频外差法之三频四相程序
% 程序开始
clc;
close all;
clear;

% 图片的初始化
width = 1280;
heigth = 720;

% 三频率
freq = [70 64 59];  %像素单位为个数，可以看做频率；正弦函数为周期含义

% 利用分块矩阵C存储3组共计12张图
% 三种频率，四组相位
C = cell(3,4);

for i=1:3
    for j=1:4
        C{i,j} = zeros(heigth,width);
    end
end

% 利用余弦函数计算12张图的灰度值
% 图像的生成
% 三种频率，四组相位
for i = 1:3 % 对应三种不同的频率
    for  j = 1:4 % 对应四种相位
        for k = 1:width
            C{i,j}(:,k) = 128+127*sin(2*pi*k*freq(i)/width+j*pi/2);
        end
    end
end

% 对灰度值进行归一化处理
for i = 1:3
    for j = 1:4
        C{i,j} = mat2gray(C{i,j});
    end
end

% 显示保存12张相位移编码图
for i = 1:3
    for j = 1:4
        % n = 4*(i-1)+j;
        % h = figure(n);
        % imshow(C{i,j});
        filename = ['./pattern/vPhase_',num2str(i-1),'_',num2str(j-1),'.bmp'];
        imwrite(C{i,j}, filename); % 保存
    end
end

% 初始化三组处理后的图片灰度矩阵
% phi也是分块矩阵

% 存储相位主值（包裹相位）图像
phi = cell(3,1);
for i = 1:3
    phi{i,1} = zeros(heigth, width);
end

% 求取相位差
% 计算每种频率对应的相位主值
% 输出三种频率的相位主值，用于相差计算
for i = 1:3 % 对于3组中的每一组图片，每一组相同频率的有四张图片
     I1 = C{i,1};
     I2 = C{i,2};
     I3 = C{i,3};
     I4 = C{i,4};
     for g = 1:heigth
        for k = 1:width          
            if I4(g,k)==I2(g,k)&&I1(g,k)>I3(g,k) %四个特殊位置
                phi{i,1}(g,k)=0;
            elseif I4(g,k)==I2(g,k)&&I1(g,k)<I3(g,k) %四个特殊位置
                phi{i,1}(g,k)=pi; 
            elseif I1(g,k)==I3(g,k)&&I4(g,k)>I2(g,k) %四个特殊位置
                phi{i,1}(g,k)=pi/2;
            elseif I1(g,k)==I3(g,k)&&I4(g,k)<I2(g,k) %四个特殊位置
                phi{i,1}(g,k)=3*pi/2;
            elseif I1(g,k)<I3(g,k) %二三象限
                phi{i,1}(g,k)=atan((I4(g,k)-I2(g,k))./(I1(g,k)-I3(g,k)))+pi;
            elseif I1(g,k)>I3(g,k)&&I4(g,k)>I2(g,k) %第一象限
                phi{i,1}(g,k)=atan((I4(g,k)-I2(g,k))./(I1(g,k)-I3(g,k)));
            elseif I1(g,k)>I3(g,k)&&I4(g,k)<I2(g,k) %第四象限
                phi{i,1}(g,k)=atan((I4(g,k)-I2(g,k))./(I1(g,k)-I3(g,k)))+2*pi;  
            end
        end
     end
end

% 计算相差
% 保存矩阵，用于多频相差的计算
PH1 = phi{1,1};   %频率1
PH2 = phi{2,1};   %频率2
PH3 = phi{3,1};   %频率3

% 初始化相差变量
% 多频相差
PH12 = zeros(heigth,width);
PH23 = zeros(heigth,width);
PH123 = zeros(heigth,width);

% 相差计算
% 解相（计算绝对相位）
for g = 1:heigth
	for k = 1:width
		% 计算第一组和第二组的相差
		if PH1(g,k)>PH2(g,k)
			PH12(g,k) = PH1(g,k)-PH2(g,k);
		else
			PH12(g,k) = PH1(g,k)+2*pi-PH2(g,k);
		end
		% 计算第二组和第三组的相差
		if PH2(g,k)>PH3(g,k)
			PH23(g,k) = PH2(g,k)-PH3(g,k);
		else
			PH23(g,k) = PH2(g,k)+2*pi-PH3(g,k);
		end
	   % plot(1,k);
	end
end

% 计算最终相差
% 相差图案
% 相位解包裹  相位展开
for g = 1:heigth
	for k = 1:width 
		if PH12(g,k)>PH23(g,k)
			PH123(g,k) = PH12(g,k)-PH23(g,k);
		else
			PH123(g,k) = PH12(g,k)+2*pi-PH23(g,k);
		end
	end
end

% 显示
% figure,imshow(mat2gray(PH12));title('1,2外差');
imwrite(mat2gray(PH12), './output/12外差.bmp');
% figure,imshow(mat2gray(PH23));title('2,3外差');
imwrite(mat2gray(PH23), './output/23外差.bmp');
% figure,imshow(mat2gray(PH123));title('1,2,3外差');
imwrite(mat2gray(PH123), './output/123外差.bmp');
% figure,imshow(mat2gray(PH1));title('1相位主值');
imwrite(mat2gray(PH1), './output/1相位主值.bmp');
% figure,imshow(mat2gray(PH2));title('2相位主值');
imwrite(mat2gray(PH2), './output/2相位主值.bmp');
% figure,imshow(mat2gray(PH3));title('3相位主值');
imwrite(mat2gray(PH3), './output/3相位主值.bmp');
