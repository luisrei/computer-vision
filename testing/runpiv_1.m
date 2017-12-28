%% -- CRIAR LISTA IMAGENS
clear 
close all
clc
%DATA DIRECTORY
base_data_dir='C:\Users\luisr\Desktop\Luis\IST\PIV\pivproject\data\maizena_chocapic\data_rgb\';
d1 = dir(cat(2,base_data_dir, 'depth1*'));
d2 = dir(cat(2,base_data_dir, 'depth2*'));
r1 = dir(cat(2,base_data_dir, 'rgb_image1_*'));
r2 = dir(cat(2,base_data_dir, 'rgb_image2_*'));
for i=1:length(d1)
    temprgb = dir(cat(2,base_data_dir, 'rgb_image1_', num2str(i),'.png'));
    im1(i).rgb = temprgb.name;
    tempdepth = dir(cat(2,base_data_dir, 'depth1_', num2str(i),'.mat'));
    im1(i).depth = tempdepth.name;
    temprgb = dir(cat(2,base_data_dir, 'rgb_image2_', num2str(i),'.png'));
    im2(i).rgb = temprgb.name;
    tempdepth = dir(cat(2,base_data_dir, 'depth2_', num2str(i),'.mat'));
    im2(i).depth = tempdepth.name;
end
%load calibration data
load cameraparametersAsus;
%WORKING DIRECTORY - location where all directories with programs
basedir='C:\Users\luisr\Desktop\Luis\IST\PIV\testing';
%POINTS DIRECTORY
ptsdir = 'C:\Users\luisr\Desktop\Luis\IST\PIV\pivproject\points';
dataset = 'maizena_chocapic';
projs=dir(basedir);
% LOGFILE
outputdir=cat(2,dataset,'/');
fichlog='output.html';
%GET WORLD FRAME TRANSFORMATION
% Get image dimension
load(im1(1).depth);
dim = size(depth_array);

% World reference
cam1toW.R = [1 0 0; 0 1 0; 0 0 1];
cam1toW.T = [0; 0; 0];

% Load depth images
load(im1(1).depth);
depth_array1 = depth_array;
load(im2(1).depth);
depth_array2 = depth_array;

% Get xyz
xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);

% Get points
load(cat(2, ptsdir, '/points_', dataset, '.mat'));
x1 = points(1,:)';
x2 = points(3,:)';
y1 = points(2,:)';
y2 = points(4,:)';

n_points = length(x1);

xyz1_points = zeros(n_points, 3);
xyz2_points = zeros(n_points, 3);

% Get xyz coordinates for points above
for i = 1:n_points
    xyz1_points(i, :) = xyz1(sub2ind(size(depth_array1), y1(i), x1(i)), :);
    xyz2_points(i, :) = xyz2(sub2ind(size(depth_array2), y2(i), x2(i)), :);
end

[d,xx,tr] = procrustes(xyz1_points, xyz2_points,'scaling',false,'reflection',false);
cam2toW.R = tr.T';
cam2toW.T = (tr.c(1,:)).';

if strcmp(dataset, 'lab1')
    cam2toW.R = [0.947680445871955 -0.0773478730699484 -0.309708054534408;...
        0.319055818409704 0.198339956876607 0.926749505662183;...
        -0.0102546209509038 -0.977076541545104 0.212641187711629];
    cam2toW.T = [-0.424194555651687;-0.0778656762093943;0.0460352375503197];
end

%%%%%%%%%%%%%%%%
%PROTEGER DADOS GLOBAIS CONTRA OS CLEAR QUE ALGUNS FAZEM DENTRO DAS FUNCOES
save dados1;
texto={};
caminho={};
%cell(3,length(projs));
%%
try %%%Correr todos os projectos em paralelo!
    %    parfor i=3:length(projs),
    for i=3:length(projs),
        close all; %fechar as janelas que deixam abertas!!!!
        if projs(i).isdir && ~strcmp(projs(i).name,'.') && ~strcmp(projs(i).name , '..'),
            cd([basedir '\' projs(i).name]);
            fprintf('correr o projecto %s \n',pwd);
            if exist('track3D_part1.m'),
                corre='[objects] = track3D_part1( im1, im2, cam_params, cam1toW, cam2toW);';
                try
                    %CHAMA CADA PROJECTO --- MUST RUN -----------
                    h = tic;
                    eval(corre);
                    tt = toc(h);
                    out = cell(3,1);
                    fprintf(' Correu bem \n');
                    texto=[texto [{projs(i).name};{sprintf('OK - Correu ate ao fim em - %d Segundos',tt)}]];
                    out{1}=objects;
                    out{2}=cam1toW;
                    out{3}=cam2toW;
                    caminho=[caminho out];
                catch erro,
                    texto=[texto [{projs(i).name};{sprintf('ERRO - %s  ',erro.message)}]];
                    caminho=[caminho cell(3,1)];
                    fprintf(' Erro %s \n',erro.message);
                end
            else
                texto=[texto [{projs(i).name};{sprintf('ERRO - ficheiro track3D_part1.m inexistente  ')}]];
                caminho=[caminho cell(3,1)];
            end
            cd(basedir)
        end
    end
catch
    cd(basedir)
    save dadoscrash
    fprintf('Deu asneira no loop \n');
    return;
end
save dados2
%%
%SHOW OUTPUT
colors=nchoosek((0:.2:1),3);
load cameraparametersAsus.mat
for i=1:size(texto,2),
    fprintf('Projecto %s - %s \n',texto{1,i}, texto{2,i})
    if strcmp(texto{2,i}(1:2),'OK'),
        obj=caminho{1,i};
        objsinframe=zeros(length(obj),length(im1));
        for j=1:length(obj),
            for k=1:length(obj(j).frames_tracked),
                objsinframe(j,obj(j).frames_tracked(k))=1;
            end
        end
        R1=caminho{2,i}.R;
        T1=caminho{2,i}.T;
        R2=caminho{3,i}.R;
        T2=caminho{3,i}.T;
        for j=1:length(im1),
            load(im1(j).depth);
            xyz1=get_xyzasus(depth_array(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
            load(im2(j).depth);
            xyz2=get_xyzasus(depth_array(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
            xyz=[(R1*xyz1'+repmat(T1,[1,640*480]))';(R2*xyz2'+repmat(T2,[1,640*480]))'];
            pc=pointCloud([xyz(:,1) xyz(:,3) -xyz(:,2)]);%MATLAB AXIS!
            figure(1);
            showPointCloud(pc);
            view([.2 -.2 .05]);
            hold on;
            indsob=find(objsinframe(:,j));
            for k=1:length(indsob),
                ind=find(obj(indsob(k)).frames_tracked==j);
                combs=combnk((1:8),2)';
                xs=obj(indsob(k)).X(ind,:);
                ys=obj(indsob(k)).Z(ind,:);
                zs=-obj(indsob(k)).Y(ind,:);
                line([xs(combs(1,:));xs(combs(2,:))],[ys(combs(1,:));ys(combs(2,:))],[zs(combs(1,:));zs(combs(2,:))],'LineWidth',2);
            end
            hold off;
            pause(0.5);
        end
    end
end