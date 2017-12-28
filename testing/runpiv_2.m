%% -- CRIAR LISTA IMAGENS
clear 
close all
clc
%DATA DIRECTORY
base_data_dir='C:\Users\luisr\Desktop\Luis\IST\PIV\pivproject\datasets\maizena_chocapic\data_rgb\';
%WORKING DIRECTORY - location where all directories with programs
basedir='C:\Users\luisr\Desktop\Luis\IST\PIV\testing';
dataset = 'maizena_chocapic';
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
projs=dir(basedir);
% LOGFILE
outputdir=cat(2,dataset,'/');
fichlog='output.html';
%PROTEGER DADOS GLOBAIS CONTRA OS CLEAR QUE ALGUNS FAZEM DENTRO DAS FUNCOES
save dados2;
texto={};
caminho={};
%cell(3,length(projs));
%%
try,%%%Correr todos os projectos em paralelo!
    %    parfor i=3:length(projs),
    for i=3:length(projs),
        close all; %fechar as janelas que deixam abertas!!!!
        if projs(i).isdir && ~strcmp(projs(i).name,'.') && ~strcmp(projs(i).name , '..'),
            cd([basedir '\' projs(i).name]);
            fprintf('correr o projecto %s \n',pwd);
            if exist('track3D_part2.m'),
                corre='[objects, cam1toW, cam2toW] = track3D_part2( im1, im2, cam_params);';
                try,
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