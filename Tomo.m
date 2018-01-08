function varargout = Tomo(varargin)
% TOMO MATLAB code for Tomo.fig
%      TOMO, by itself, creates a new TOMO or raises the existing
%      singleton*.
%
%      H = TOMO returns the handle to a new TOMO or the handle to
%      the existing singleton*.
%
%      TOMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TOMO.M with the given input arguments.
%
%      TOMO('Property','Value',...) creates a new TOMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Tomo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Tomo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Tomo

% Last Modified by GUIDE v2.5 19-Dec-2017 16:00:52
cd(fileparts(mfilename('fullpath')))
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Tomo_OpeningFcn, ...
    'gui_OutputFcn',  @Tomo_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Tomo is made visible.
function Tomo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Tomo (see VARARGIN)

% Choose default command line output for Tomo
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Tomo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Tomo_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
update_Stat(hObject, eventdata, handles)
% hObject    handle to SelectFolder
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in SelectFolder.
function dname = SelectFolder_Callback(hObject, eventdata, handles)
dir_res = 'C:\';
dname = uigetdir(dir_res);
%
global X;
global XGT;
global alpha;
global FWD;
global theta;
global end_file;
global Err_RMSE;
global Err_Egality;
global Err_Negativity;
%
if dname ~= 0
    % Get the struct of the choosen folder
    D = dir(dname);
    D(1:2) = [];
    % Sort for the oldest to youngest folder
    S = [D(:).datenum].';
    [~,S] = sort(S);
    S = {D(S).name}';
    %
    end_mat_REC1 = '\Data_REC.mat';
    end_mat_FWD1 = '\Data_FWD.mat';
    
    end_mat_REC2 = '\recon.mat';
    end_mat_FWD2 = '\forward.mat';
    handles.InfoPanel.String = ['Chargement des données... 0/',num2str(size(S,1))];
    for ii = 1:size(S,1)
        if ii == 1
            try
                load([dname,'\',S{ii,:},end_mat_REC1]);
                load([dname,'\',S{ii,:},end_mat_FWD1]);
                end_file = 1;
            catch
                load([dname,'\',S{ii,:},end_mat_REC2]);
                load([dname,'\',S{ii,:},end_mat_FWD2]);
                end_file = 2;
            end
        end
        
        if end_file == 1
            load([dname,'\',S{ii,:},end_mat_REC1]);
            load([dname,'\',S{ii,:},end_mat_FWD1]);
        elseif end_file == 2
            load([dname,'\',S{ii,:},end_mat_REC2]);
            load([dname,'\',S{ii,:},end_mat_FWD2]);
        end
        
        if ii == 1
            if end_file == 1
                FWD = FWD_ToSave;
            elseif end_file == 2
                FWD = FWD;
            end
            X = zeros(size(S,1), FWD.dim.M, FWD.dim.Px, FWD.dim.Py);
            XGT = zeros(size(S,1), FWD.dim.M, FWD.dim.Px, FWD.dim.Py);
        end
        %         var{ii} = REC_ToSave;
        if end_file == 1
            if numel(REC_ToSave.alpha) ~= 1
                [~,p] = min(REC_ToSave.error);
            else
                p = 1;
            end
        elseif end_file == 2
            if numel(REC.param.alpha) ~= 1
                [~,p] = min(REC.final.error);
            else
                p = 1;
            end
        end
        % Formatting data to speed up display
        if end_file == 1
            XGT(ii,:,:,:) = reshape(FWD_ToSave.X(:),FWD.dim.M,FWD.dim.Px,FWD.dim.Py);
            X(ii,:,:,:)   = reshape(REC_ToSave.XforEachAlpha{p}(:),FWD.dim.M,FWD.dim.Px,FWD.dim.Py);
        elseif end_file == 2
            XGT(ii,:,:,:) = reshape(FWD.A(:),FWD.dim.M,FWD.dim.Px,FWD.dim.Py);
            X(ii,:,:,:)   = reshape(REC.A{p}(:),FWD.dim.M,FWD.dim.Px,FWD.dim.Py);
        end
        % Erreurs
        % RMSE
        for mm = 1:FWD.dim.M
            Err_RMSE(ii,mm) = 1 / FWD.dim.M * ((norm(squeeze(X(ii,mm,:,:) - XGT(ii,mm,:,:)))) / norm(squeeze(XGT(ii,mm,:,:))));
        end
        % Egality
        for mm = 1:FWD.dim.M
            TrueSum = sum(sum(squeeze(XGT(ii,mm,:,:)),1),2);
            SumX    = sum(sum(squeeze(X(ii,mm,:,:)),1),2);
            Err_Egality(ii,mm) = 1 / FWD.dim.M * sum((abs(SumX - TrueSum) ./ TrueSum));
        end
        % Negativity
        X_temp = reshape(X(ii,:,:,:), FWD.dim.M, FWD.dim.P);
        for mm = 1:FWD.dim.M
            Err_Negativity(ii,mm) = sum(squeeze(X_temp(mm,:)) < 0) / (FWD.dim.P) * 100;
        end
        %
        if end_file == 1
            bool_th = find(S{ii} == '-',2,'first');
            if ~isempty(bool_th)
                theta(ii) = str2double(S{ii}(bool_th(1)+1:bool_th(2)-1));
            end
        elseif end_file == 2
            theta(ii) = str2double(S{ii}(end-2:end));
        end
        %
        handles.InfoPanel.String = ['Chargement des données... ',num2str(ii),'/',num2str(size(S,1))];
        drawnow;
    end
    if end_file == 1
        alpha = REC_ToSave.alpha;
        if numel(alpha) ~= 1
            [~,p] = min(REC_ToSave.error);
            alpha = REC_ToSave.alpha(p);
        end
    elseif end_file == 2
        alpha = REC.param.alpha;
        if numel(alpha) ~= 1
            [~,p] = min(REC.final.error);
            alpha = REC.param.alpha(p);
        end
    end
    
    sortData();
    % Change sliders, display the loading is done
    handles.InfoPanel.String      = 'Chargement des données finis';
    handles.slider3.Value         = 1;
    handles.slider3.Min           = 1;
    handles.slider3.Max           = FWD.dim.M;
    handles.slider3.SliderStep(1) = 1 / (FWD.dim.M-1);
    handles.slider3.SliderStep(2) = 1 / (FWD.dim.M-1);
    %
    handles.slider4.Value         = 1;
    handles.slider4.Min           = 1;
    handles.slider4.Max           = FWD.dim.M;
    handles.slider4.SliderStep(1) = 1 / (FWD.dim.M-1);
    handles.slider4.SliderStep(2) = 1 / (FWD.dim.M-1);
    %
    if size(S,1) == 1
        handles.slider1.Visible = 'off';
    else
        handles.slider1.Visible = 'on';
    end
    handles.slider1.Value = 1;
    handles.slider1.Min = 1;
    handles.slider1.Max = size(S,1);
    % Display an image and wait
    update_im_proj(hObject, eventdata, handles);
end
% hObject    handle to SelectFolder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function sortData()
global theta;
global X;
global XGT;
a = 1;
if ~issorted(theta)
    [theta, I] = sort(theta);
    X          = X(I,:,:,:);
    XGT        = XGT(I,:,:,:);
end

% --- Executes on button press in RunTomo.
function RunTomo_Callback(hObject, eventdata, handles)
global FWD;
global theta;
global Vol_Tomo;
global Vol_Tomo_GT;
global X;
global XGT;
global end_file;

if end_file == 1
    slice_max = FWD.dim.Py;
    for mm = 1:FWD.dim.M
        for ss = 1:slice_max
            if mm == 1 && ss == 1
                t = iradon(permute(squeeze(X(:,mm,:,ss)),  [2 1]), theta);
                Vol_Tomo = zeros(FWD.dim.M,size(t,1),size(t,2),slice_max);
                Vol_Tomo_GT = zeros(FWD.dim.M,size(t,1),size(t,2),slice_max);
            end
            Vol_Tomo(mm,:,:,ss)    =  iradon(permute(squeeze(X(:,mm,:,ss)),  [2 1]), theta);
            %
            Vol_Tomo_GT(mm,:,:,ss) =  iradon(permute(squeeze(XGT(:,mm,:,ss)),[2 1]), theta);
            %
            handles.InfoPanel.String = ['Tomo : M = ',num2str(mm),'/',num2str(FWD.dim.M),' || Slice : ',num2str(ss),'/',num2str(slice_max)];
            drawnow;
        end
    end
elseif end_file == 2
    slice_max = FWD.dim.Px;
    for mm = 1:FWD.dim.M
        for ss = 1:slice_max
            if mm == 1 && ss == 1
                t = iradon(permute(squeeze(X(:,mm,ss,:)),  [2 1]), theta);
                Vol_Tomo = zeros(FWD.dim.M,size(t,1),size(t,2),slice_max);
                Vol_Tomo_GT = zeros(FWD.dim.M,size(t,1),size(t,2),slice_max);
            end
            Vol_Tomo(mm,:,:,ss)    =  iradon(permute(squeeze(X(:,mm,ss,:)),  [2 1]), theta);
            %
            Vol_Tomo_GT(mm,:,:,ss) =  iradon(permute(squeeze(XGT(:,mm,ss,:)),[2 1]), theta);
            %
            handles.InfoPanel.String = ['Tomo : M = ',num2str(mm),'/',num2str(FWD.dim.M),' || Slice : ',num2str(ss),'/',num2str(slice_max)];
            drawnow;
        end
    end
end
handles.InfoPanel.String = 'Tomo finie';
%
handles.slider2.Value = 1;
handles.slider2.Min = 1;
handles.slider2.Max = slice_max;
update_im_slice(hObject, eventdata, handles);
% hObject    handle to RunTomo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function Proj_Callback(hObject, eventdata, handles)
handles.slider1.Value = floor(str2num(handles.Proj.String(8:end)));
slider1_Callback(hObject, eventdata, handles)
% hObject    handle to Proj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Proj as text
%        str2double(get(hObject,'String')) returns contents of Proj as a double


% --- Executes during object creation, after setting all properties.
function Proj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Proj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function InfoPanel_Callback(hObject, eventdata, handles)
% hObject    handle to InfoPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of InfoPanel as text
%        str2double(get(hObject,'String')) returns contents of InfoPanel as a double


% --- Executes during object creation, after setting all properties.
function InfoPanel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to InfoPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
update_im_slice(hObject, eventdata, handles);
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Slice_Callback(hObject, eventdata, handles)
% hObject    handle to slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of slice as text
%        str2double(get(hObject,'String')) returns contents of slice as a double


% --- Executes during object creation, after setting all properties.
function Slice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
update_im_slice(hObject, eventdata, handles);
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Mat_Callback(hObject, eventdata, handles)
handles.slider3.Value = floor(str2num(handles.Mat.String(7:end)));
slider3_Callback(hObject, eventdata, handles);
% hObject    handle to Mat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mat as text
%        str2double(get(hObject,'String')) returns contents of Mat as a double


% --- Executes during object creation, after setting all properties.
function Mat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function slice_Callback(hObject, eventdata, handles)
% hObject    handle to slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of slice as text
%        str2double(get(hObject,'String')) returns contents of slice as a double


% --- Executes during object creation, after setting all properties.
function slice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mat2Slice_Callback(hObject, eventdata, handles)
% hObject    handle to Mat2Slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mat2Slice as text
%        str2double(get(hObject,'String')) returns contents of Mat2Slice as a double


% --- Executes during object creation, after setting all properties.
function Mat2Slice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mat2Slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function update_im_proj(hObject, eventdata, handles)
global FWD;
switch handles.popupmenu1.Value
    case 1 % Decompositions
        global X;
        global XGT;
        axes(handles.axes1);
        cla reset;
        Proj = floor(handles.slider1.Value);
        Mat  = floor(handles.slider3.Value);
        
        if handles.groundtruth.Value
            xx = squeeze(XGT(Proj,:,:,:));
        else
            xx = squeeze(X(Proj,:,:,:));
        end
        
        if handles.NegValueBox.Value
            xx = xx < 0;
        end
        
        if handles.overlay.Value
            im_to_disp =  xx ./ max(max(xx,[],2),[],3);
            im_to_disp = squeeze(permute(im_to_disp,[2 3 1]));
            %
        else
            im_to_disp = squeeze(xx(Mat,:,:));
            %
            handles.Mat.String = ['Mat : ',num2str(Mat)];
        end
        handles.Proj.String = ['Proj : ',num2str(Proj)];
        imagesc(im_to_disp), colormap(hot); axis image
        if handles.colorbar.Value
            colorbar;
        end
    case 2 % RMSE
        global Err_RMSE;
        axes(handles.axes1);
        cla reset;
        hold all;
        str = {};
        for mm = 1:FWD.dim.M
            str = [str, ['Mat m = ',num2str(mm)]];
            loglog(handles.axes1,1:size(Err_RMSE,1), squeeze(Err_RMSE(:,mm)),'*-','LineWidth',1,'MarkerSize',5); hold on
        end
        str = [str, 'Sum over materials'];
        loglog(handles.axes1,1:size(Err_RMSE,1), sum(Err_RMSE,2),'*-','LineWidth',1,'MarkerSize',5); hold off
        legend(str,'FontSize',10,'Interpreter','latex');
        xlabel('Theta','FontSize',10,'Interpreter','latex');
        ylabel('RMSE','FontSize',7,'Interpreter','latex');
        grid on;
        set(gca,'XLim',[1, size(Err_RMSE,1)]);
        set(gca,'FontSize',7);
        
    case 3 % Egality
        global Err_Egality;
        axes(handles.axes1);
        cla reset;
        hold all;
        str = {};
        for mm = 1:FWD.dim.M
            str = [str, ['Mat m = ',num2str(mm)]];
            loglog(handles.axes1,1:size(Err_Egality,1), squeeze(Err_Egality(:,mm)),'*-','LineWidth',1,'MarkerSize',5); hold on
        end
        str = [str, 'Sum over materials'];
        loglog(handles.axes1,1:size(Err_Egality,1), sum(Err_Egality,2),'*-','LineWidth',1,'MarkerSize',5); hold off
        legend(str,'FontSize',10,'Interpreter','latex');
        xlabel('Theta','FontSize',10,'Interpreter','latex');
        ylabel('Egality','FontSize',7,'Interpreter','latex');
        grid on;
        set(gca,'XLim',[1, size(Err_Egality,1)]);
        set(gca,'FontSize',7);
    case 4 % Negativity
        global Err_Negativity;
        axes(handles.axes1);
        cla reset;
        hold all;
        str = {};
        for mm = 1:FWD.dim.M
            str = [str, ['Mat m = ',num2str(mm)]];
            loglog(handles.axes1,1:size(Err_Negativity,1), squeeze(Err_Negativity(:,mm)),'*-','LineWidth',1,'MarkerSize',5); hold on
        end
        str = [str, 'Sum over materials'];
        loglog(handles.axes1,1:size(Err_Negativity,1), sum(Err_Negativity,2),'*-','LineWidth',1,'MarkerSize',5); hold off
        legend(str,'FontSize',10,'Interpreter','latex');
        xlabel('Theta','FontSize',10,'Interpreter','latex');
        ylabel('Egality','FontSize',7,'Interpreter','latex');
        grid on;
        set(gca,'XLim',[1, size(Err_Negativity,1)]);
        set(gca,'FontSize',7);
end

function update_im_slice(hObject, eventdata, handles)
global Vol_Tomo;
global Vol_Tomo_GT;
if ~isempty(Vol_Tomo)
    axes(handles.axes2);
    Slice = floor(handles.slider2.Value);
    Mat  = floor(handles.slider4.Value);
    
    if handles.groundtruth.Value
        xx = squeeze(Vol_Tomo_GT(:,:,:,Slice));
    else
        xx = squeeze(Vol_Tomo(:,:,:,Slice));
    end
    
    if handles.overlay.Value
        im_to_disp =  xx ./ max(max(xx,[],2),[],3);
        im_to_disp = squeeze(permute(im_to_disp,[2 3 1]));
        %
    else
        im_to_disp = squeeze(xx(Mat,:,:));
        %
        handles.Mat2Slice.String = ['Mat : ',num2str(Mat)];
    end
    handles.slice.String = ['Slice : ',num2str(Slice)];
    imagesc(im_to_disp), colormap(hot); axis image
    if handles.colorbar.Value
        colorbar;
    end
end

function update_Stat(hObject, eventdata, handles)
ind = handles.listbox1.Value;
angle = floor(handles.slider1.Value);
global Err_RMSE;
global Err_Negativity;
global Err_Egality;
global X;
        str = {};
        str = handles.listbox1.String(ind);
        str = [str; ['Mat           |       Value']];

switch ind
    case 1 % RMSE Current Angle
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(Err_RMSE(angle,mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all RMSE / M = ',num2str(sum(Err_RMSE(angle,:)) / size(X,2))]];
    case 2 % Egality Current Angle
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(Err_Egality(angle,mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all egality error / M = ',num2str(sum(Err_Egality(angle,:)) / size(X,2))]];
    case 3 % Negativity Current Angle
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(Err_Negativity(angle,mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all negativity percentage / M = ',num2str(sum(Err_Negativity(angle,:)) / size(X,2))]];
    case 4 % Mean RMSE
        meanRMSE = mean(Err_RMSE,1);
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(meanRMSE(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all mean RMSE / M = ',num2str(sum(meanRMSE) / size(X,2))]];
    case 5 % Mean Egality
        meanEg = mean(Err_Egality,1);
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(meanEg(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all mean egality error / M = ',num2str(sum(meanEg) / size(X,2))]];
    case 6 % Mean Negativity
        meanEg = mean(Err_Negativity,1);
        for mm = 1:size(X,2)
            str = [str;[num2str(mm),'               |       ',num2str(meanEg(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Sum of all mean negativity percentage / M = ',num2str(sum(meanEg) / size(X,2))]];
    case 7 % Minimum Value Current Angle
        XAngle = reshape(squeeze(X(angle,:,:,:)), size(X,2), size(X,3) * size(X,4));   
        for mm = 1:size(X,2)
            minX(mm) = min(squeeze(XAngle(mm,:)));
            str = [str;[num2str(mm),'               |       ',num2str(minX(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Min of the current angle = ',num2str(min(minX))]];
    case 8 % Maximum Value Current Angle   
        XAngle = reshape(squeeze(X(angle,:,:,:)), size(X,2), size(X,3) * size(X,4));   
        for mm = 1:size(X,2)
            maxX(mm) = max(squeeze(XAngle(mm,:)));
            str = [str;[num2str(mm),'               |       ',num2str(maxX(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Max of the current angle = ',num2str(max(maxX))]];  
    case 9 % Minimum Value All Angle
         XTemp = reshape(X, size(X,1), size(X,2), size(X,3) * size(X,4));   
        for mm = 1:size(X,2)
            xx = squeeze(XTemp(:,mm,:));
            minX(mm) = min(xx(:));
            str = [str;[num2str(mm),'               |       ',num2str(minX(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Min of all angle = ',num2str(min(minX))]];
    case 10 % Maximum Value All Angle
         XTemp = reshape(X, size(X,1), size(X,2), size(X,3) * size(X,4));   
        for mm = 1:size(X,2)
            xx = squeeze(XTemp(:,mm,:));
            maxX(mm) = max(xx(:));
            str = [str;[num2str(mm),'               |       ',num2str(maxX(mm))]];
        end
        str = [str;''];
        str = [str;'---------------------'];
        str = [str; ['Max of all angle = ',num2str(min(maxX))]];
end
        handles.text4.String = str;

% --- Executes during object creation, after setting all properties.
function overlay_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
update_im_slice(hObject, eventdata, handles);

if handles.overlay.Value
    handles.Mat.Visible = 'off';
    handles.slider3.Visible = 'off';
    handles.Mat2Slice.Visible = 'off';
    handles.slider4.Visible = 'off';
else
    handles.Mat.Visible = 'on';
    handles.slider3.Visible = 'on';
    handles.Mat2Slice.Visible = 'on';
    handles.slider4.Visible = 'on';
end
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called% --- Executes during object creation, after setting all properties.

function overlay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% Hint: delete(hObject) closes the figure
delete(hObject);
clear;
clear global;
clc;
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in colorbar.
function colorbar_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
update_im_slice(hObject, eventdata, handles);
% hObject    handle to colorbar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of colorbar


% --- Executes on button press in groundtruth.
function groundtruth_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
update_im_slice(hObject, eventdata, handles);
% hObject    handle to groundtruth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of groundtruth


% --- Executes during object deletion, before destroying properties.
function Mat2Slice_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to Mat2Slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in NegValueBox.
function NegValueBox_Callback(hObject, eventdata, handles)
update_im_proj(hObject, eventdata, handles);
update_im_slice(hObject, eventdata, handles);
% hObject    handle to NegValueBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of NegValueBox


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
update_Stat(hObject, eventdata, handles)
% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox4.
function listbox4_Callback(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox4


% --- Executes during object creation, after setting all properties.
function listbox4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
