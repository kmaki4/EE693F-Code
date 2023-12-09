function varargout = SimRIS_GUI(varargin)
% SIMRIS_GUI MATLAB code for SimRIS_GUI.fig
%      SIMRIS_GUI, by itself, creates a new SIMRIS_GUI or raises the existing
%      singleton*.
%
%      H = SIMRIS_GUI returns the handle to a new SIMRIS_GUI or the handle to
%      the existing singleton*.
%
%      SIMRIS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMRIS_GUI.M with the given input arguments.
%
%      SIMRIS_GUI('Property','Value',...) creates a new SIMRIS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SimRIS_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SimRIS_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SimRIS_GUI

% Last Modified by GUIDE v2.5 18-May-2020 21:30:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SimRIS_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SimRIS_GUI_OutputFcn, ...
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


% --- Executes just before SimRIS_GUI is made visible.
function SimRIS_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SimRIS_GUI (see VARARGIN)

% Choose default command line output for SimRIS_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SimRIS_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SimRIS_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
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


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Choose Environment
Environment_content = get(handles.popupmenu2,'String'); 
Environment_choice = Environment_content{get(handles.popupmenu2,'Value')};
  switch Environment_choice
      case 'Indoor (InH Office)'
        Environment=1;
      case 'Outdoor (UMi Street Canyon)'
        Environment=2;
  end
%% Choose RIS position
Position_content = get(handles.popupmenu3,'String'); 
Position_choice = Position_content{get(handles.popupmenu3,'Value')};
  switch Position_choice
      case 'yz plane (Scenario 2)'
        Scenario=2;
      case 'xz plane (Scenario 1)'
        Scenario=1;
  end
    
%% Choose Frequency
Frequency_content = get(handles.popupmenu7,'String'); 
Frequency_choice = Frequency_content{get(handles.popupmenu7,'Value')};
  switch Frequency_choice
      case '28 GHz'
        Frequency=28;
      case '73 GHz'
        Frequency=73;
  end
  lambda=(3*10^8)/(Frequency*10^9);  % Wavelength;
%% Number of RIS elements    
  N=str2double(get(handles.edit1,'string'));
 if mod(sqrt(N),1)~=0
%     error('N should be an integer power of 2')
    set(handles.text22,'String', sprintf('Error: N should be an even power of 2!'));
    set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
 else
     set(handles.text22,'String', sprintf(''));
 end
 
 %% Number of Channel Realizations
 Nsym=str2double(get(handles.edit14,'string'));
 %% Transmitter Location    
 Tx_x=str2double(get(handles.edit2,'string'));
 Tx_y=str2double(get(handles.edit3,'string'));
 Tx_z=str2double(get(handles.edit4,'string'));
 Tx_xyz=[Tx_x,Tx_y,Tx_z];       % Tx coordinates (in meters)
 if Tx_x~=0
%     error('N should be an integer power of 2')
    set(handles.text32,'String', sprintf('Error: Tx should be on xz plane with x=0'));
    set(handles.text26,'String', sprintf('Warning! Check the error box!'));
    
 else
     set(handles.text32,'String', sprintf(''))
 end
 

 %% Receiver Location    
 Rx_x=str2double(get(handles.edit8,'string'));
 Rx_y=str2double(get(handles.edit9,'string'));
 Rx_z=str2double(get(handles.edit10,'string'));
 Rx_xyz=[Rx_x,Rx_y,Rx_z];       % Rx coordinates (in meters)
   if Rx_z>2
%     error('N should be an integer power of 2')
    set(handles.text46,'String', sprintf('Error: Rx is a ground user equipment! Height should be less than 2 in the z-axis!'));
    set(handles.text26,'String', sprintf('Warning! Check the error box!'));
    
 else
     set(handles.text46,'String', sprintf(''))
 end
 
 %% RIS Location    
 RIS_x=str2double(get(handles.edit13,'string'));
 RIS_y=str2double(get(handles.edit12,'string'));
 RIS_z=str2double(get(handles.edit11,'string'));
 RIS_xyz=[RIS_x,RIS_y,RIS_z];   % RIS coordinates (in meters) 
 d_T_RIS = norm(Tx_xyz-RIS_xyz);  
 d_RIS_R = norm(RIS_xyz-Rx_xyz);
 d_T_R = norm(Tx_xyz-Rx_xyz);
 
  if Environment==1 && Tx_z>3
     set(handles.text40,'String', sprintf('Error: Typical Tx height 2-3 meters for InH-Office'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
  elseif Environment==2 && Tx_z>20
      set(handles.text40,'String', sprintf('Error: Typical Tx height 20 meters for UMi-Street Canyon'))
         set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
       else
     set(handles.text40,'String', sprintf(''))
 end
 
 if Environment==1 && d_RIS_R>10
     set(handles.text33,'String', sprintf('Error: Rx is far away (max 8-10 meters) from RIS'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
      else
     set(handles.text33,'String', sprintf(''))
 end
 
 if Environment==1 && d_T_R>75
     set(handles.text39,'String', sprintf('Error: Typical cell radius max 75 meters for InH-Office'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
 elseif Environment==2 && d_T_R>100
     set(handles.text39,'String', sprintf('Error: Typical cell radius max 100 meters for UMi-Street Canyon'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
 else
     set(handles.text39,'String', sprintf(''))
 end
 
 cond = N*lambda/2; %far-field condition
 if d_T_RIS<cond 
     set(handles.text41,'String', sprintf('Error: Far field condition is not satisfied'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
 elseif d_RIS_R<cond 
      set(handles.text41,'String', sprintf('Error: Far field condition is not satisfied'))
         set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
elseif d_T_R<cond
           set(handles.text41,'String', sprintf('Error: Far field condition is not satisfied'))
              set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
 else
     set(handles.text41,'String', sprintf(''))
 end
 
  if Environment==2 && Tx_z<RIS_z
     set(handles.text45,'String', sprintf('Tx should be higher than Rx and RIS for outdoor'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
  elseif Environment==2 && Tx_z<Rx_z
     set(handles.text45,'String', sprintf('Tx should be higher than Rx and RIS for outdoor'))
        set(handles.text26,'String', sprintf('Warning! Check the error box!'));
   return
  else
      set(handles.text45,'String', sprintf(''))
 end
set(handles.text26, 'String', 'SimRIS is running...');
pause(3);
tic
parfor repeat=1:Nsym 
[h(:,repeat),g(:,repeat),h_SISO(:,repeat)]=SimRIS_v15(Environment,Scenario,Frequency,N,Tx_xyz,Rx_xyz,RIS_xyz);
% save('pqfile.mat','h')
end
toc
 handles.channel_h = h;
 handles.channel_g = g;
 handles.channel_h_SISO = h_SISO;
assignin('base', 'h', h);
assignin('base', 'g', g);
assignin('base', 'h_SISO', h_SISO);
set(handles.text26, 'String', 'SimRIS executed succesfully. Tx-RIS (h), RIS-Rx(g), and Tx-Rx channels (h_SISO) are generated!');
set(handles.text27, 'String', sprintf('Simulation Time: %0.4f sec',toc));

sprintf('Simulation Time: %0.rf sec',toc)
% uisave({'h','g','h_SISO'},'Channel_vectors')
guidata(hObject,handles)

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Choose Environment
Environment_content = get(handles.popupmenu2,'String'); 
Environment_choice = Environment_content{get(handles.popupmenu2,'Value')};
  switch Environment_choice
      case 'Indoor (InH Office)'
        Environment=1;
      case 'Outdoor (UMi Street Canyon)'
        Environment=2;
  end
%% Choose RIS position
Position_content = get(handles.popupmenu3,'String'); 
Position_choice = Position_content{get(handles.popupmenu3,'Value')};
  switch Position_choice
      case 'yz plane (Scenario 2)'
        Scenario=2;
      case 'xz plane (Scenario 1)'
        Scenario=1;
  end
  
% % for test (Scenario 2 - Indoors)
% Tx_xyz=[0,25,2];       % Tx coordinates (in meters)
% Rx_xyz=[70,30,1];     % Rx coordinates (in meters)
% RIS_xyz=[75,25,2];       % RIS coordinates (in meters)
if Environment==1 && Scenario==1
    set(handles.edit2,'String', '0');   % Recommenden Tx_x
    set(handles.edit3,'String', '25');  % Recommenden Tx_y
    set(handles.edit4,'String', '2');   % Recommenden Tx_z
    set(handles.edit8,'String', '38');% Recommenden Rx_x
    set(handles.edit9,'String', '48');  % Recommenden Rx_y
    set(handles.edit10,'String', '1');  % Recommenden Rx_z
    set(handles.edit13,'String', '40'); % Recommenden RIS_x
    set(handles.edit12,'String', '50'); % Recommenden RIS_y
    set(handles.edit11,'String', '2');  % Recommenden RIS_z   
elseif  Environment==1 && Scenario==2
    set(handles.edit2,'String', '0');   % Recommenden Tx_x
    set(handles.edit3,'String', '25');  % Recommenden Tx_y
    set(handles.edit4,'String', '2');   % Recommenden Tx_z
    set(handles.edit8,'String', '70');% Recommenden Rx_x
    set(handles.edit9,'String', '35');  % Recommenden Rx_y
    set(handles.edit10,'String', '1');  % Recommenden Rx_z
    set(handles.edit13,'String', '70'); % Recommenden RIS_x
    set(handles.edit12,'String', '30'); % Recommenden RIS_y
    set(handles.edit11,'String', '2');  % Recommenden RIS_z
elseif  Environment==2 && Scenario==1
    set(handles.edit2,'String', '0');   % Recommenden Tx_x
    set(handles.edit3,'String', '25');  % Recommenden Tx_y
    set(handles.edit4,'String', '20');   % Recommenden Tx_z
    set(handles.edit8,'String', '80');% Recommenden Rx_x
    set(handles.edit9,'String', '75');  % Recommenden Rx_y
    set(handles.edit10,'String', '1');  % Recommenden Rx_z
    set(handles.edit13,'String', '70'); % Recommenden RIS_x
    set(handles.edit12,'String', '85'); % Recommenden RIS_y
    set(handles.edit11,'String', '10');  % Recommenden RIS_z
elseif   Environment==2 && Scenario==2
    set(handles.edit2,'String', '0');   % Recommenden Tx_x
    set(handles.edit3,'String', '25');  % Recommenden Tx_y
    set(handles.edit4,'String', '20');   % Recommenden Tx_z
    set(handles.edit8,'String', '70');% Recommenden Rx_x
    set(handles.edit9,'String', '65');  % Recommenden Rx_y
    set(handles.edit10,'String', '1');  % Recommenden Rx_z
    set(handles.edit13,'String', '85'); % Recommenden RIS_x
    set(handles.edit12,'String', '40'); % Recommenden RIS_y
    set(handles.edit11,'String', '10');  % Recommenden RIS_z
end

function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
function edit12_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function edit11_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)

function edit10_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)

function edit9_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)

function edit8_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)

function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)

function edit4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)

function popupmenu7_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% %%%%%%%%%%%%%%%
function [h,g,h_SISO]=SimRIS_v15(Environment,Scenario,Frequency,N,Tx_xyz,Rx_xyz,RIS_xyz)

% Inputs: Tx/Rx/RIS x-y-z Coordinates, Environment: Indoor/Outdoor (1/2), RIS Orientation (Scenario 1/2)
% Frequency (f),  Number of RIS Elements (N)

% Outputs: h, g, and h_SISO

% Environment=2;                     % 1 Indoor (InH - Indoor Office) / 2 Outdoor (UMi - Street Canyon)
% Scenario=1;                        % 1 (RIS in xz plane - left side wall) or 2 (RIS in yz plane - opposite wall)
% Frequency=28;                      % Operating Frequency (in GHz)
% N=64;                              % Number of RIS  Elements (sqrt(N) should be an integer)

lambda=(3*10^8)/(Frequency*10^9);  % Wavelength
k=2*pi/lambda;                     % Wavenumber
dis=lambda/2;                      % RIS element spacing (can be modified to observe its effect on h and g)
% Ge=pi;                           % Gain of RIS elements (maximum gain) - UPDATED VERSION 1.5

% Coordinates (with respect to reference (0,0,0) point (Scenario 1 - Indoors)
% Tx_xyz=[0,25,2];       % Tx coordinates (in meters)
% Rx_xyz=[38,48,1];     % Rx coordinates (in meters)
% RIS_xyz=[40,50,2];       % RIS coordinates (in meters)

% % for test (Scenario 1 - Indoors)
% Tx_xyz=[0,25,3];       % Tx coordinates (in meters)
% Rx_xyz=[70,45,0.75];     % Rx coordinates (in meters)
% RIS_xyz=[65,50,0.5];       % RIS coordinates (in meters)

% % for test (Scenario 2 - Indoors)
% Tx_xyz=[0,25,2];       % Tx coordinates (in meters)
% Rx_xyz=[70,30,1];     % Rx coordinates (in meters)
% RIS_xyz=[75,25,2];       % RIS coordinates (in meters)

% % for test (Scenario 1 - Outdoors)
% Tx_xyz=[0,25,30];       % Tx coordinates (in meters)
% Rx_xyz=[140,120,1];     % Rx coordinates (in meters)
% RIS_xyz=[120,150,10];       % RIS coordinates (in meters)

% % % for test (Scenario 2 - Outdoors)
% Tx_xyz=[0,25,30];       % Tx coordinates (in meters)
% Rx_xyz=[150,100,1];     % Rx coordinates (in meters)
% RIS_xyz=[180,40,10];       % RIS coordinates (in meters)

x_Tx=Tx_xyz(1);y_Tx=Tx_xyz(2);z_Tx=Tx_xyz(3);
x_Rx=Rx_xyz(1);y_Rx=Rx_xyz(2);z_Rx=Rx_xyz(3);
x_RIS=RIS_xyz(1);y_RIS=RIS_xyz(2);z_RIS=RIS_xyz(3);

% Give warnings (if RIS not on left side wall or opposite wall, if Tx is not on xz plane with x=0)
% Give warnings (if Rx is far away (max 8-10 meters) from RIS for indoors) - we need high LOS probability
% Give warnings for Outdoors (BS should be higher than the rest, RIS and Rx must not be too close)
% Give warnings for Indoors (if distances are too large - to avoid infinite loop - Critical error)
% Give warnings if far field condition is not satisfied (all distances > N*lambda/2)
% Give warnings if Frequency not equal to 28 or 73 (GHz)
% Give warnings (typical Tx height 2-3 meters for InH-Office, 3-20 meters for UMi-Street Canyon)
% Give warnings (typical cell radius max 75 meters for InH-Office, max 100 meters for UMi-Street Canyon)

% Give warning if N is not suitable for a square array
if mod(sqrt(N),1)~=0
    error('N should be an integer power of 2')
end

% figure;
% plot3(x_Tx,y_Tx,z_Tx,'ko');grid on;hold on;
% plot3(x_Rx,y_Rx,z_Rx,'kx');
% plot3(x_RIS,y_RIS,z_RIS,'ksquare')
% xlabel('x');ylabel('y');zlabel('z');
% text(x_Tx,y_Tx,z_Tx,'  Tx')
% text(x_Rx,y_Rx,z_Rx,'  Rx')
% text(x_RIS,y_RIS,z_RIS,'  RIS')

% if Environment==1
% axis([0 75 0 50 0 3.5]) % for indoors (room dimensions 5G)
% end

% BE CAREFUL WITH CHANNEL PARAMETERS
% PRE-DEFINE FOR ENVIROMENT 1 AND 2 (FOR LOS AND NLOS)
% From 5G Channel Model (2016) - Indoor Hotspot (InH) and Urban Microcellular (UMi)
% InH-Shopping Mall and UMi-Open Square can be added later

if Environment==1 % INDOORS
    % Environment 1 (InH - Office) - NLOS
    n_NLOS=3.19;          % Path Loss Exponent (Indoor Office NLOS)
    sigma_NLOS=8.29;      % Shadow Fading Term (dB) (Indoor Office NLOS)
    b_NLOS=0.06;          % Path Loss Parameter (Indoor Office NLOS)
    f0=24.2;              % Path Loss Parameter (GHz) (Indoor Office NLOS)
    
    % Environment 1 (InH - Office) - LOS
    n_LOS=1.73;           % Path Loss Exponent (Indoor Office LOS)
    sigma_LOS=3.02;       % Shadow Fading Term (dB) (Indoor Office LOS)
    b_LOS=0;              % Path Loss Parameter (Indoor Office NLOS)
    
else  % OUTDOORS
    % Enviroment 2 (UMi - Street Canyon) - NLOS
    n_NLOS=3.19;          % Path Loss Exponent (UMi Street Canyon NLOS)
    sigma_NLOS=8.2;       % Shadow Fading Term (dB) (UMi Street Canyon NLOS)
    b_NLOS=0;             % Path Loss Parameter (UMi Street Canyon NLOS)
    f0=24.2;              % Useless since b_NLOS=0
    
    % Enviroment 2 (UMi - Street Canyon) - LOS
    n_LOS=1.98;          % Path Loss Exponent (UMi Street Canyon LOS)
    sigma_LOS=3.1;       % Shadow Fading Term (dB) (UMi Street Canyon LOS)
    b_LOS=0;             % Path Loss Parameter (UMi Street Canyon LOS)
end

% Parameter of Number of Clusters
if Frequency==28
    lambda_p=1.8;
elseif Frequency==73
    lambda_p=1.9;
end

% Element Radiation Pattern Parameters (Version 1.5)
q=0.285;
Gain=pi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STEP 1
% Calculate Tx-RIS LOS distance  and Generate LOS Component for h (Version 1.4)
d_T_RIS = norm(Tx_xyz-RIS_xyz);    % or sqrt(sum((RIS_xyz-Tx_xyz).^2))

% LOS Probability is Relatively Low for Indoors if d_T_RIS > 20
if Environment==1
    if z_RIS<z_Tx   % for ground level RIS
        % InH LOS Probability
        if d_T_RIS<= 1.2
            p_LOS=1;
        elseif 1.2<d_T_RIS && d_T_RIS<6.5
            p_LOS=exp(-(d_T_RIS-1.2)/4.7);
        else
            p_LOS=0.32*exp(-(d_T_RIS-6.5)/32.6);
        end
        
        I_LOS=randsrc(1,1,[1,0;p_LOS 1-p_LOS]);
        
    elseif z_RIS>=z_Tx % for an RIS mounted at a high place (100% LOS)
        I_LOS=1;
    end
    
elseif Environment==2
    % UMi LOS Probability
    p_LOS=min([20/d_T_RIS,1])*(1-exp(-d_T_RIS/39)) + exp(-d_T_RIS/39);
    
    I_LOS=randsrc(1,1,[1,0;p_LOS 1-p_LOS]);
    
end



if I_LOS==1
    %%%%%%%%%%%%
    
    % Calculate RIS arrival angles to calculate array response vector
    if Scenario==1   % side-wall RIS
        
        I_phi=sign(x_RIS-x_Tx);
        phi_RIS_LOS = I_phi* atand ( abs( x_RIS-x_Tx) / abs(y_RIS-y_Tx) );
        
        I_theta=sign(z_Tx-z_RIS);
        theta_RIS_LOS=I_theta * asind ( abs (z_RIS-z_Tx )/ d_T_RIS );
        
    elseif Scenario==2 % opposite-wall RIS
        
        
        I_phi=sign(y_Tx-y_RIS);   % These are different from Scenario 1
        phi_RIS_LOS  = I_phi* atand ( abs(y_RIS-y_Tx ) / abs(  x_RIS-x_Tx ) );
        
        I_theta=sign(z_Tx-z_RIS);  % Same as Scenario 1
        theta_RIS_LOS=I_theta * asind ( abs (z_RIS-z_Tx )/ d_T_RIS );
               
    end
    % Array Response Calculation (LOS) (Be Careful with sind/cosd)
    array_LOS=zeros(1,N);
    
    counter3=1;
    for x=0:sqrt(N)-1
        for y=0:sqrt(N)-1
            
            array_LOS(counter3)=exp(1i*k*dis*(x*sind(theta_RIS_LOS) + y*sind(phi_RIS_LOS)*cosd(theta_RIS_LOS) )) ;
            counter3=counter3+1;
            
        end
    end
    
    
    % Link Attentuation (LOS)
    
    % Note: This is different than FSPL (Shadowing/Waveguiding effect included with n < 2)
    L_dB_LOS=-20*log10(4*pi/lambda) - 10*n_LOS*(1+b_LOS*((Frequency-f0)/f0))*log10(d_T_RIS)- randn*sigma_LOS;
    
    L_LOS=10^(L_dB_LOS/10);
    
    % LOS COMPONENT GENERATED (with random phase) (practical PL and shadowing)
    % Element radiation pattern considered (Version 1.5)
    h_LOS=sqrt(L_LOS)*transpose(array_LOS)*exp(1i*rand*2*pi)*sqrt(Gain*(cosd(theta_RIS_LOS))^(2*q));
    
    
    %%%%%%%%%%%%
else
    h_LOS=0;
end

% Generated h_LOS can be used for both environments.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 2
% Generate Clusters/Sub-rays, Azimuth/Elevation Departure Angles and Cluster Distances

for generate=1:100  % To ensure that at least one scatterer exist
    
    % Number of Clusters - lambda_p was defined earlier: 1.8/1.9
    C=max([1,poissrnd(lambda_p)]);  % Poisson distributed
    
    % Number of Sub-rays per Cluster
    S=randi(30,1,C);   
    % % for internal checks/tests (please comment)
    % C=1;
    % S=1;   
    % Azimuth/Elevation Departure Angles
    phi_Tx=[ ];
    theta_Tx=[ ];
    phi_av=zeros(1,C);
    theta_av=zeros(1,C); 
    for counter=1:C      
        phi_av(counter)  = rand*180-90;     % mean azimuth
        theta_av(counter)= rand*90-45;      % mean elevation  (needs update & most of the clusters are above ceiling)
        % for outdoors this might not be a big concern      
        % cluster angles: First S(1) belongs to Cluster 1, Next S(2) belongs to Cluster 2....
        phi_Tx   = [phi_Tx,log(rand(1,S(counter))./rand(1,S(counter)))*sqrt(25/2) + phi_av(counter)];
        theta_Tx = [theta_Tx,log(rand(1,S(counter))./rand(1,S(counter)))*sqrt(25/2) + theta_av(counter)];       
    end  
    % Cluster Distances
    a_c=1+rand(1,C)*(d_T_RIS-1);    % Cluster distances uniform [1,d_T_RIS] % can be modified later
    % Correction on Cluster Locations for Indoors
    if Environment ==1
        % Room dimensions (Indoor Hotspot)
        dim=[75,50,3.5];                  % x-y dimensions recommended by 5G Channel Model, height is assumed as 3.5 m      
        % Reducing Distances for Outside Clusters - Check Cluster Coordinates with Mean Angles       
        Coordinates=zeros(C,3);          % for Clusters
        Coordinates2=zeros(sum(S),3);    % for Scatterers       
        for counter=1:C
            loop=1;
            Coordinates(counter,:)=[x_Tx + a_c(counter)*cosd(theta_av(counter))*cosd(phi_av(counter)),...
                y_Tx - a_c(counter)*cosd(theta_av(counter))*sind(phi_av(counter)),...
                z_Tx + a_c(counter)*sind(theta_av(counter))] ;        
            while Coordinates(counter,3)>dim(3) || Coordinates(counter,3)<0 ||  Coordinates(counter,2)>dim(2) ||  Coordinates(counter,2)<0  ||  Coordinates(counter,1)>dim(1) ||  Coordinates(counter,1)<0
                a_c(counter)=    0.8*a_c(counter)  ;     % reduce distance 10%-20% if coordinates are not met!               
                % Note: While the above 10% reduction ensures that all clusters are in the range, to ensure that most of the scatterers are
                % in the range, we may consider 20% or 30% reduction. Scatter correction saves this issue.             
                Coordinates(counter,:)=[x_Tx + a_c(counter)*cosd(theta_av(counter))*cosd(phi_av(counter)),...
                    y_Tx - a_c(counter)*cosd(theta_av(counter))*sind(phi_av(counter)),...
                    z_Tx + a_c(counter)*sind(theta_av(counter))] ;
                %    loop=loop+1               
            end         
            %     % Plot Clusters ( comment figures in mass generation of channels )
            % plot3(Coordinates(counter,1),Coordinates(counter,2),Coordinates(counter,3),'mdiamond')
            % hold on;          
        end       
    elseif Environment==2 % Outdoors       
        % Reducing Distances for Outside Clusters - Check Cluster Coordinates with Mean Angles        
        Coordinates=zeros(C,3);          % for Clusters
        Coordinates2=zeros(sum(S),3);    % for Scatterers        
        for counter=1:C
            loop=1;
            Coordinates(counter,:)=[x_Tx + a_c(counter)*cosd(theta_av(counter))*cosd(phi_av(counter)),...
                y_Tx - a_c(counter)*cosd(theta_av(counter))*sind(phi_av(counter)),...
                z_Tx + a_c(counter)*sind(theta_av(counter))] ;          
            while  Coordinates(counter,3)<0    % only underground clusters will be ignored
                a_c(counter)=    0.8*a_c(counter)  ;     % reduce distance 10%-20% if coordinates are not met!               
                % Note: While the above 10% reduction ensures that all clusters are in the range, to ensure that most of the scatterers are
                % in the range, we may consider 20% or 30% reduction.               
                Coordinates(counter,:)=[x_Tx + a_c(counter)*cosd(theta_av(counter))*cosd(phi_av(counter)),...
                    y_Tx - a_c(counter)*cosd(theta_av(counter))*sind(phi_av(counter)),...
                    z_Tx + a_c(counter)*sind(theta_av(counter))] ;
                %    loop=loop+1                
            end          
            %     % Plot Clusters (comment figures in mass generation of channels (with repeat))
            % plot3(Coordinates(counter,1),Coordinates(counter,2),Coordinates(counter,3),'mdiamond')
            % hold on;          
        end      
    end       
    % Plot Scatterers
    a_c_rep=[];
    for counter3=1:C
        a_c_rep=[a_c_rep,repmat(a_c(counter3),1,S(counter3))];
    end    
    for counter2=1:sum(S)
        Coordinates2(counter2,:)=[x_Tx + a_c_rep(counter2)*cosd(theta_Tx(counter2))*cosd(phi_Tx(counter2)),...
            y_Tx - a_c_rep(counter2)*cosd(theta_Tx(counter2))*sind(phi_Tx(counter2)),...
            z_Tx + a_c_rep(counter2)*sind(theta_Tx(counter2))] ;
        
        %       % comment figures in mass generation of channels
        %     plot3(Coordinates2(counter2,1),Coordinates2(counter2,2),Coordinates2(counter2,3),'m.')
        %     hold on;
    end
    
    
    
    % Correction on Scatters
    % You may ignore the scatterers outside the walls for Enviroment 1 (Indoors) or underground for Enviroment 2 (Outdoors)
    
    if Environment==1
        ignore=[];
        
        for counter2=1:sum(S)
            if Coordinates2(counter2,3)>dim(3) || Coordinates2(counter2,3)<0 ||  Coordinates2(counter2,2)>dim(2) ||  Coordinates2(counter2,2)<0  ||  Coordinates2(counter2,1)>dim(1) ||  Coordinates2(counter2,1)<0
                ignore=[ignore,counter2];   % contains the indices of scatterers that can be ignored
            end
        end
        
        % updated indices
        indices=setdiff(1:sum(S),ignore);    % the set of active scatterer indices
        M_new=length(indices);               % number of IOs inside the room or above ground
        
        
        
    elseif Environment==2
        
        ignore=[];
        
        for counter2=1:sum(S)
            if  Coordinates2(counter2,3)<0   % only underground scatterers
                ignore=[ignore,counter2];   % contains the indices of scatterers that can be ignored
            end
        end
        
        % updated indices
        indices=setdiff(1:sum(S),ignore);    % the set of active scatterer indices
        M_new=length(indices);               % number of IOs inside the room or above ground
        
    end
    
    % if you want to revert back Version 1.1 from Version 1.2, simply set
    % indices=1:sum(S);
    % M_new=sum(S);
    
    % Neccessary Loop to have at least one scatterer
    if M_new>0 % if M_new==0 --> all scatters are outside
        break  % break generate=1:100 % if M_new >0 we are OK (at least one scatter)
    end
    
end  % for generate=1:100
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 3 (Common for Enviroments 1 and 2)
% Calculate Arrival Angles for the RIS and the Link Distances

phi_RIS=zeros(1,sum(S));
theta_RIS=zeros(1,sum(S));
b_cs=zeros(1,sum(S));
d_cs=zeros(1,sum(S));

if Scenario==1   % side-wall RIS
    
    for counter2=indices
        
        b_cs(counter2)=norm(RIS_xyz-Coordinates2(counter2,:));   % Distance between Scatterer and RIS
        d_cs(counter2)=a_c_rep(counter2)+b_cs(counter2);         % Total distance Tx-Scatterer-RIS
        
        I_phi=sign(x_RIS-Coordinates2(counter2,1));
        phi_RIS(counter2)  = I_phi* atand ( abs( x_RIS-Coordinates2(counter2,1)) / abs(y_RIS-Coordinates2(counter2,2)) );
        
        I_theta=sign(Coordinates2(counter2,3)-z_RIS);
        theta_RIS(counter2)=I_theta * asind ( abs (z_RIS-Coordinates2(counter2,3) )/ b_cs(counter2) );
        
    end
    
elseif Scenario==2 % opposite-wall RIS
    
    for counter2=indices
        
        b_cs(counter2)=norm(RIS_xyz-Coordinates2(counter2,:));   % Distance between Scatterer and RIS
        d_cs(counter2)=a_c_rep(counter2)+b_cs(counter2);         % Total distance Tx-Scatterer-RIS
        
        I_phi=sign(Coordinates2(counter2,2)-y_RIS);   % These are different from Scenario 1
        phi_RIS(counter2)  = I_phi* atand ( abs(y_RIS-Coordinates2(counter2,2) ) / abs(  x_RIS-Coordinates2(counter2,1) ) );
        
        I_theta=sign(Coordinates2(counter2,3)-z_RIS);  % the same as Scenario 1
        theta_RIS(counter2)=I_theta * asind ( abs (z_RIS-Coordinates2(counter2,3) )/ b_cs(counter2) );
        
    end  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 4 (Common for Enviroments 1 and 2)
% Array Response Calculation
array_1=zeros(sum(S),N);
for counter2=indices 
    counter3=1;
    for x=0:sqrt(N)-1
        for y=0:sqrt(N)-1            
            array_1(counter2,counter3)=exp(1i*k*dis*(x*sind(theta_RIS(counter2)) + y*sind(phi_RIS(counter2))*cosd(theta_RIS(counter2)) )) ;
            counter3=counter3+1;            
        end
    end    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 5 (Common for Enviroments 1 and 2)
% Calculate Link Attenuation and Generate Tx-RIS Channel (h) using 5G Channel Model
% Ge Updated (Version 1.5)
h_NLOS=zeros(N,1);
beta=zeros(1,sum(S)); % to be reused for shared clusters (Environment 1)
shadow=beta;          % to be reused for shared clusters (Environment 1) - Version 1.4
for counter2=indices   
    X_sigma=randn*sigma_NLOS;    
    Lcs_dB=-20*log10(4*pi/lambda) - 10*n_NLOS*(1+b_NLOS*((Frequency-f0)/f0))*log10(d_cs(counter2))- X_sigma;    
    Lcs=10^(Lcs_dB/10);    
    beta(counter2)=((randn+1i*randn)./sqrt(2));  % common complex gain for shared clusters    
    shadow(counter2)=X_sigma;                    % commun shadow factor for shared clusters    
    h_NLOS = h_NLOS + beta(counter2)*sqrt(Gain*(cosd(theta_RIS(counter2)))^(2*q))*sqrt(Lcs)*transpose(array_1(counter2,:));  % consider all scatters
end
h_NLOS=h_NLOS.*sqrt(1/M_new);  % normalization
h=h_NLOS+h_LOS; % include the LOS component (if any) h_LOS=0 when there is no LOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEPS 6-7
% Generation of g (RIS-Rx Channel)
if Environment==1   % GENERATE A LOS CHANNEL (modified in Version 1.4)
    % Version 1.3 (and older) uses the pure LOS channel
    % Version 1.4 uses the 5G LOS component with Practical Path Loss (~1-3 dB less path loss)
    % Version 1.5 ignores Ge^2 in g and includes the element radiation pattern   
    % Calculate Departure Angles Considering RIS and Rx Coordinates
    d_RIS_R=norm(RIS_xyz-Rx_xyz);
    % Elevation Departure Angle
    I_theta=sign(z_Rx - z_RIS);
    theta_Rx_RIS=I_theta * asind( abs(z_Rx-z_RIS)/d_RIS_R );    
    % Azimuth Departure Angle
    if Scenario==1       
        I_phi=sign(x_RIS - x_Rx);
        phi_Rx_RIS=I_phi * atand( abs(x_Rx-x_RIS)/ abs(y_Rx-y_RIS) );       
    elseif Scenario==2        
        I_phi=sign(y_Rx- y_RIS);
        phi_Rx_RIS=I_phi * atand( abs(y_Rx-y_RIS)/ abs(x_Rx-x_RIS) );       
    end  
    % Recalculate Array Response for Two Angles (in Rx direction)
    array_2=zeros(1,N);    
    counter3=1;
    for x=0:sqrt(N)-1
        for y=0:sqrt(N)-1
            
            array_2(counter3)=exp(1i*k*dis*(x*sind(theta_Rx_RIS) + y*sind(phi_Rx_RIS)*cosd(theta_Rx_RIS) )) ;
            counter3=counter3+1;
            
        end
    end
    
    
    % % LOS Link Attenuation (Version 1.3)
    % Ge=pi;
    % L_LOS=(Ge^2 * lambda^2)  / ((4*pi*d_RIS_R)^2);
    
    L_dB_LOS_2=-20*log10(4*pi/lambda) - 10*n_LOS*(1+b_LOS*((Frequency-f0)/f0))*log10(d_RIS_R)- randn*sigma_LOS;
    
    L_LOS_2=10^(L_dB_LOS_2/10);
    
    
    % Generate g (Pure LOS)
    % g=Ge*sqrt(L_LOS_2)*transpose(array_2)*exp(1i*rand*2*pi);
    g=sqrt(Gain*(cosd(theta_Rx_RIS))^(2*q))*sqrt(L_LOS_2)*transpose(array_2)*exp(1i*rand*2*pi);   % Version 1.5
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % GENERATION OF g FOR OUTDOORS (Involves STEPS 1-5)
    % including a LOS component (Version 1.4)
    
    % For simplicity in algorithm, we use "_2" for this link compared to Tx-RIS
    
elseif Environment==2
    
    d_RIS_R=norm(RIS_xyz-Rx_xyz);
    
    %%%%%%%%%%%%%%%% LOS COMPONENT CALCULATION (Version 1.4) from Step 1
    
    % UMi LOS Probability
    p_LOS_2=min([20/d_RIS_R,1])*(1-exp(-d_RIS_R/39)) + exp(-d_RIS_R/39);
    
    I_LOS_2=randsrc(1,1,[1,0;p_LOS_2 1-p_LOS_2]);
    
    
    if I_LOS_2==1
        %%%%%%%%%%%%
        if Scenario==1   % side-wall RIS
            
            
            I_phi=sign(x_RIS-x_Rx);
            phi_RIS_LOS_2 = I_phi* atand ( abs( x_RIS-x_Rx) / abs(y_RIS-y_Rx) );
            
            I_theta=sign(z_Rx-z_RIS);
            theta_RIS_LOS_2=I_theta * asind ( abs (z_RIS-z_Rx )/ d_RIS_R );
            
            
        elseif Scenario==2 % opposite-wall RIS
            
            
            I_phi=sign(y_Rx-y_RIS);   % These are different from Scenario 1
            phi_RIS_LOS_2  = I_phi* atand ( abs(y_RIS-y_Rx ) / abs(  x_RIS-x_Rx ) );
            
            I_theta=sign(z_Rx-z_RIS);  % Same as Scenario 1
            theta_RIS_LOS_2=I_theta * asind ( abs (z_RIS-z_Rx )/ d_RIS_R );
            
            
        end
        
        
        % Array Response Calculation (LOS)
        array_LOS_2=zeros(1,N);
        
        
        
        counter3=1;
        for x=0:sqrt(N)-1
            for y=0:sqrt(N)-1
                
                array_LOS_2(counter3)=exp(1i*k*dis*(x*sind(theta_RIS_LOS_2) + y*sind(phi_RIS_LOS_2)*cosd(theta_RIS_LOS_2) )) ;
                counter3=counter3+1;
                
            end
        end
        
        
        % Link Attentuation (LOS)
        
        
        
        L_dB_LOS_2=-20*log10(4*pi/lambda) - 10*n_LOS*(1+b_LOS*((Frequency-f0)/f0))*log10(d_RIS_R)- randn*sigma_LOS;
        
        L_LOS_2=10^(L_dB_LOS_2/10);
        
        % LOS COMPONENT GENERATED
        g_LOS=sqrt(L_LOS_2)*transpose(array_LOS_2)*exp(1i*rand*2*pi)*sqrt(Gain*(cosd(theta_RIS_LOS_2))^(2*q));
        
        
        %%%%%%%%%%%%
    else
        g_LOS=0;
    end
    
    
    
    
    
    %%%%%%%%%%%%%%%%
    
    
    % Generate New Clusters/Scatters (Step 2) - Ensure that All Scattters are above ground
    for generate2=1:100  % To ensure that at least one scatterer exist
        
        % lambda_p was defined before for 28/73 GHz
        C_2=max([1,poissrnd(lambda_p)]);  % Poisson distributed
        
        % Number of Sub-rays per Cluster
        S_2=randi(30,1,C_2);
        
        
        % Azimuth/Elevation Departure Angles
        phi_Tx_2=[ ];
        theta_Tx_2=[ ];
        phi_av_2=zeros(1,C_2);
        theta_av_2=zeros(1,C_2);
        
        for counter=1:C_2
            
            phi_av_2(counter)  = rand*90-45;   % mean azimuth (reduced to ensure that all scatters are within the field of view)
            theta_av_2(counter)= rand*90-45; % mean elevation  (needs update & most of the clusters are above ceiling)
            % for outdoors this might not be a big concern
            
            % cluster angles: First S(1) belongs to Cluster 1, Next S(2) belongs to Cluster 2....
            phi_Tx_2   = [phi_Tx_2,  log(rand(1,S_2(counter))./rand(1,S_2(counter)))*sqrt(25/2) + phi_av_2(counter)];
            theta_Tx_2 = [theta_Tx_2,log(rand(1,S_2(counter))./rand(1,S_2(counter)))*sqrt(25/2) + theta_av_2(counter)];
            
        end
        
        
        
        % Cluster Distances
        a_c_2=1+rand(1,C_2)*(d_RIS_R-1);    % Cluster distances uniform [1,d_RIS_R] % can be modified later
        
        
        % Reducing Distances for Outside Clusters - Check Cluster Coordinates with Mean Angles
        
        Coordinates_2=zeros(C_2,3);          % for Clusters
        Coordinates2_2=zeros(sum(S_2),3);    % for Scatterers
        
        for counter=1:C_2
            loop=1;
            if Scenario==1       % CHECK !!!
                Coordinates_2(counter,:)=[x_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*sind(phi_av_2(counter)),...
                    y_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*cosd(phi_av_2(counter)),...
                    z_RIS + a_c_2(counter)*sind(theta_av_2(counter))];
                
            elseif Scenario==2   % CHECK !!!
                Coordinates_2(counter,:)=[x_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*cosd(phi_av_2(counter)),...
                    y_RIS + a_c_2(counter)*cosd(theta_av_2(counter))*sind(phi_av_2(counter)),...
                    z_RIS + a_c_2(counter)*sind(theta_av_2(counter))];
                
            end
            
            
            
            while  Coordinates_2(counter,3)<0    % only underground clusters will be ignored
                a_c_2(counter)=    0.8*a_c_2(counter)  ;     % reduce distance 10%-20% if coordinates are not met!
                
                % Note: While the above 10% reduction ensures that all clusters are in the range, to ensure that most of the scatterers are
                % in the range, we may consider 20% or 30% reduction.
                
                if Scenario==1       % CHECK !!!
                    Coordinates_2(counter,:)=[x_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*sind(phi_av_2(counter)),...
                        y_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*cosd(phi_av_2(counter)),...
                        z_RIS + a_c_2(counter)*sind(theta_av_2(counter))];
                    
                elseif Scenario==2   % CHECK !!!
                    Coordinates_2(counter,:)=[x_RIS - a_c_2(counter)*cosd(theta_av_2(counter))*cosd(phi_av_2(counter)),...
                        y_RIS + a_c_2(counter)*cosd(theta_av_2(counter))*sind(phi_av_2(counter)),...
                        z_RIS + a_c_2(counter)*sind(theta_av_2(counter))];
                    
                end
                %    loop=loop+1
                
            end
            
            %     % Plot Clusters (comment figures in mass generation of channels (with repeat))
            % plot3(Coordinates_2(counter,1),Coordinates_2(counter,2),Coordinates_2(counter,3),'bdiamond')
            % hold on;
            
            
            
        end
        
        
        % Plot Scatterers
        a_c_rep_2=[];
        for counter3=1:C_2
            a_c_rep_2=[a_c_rep_2,repmat(a_c_2(counter3),1,S_2(counter3))];
        end
        
        
        for counter2=1:sum(S_2)
            
            
            if Scenario==1       % CHECK !!!
                Coordinates2_2(counter2,:)=[x_RIS - a_c_rep_2(counter2)*cosd(theta_Tx_2(counter2))*sind(phi_Tx_2(counter2)),...
                    y_RIS - a_c_rep_2(counter2)*cosd(theta_Tx_2(counter2))*cosd(phi_Tx_2(counter2)),...
                    z_RIS + a_c_rep_2(counter2)*sind(theta_Tx_2(counter2))];
                
            elseif Scenario==2   % CHECK !!!
                Coordinates2_2(counter2,:)=[x_RIS - a_c_rep_2(counter2)*cosd(theta_Tx_2(counter2))*cosd(phi_Tx_2(counter2)),...
                    y_RIS + a_c_rep_2(counter2)*cosd(theta_Tx_2(counter2))*sind(phi_Tx_2(counter2)),...
                    z_RIS + a_c_rep_2(counter2)*sind(theta_Tx_2(counter2))];
                
            end
            
            
            %       % comment figures in mass generation of channels (with repeat)
            %     plot3(Coordinates2_2(counter2,1),Coordinates2_2(counter2,2),Coordinates2_2(counter2,3),'b.')
            %   hold on;
            
        end
        
        
        
        % Correction on Scatters
        
        ignore_2=[];
        
        for counter2=1:sum(S_2)
            if  Coordinates2_2(counter2,3)<0   % only underground scatterers
                ignore_2=[ignore_2,counter2];   % contains the indices of scatterers that can be ignored
            end
        end
        
        % updated indices
        indices_2=setdiff(1:sum(S_2),ignore_2);    % the set of active scatterer indices
        M_new_2=length(indices_2);               % number of IOs inside the room or above ground
        
        
        % Neccessary Loop to have at least one scatterer
        if M_new_2>0 % all scatters are outside
            break  % break generate=1:100
        end
        
    end  % for generate=1:100
    
    
    % Calculate Array Response (Step 4)
    % STEP 4
    % Array Response Calculation
    array_2=zeros(sum(S_2),N);
    
    for counter2=indices_2
        
        counter3=1;
        for x=0:sqrt(N)-1
            for y=0:sqrt(N)-1
                
                array_2(counter2,counter3)=exp(1i*k*dis*(x*sind(theta_Tx_2(counter2)) + y*sind(phi_Tx_2(counter2))*cosd(theta_Tx_2(counter2)) )) ;
                counter3=counter3+1;
                
            end
        end
        
        
    end
    
    
    % Calculate Link Lengths (Step 3) and Path Loss (Step 5)
    b_cs_2=zeros(1,sum(S_2));
    d_cs_2=zeros(1,sum(S_2));
    
    
    for counter2=indices_2
        
        b_cs_2(counter2)=norm(Rx_xyz-Coordinates2_2(counter2,:));   % Distance between Scatterer and Rx
        d_cs_2(counter2)=a_c_rep_2(counter2)+b_cs_2(counter2);         % Total distance RIS-Scatterer-Rx
        
    end
    
    
    % Generate g (Step 5)
    g_NLOS=zeros(N,1);
    
    for counter2=indices_2
        
        X_sigma_2=randn*sigma_NLOS;
        
        Lcs_dB_2=-20*log10(4*pi/lambda) - 10*n_NLOS*(1+b_NLOS*((Frequency-f0)/f0))*log10(d_cs_2(counter2))- X_sigma_2;
        
        Lcs_2=10^(Lcs_dB_2/10);
        
        beta_2=((randn+1i*randn)./sqrt(2));
        
        g_NLOS = g_NLOS + beta_2*sqrt(Lcs_2)*sqrt(Gain*(cosd(theta_Tx_2(counter2)))^(2*q))*transpose(array_2(counter2,:));   % gain of RIS elements included/updated
    end
    
    g_NLOS=g_NLOS.*sqrt(1/M_new_2);  % normalization
    
    % for Environment 2 only (Enviroment 1 is already LOS)
    g=g_NLOS+g_LOS;
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 8
% Generation of h_SISO

if Environment==1  % WITH SHARED CLUSTERS AND COMMON LOS
    
    d_T_R=norm(Tx_xyz-Rx_xyz);
    
    d_cs_tilde=zeros(1,sum(S));
    h_SISO_NLOS=0;
    
    for counter2=indices
        
        % due to shared clusters d_cs_tilde ~ d_cs
        d_cs_tilde(counter2) = a_c_rep(counter2) + norm(Coordinates2(counter2,:)- Rx_xyz);
        
        %    X_sigma=randn*sigma; SHARED CLUSTERS (same Shadow Factor - Version 1.4 Updated)
        
        Lcs_dB_SISO=-20*log10(4*pi/lambda) - 10*n_NLOS*(1+b_NLOS*((Frequency-f0)/f0))*log10(d_cs_tilde(counter2))- shadow(counter2);
        
        Lcs_SISO=10^(Lcs_dB_SISO/10);
        
        % We consider the same complex path gain (small scale fading) with an excess phase (similar to array response)
        eta=k* ( norm(Coordinates2(counter2,:)- RIS_xyz) -  norm(Coordinates2(counter2,:)- Rx_xyz));
        
        h_SISO_NLOS = h_SISO_NLOS + beta(counter2)*exp(1i*eta)*sqrt(Lcs_SISO);
    end
    
    h_SISO_NLOS=h_SISO_NLOS.*sqrt(1/M_new);  % normalization
    
    if z_RIS >= z_Tx
        
        % % Include LOS component (Version 1.4)
        %     % InH LOS Probability
        if d_T_R<= 1.2
            p_LOS_3=1;
        elseif 1.2<d_T_R && d_T_R<6.5
            p_LOS_3=exp(-(d_T_R-1.2)/4.7);
        else
            p_LOS_3=0.32*exp(-(d_T_R-6.5)/32.6);
        end
        
        I_LOS_3=randsrc(1,1,[1,0;p_LOS_3 1-p_LOS_3]);
        
        % Do not recalculate, if T-RIS has LOS, we might have LOS for h_SISO as well (for ground level RIS)
        % If we have LOS for Tx-RIS, we have LOS for Tx-Rx
    elseif z_RIS < z_Tx  % RIS in the ground level
        
        I_LOS_3=I_LOS;
        
    end
    
    if I_LOS_3==1        
        L_SISO_LOS_dB=-20*log10(4*pi/lambda) - 10*n_LOS*(1+b_LOS*((Frequency-f0)/f0))*log10(d_T_R)- randn*sigma_LOS;
        L_SISO_LOS=10^(L_SISO_LOS_dB/10);        
        h_SISO_LOS= sqrt(L_SISO_LOS)*exp(1i*rand*2*pi);        
    else        
        h_SISO_LOS=0;        
    end
    
    h_SISO=h_SISO_NLOS + h_SISO_LOS; % include LOS Component if I_LOS_3==1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif Environment==2 % WITH NEW CLUSTERS
    
    % Generate New Clusters/Scatterers (Outdoors)
    d_T_R=norm(Tx_xyz-Rx_xyz);   
    % Calculate LOS Component (Version 1.4)
    % UMi LOS Probability
    p_LOS_3=min([20/d_T_R,1])*(1-exp(-d_T_R/39)) + exp(-d_T_R/39);
    I_LOS_3=randsrc(1,1,[1,0;p_LOS_3 1-p_LOS_3]);
    if I_LOS_3==1
        L_SISO_LOS_dB=-20*log10(4*pi/lambda) - 10*n_LOS*(1+b_LOS*((Frequency-f0)/f0))*log10(d_T_R)- randn*sigma_LOS;
        L_SISO_LOS=10^(L_SISO_LOS_dB/10);        
        h_SISO_LOS= sqrt(L_SISO_LOS)*exp(1i*rand*2*pi);   % LOS Component for the Tx-Rx link        
    else
        h_SISO_LOS=0;
    end    
    % Generate New Clusters/Scatters (Step 2) - Ensure that All Scattters are above ground
    for generate3=1:100  % To ensure that at least one scatterer exist 
        % lambda_p was defined before for 28/73 GHz
        C_3=max([1,poissrnd(lambda_p)]);  % Poisson distributed
        % Number of Sub-rays per Cluster
        S_3=randi(30,1,C_3);        
        % Azimuth/Elevation Departure Angles
        phi_Tx_3=[ ];
        theta_Tx_3=[ ];
        phi_av_3=zeros(1,C_3);
        theta_av_3=zeros(1,C_3);        
        for counter=1:C_3            
            phi_av_3(counter)  = rand*180-90;   % mean azimuth
            theta_av_3(counter)= rand*90-45; % mean elevation  (needs update & most of the clusters are above ceiling)
            % for outdoors this might not be a big concern            
            % cluster angles: First S(1) belongs to Cluster 1, Next S(2) belongs to Cluster 2....
            phi_Tx_3   = [phi_Tx_3,  log(rand(1,S_3(counter))./rand(1,S_3(counter)))*sqrt(25/2) + phi_av_3(counter)];
            theta_Tx_3 = [theta_Tx_3,log(rand(1,S_3(counter))./rand(1,S_3(counter)))*sqrt(25/2) + theta_av_3(counter)];            
        end
        % Cluster Distances
        a_c_3=1+rand(1,C_3)*(d_T_R-1);    % Cluster distances uniform [1,d_T_R]  
        % Reducing Distances for Outside Clusters - Check Cluster Coordinates with Mean Angles  
        Coordinates_3=zeros(C_3,3);          % for Clusters
        Coordinates2_3=zeros(sum(S_3),3);    % for Scatterers
        for counter=1:C_3
            loop=1;            
            Coordinates_3(counter,:)=[x_Tx + a_c_3(counter)*cosd(theta_av_3(counter))*cosd(phi_av_3(counter)),...
                y_Tx - a_c_3(counter)*cosd(theta_av_3(counter))*sind(phi_av_3(counter)),...
                z_Tx + a_c_3(counter)*sind(theta_av_3(counter))] ;                                   
            while  Coordinates_3(counter,3)<0    % only underground clusters will be ignored
                a_c_3(counter)=    0.8*a_c_3(counter)  ;     % reduce distance 10%-20% if coordinates are not met!                
                % Note: While the above 10% reduction ensures that all clusters are in the range, to ensure that most of the scatterers are
                % in the range, we may consider 20% or 30% reduction.
                
                Coordinates_3(counter,:)=[x_Tx + a_c_3(counter)*cosd(theta_av_3(counter))*cosd(phi_av_3(counter)),...
                    y_Tx - a_c_3(counter)*cosd(theta_av_3(counter))*sind(phi_av_3(counter)),...
                    z_Tx + a_c_3(counter)*sind(theta_av_3(counter))] ;
                %    loop=loop+1                
            end            
            %     % Plot Clusters (comment figures in mass generation of channels (with repeat))
            % plot3(Coordinates_3(counter,1),Coordinates_3(counter,2),Coordinates_3(counter,3),'rdiamond')
            % hold on;         
        end       
        % Plot Scatterers
        a_c_rep_3=[];
        for counter3=1:C_3
            a_c_rep_3=[a_c_rep_3,repmat(a_c_3(counter3),1,S_3(counter3))];
        end
        for counter2=1:sum(S_3)      
            Coordinates2_3(counter2,:)=[x_Tx + a_c_rep_3(counter2)*cosd(theta_Tx_3(counter2))*cosd(phi_Tx_3(counter2)),...
                y_Tx - a_c_rep_3(counter2)*cosd(theta_Tx_3(counter2))*sind(phi_Tx_3(counter2)),...
                z_Tx + a_c_rep_3(counter2)*sind(theta_Tx_3(counter2))] ;                        
            %       % comment figures in mass generation of channels (with repeat)
            %     plot3(Coordinates2_3(counter2,1),Coordinates2_3(counter2,2),Coordinates2_3(counter2,3),'r.')
            %   hold on;           
        end       
        % Correction on Scatters        
        ignore_3=[];        
        for counter2=1:sum(S_3)
            if  Coordinates2_3(counter2,3)<0   % only underground scatterers
                ignore_3=[ignore_3,counter2];   % contains the indices of scatterers that can be ignored
            end
        end        
        % updated indices
        indices_3=setdiff(1:sum(S_3),ignore_3);    % the set of active scatterer indices
        M_new_3=length(indices_3);               % number of IOs inside the room or above ground       
        % Neccessary Loop to have at least one scatterer
        if M_new_3>0 % all scatters are outside
            break  % break generate=1:100
        end        
    end  % for generate=1:100 
    % Calculate Link Lengths and Path Loss
    b_cs_3=zeros(1,sum(S_3));
    d_cs_3=zeros(1,sum(S_3));       
    for counter2=indices_3        
        b_cs_3(counter2)=norm(Tx_xyz-Coordinates2_3(counter2,:));      % Distance between Scatterer and Tx
        d_cs_3(counter2)=a_c_rep_3(counter2)+b_cs_3(counter2);         % Total distance Tx-Scatterer-Rx       
    end    
    % Generate h_SISO    
    h_SISO_NLOS=0;    
    for counter2=indices_3        
        X_sigma_3=randn*sigma_NLOS;        
        Lcs_dB_SISO_3=-20*log10(4*pi/lambda) - 10*n_NLOS*(1+b_NLOS*((Frequency-f0)/f0))*log10(d_cs_3(counter2))- X_sigma_3;        
        Lcs_SISO_3=10^(Lcs_dB_SISO_3/10);        
        h_SISO_NLOS = h_SISO_NLOS + ((randn+1i*randn)/sqrt(2))*sqrt(Lcs_SISO_3);
    end    
    h_SISO_NLOS = h_SISO_NLOS.*sqrt(1/M_new_3);  % normalization    
    h_SISO = h_SISO_NLOS + h_SISO_LOS;    % h_SISO_LOS=0  if there is no LOS    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the END :-)


% % --- Executes during object creation, after setting all properties.
% function axes2_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to axes2 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% % a=imread('corelab_saydam.png');
% % imshow(a)
% % This creates the 'background' axes
% ha = axes('units','normalized', ...
%             'position',[0 0 1 1]);
% % Move the background axes to the bottom
% uistack(ha,'bottom');
% % Load in a background image and display it using the correct colors
% % The image used below, is in the Image Processing Toolbox.  If you do not have %access to this toolbox, you can use another image file instead.
% % I=imread('corelab_saydam.png');
% I=imread('backg.jpg');
% 
% hi = imagesc(I)
% colormap gray
% % Turn the handlevisibility off so that we don't inadvertently plot into the axes again
% % Also, make the axes invisible
% set(ha,'handlevisibility','off', ...
%             'visible','off')
% % Now we can use the figure, as required.
% % For example, we can put a plot in an axes
% % axes('position',[0.3,0.35,0.4,0.4])
% 
% % Hint: place code in OpeningFcn to populate axes2
% 

% --- Executes during object creation, after setting all properties.

% Hint: place code in OpeningFcn to populate axes3



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dos('explorer http://corelab.ku.edu.tr');  
% --- Executes during object creation, after setting all properties.
function axes5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes5
% I=imread('corelab_beyaz.jpg');
imagesc(imread('corelab_beyaz.jpg'))
% axis off
% axis image


% % --- Executes during object creation, after setting all properties.
% function axes6_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to axes6 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: place code in OpeningFcn to populate axes6
% % % I=imread('corelab_beyaz.jpg');
% imagesc(imread('corelab_beyaz.jpg'))
% axis off
% axis image


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);  % Update!
h=handles.channel_h;
g= handles.channel_g;
h_SISO =  handles.channel_h_SISO;

uisave({'h','g','h_SISO'},'RIS_Channels')
%  handles.channel_h = h;
%  handles.channel_g = g;
%  handles.channel_h_SISO = h_SISO;


% % --- Executes during object creation, after setting all properties.
% function axes7_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to axes7 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: place code in OpeningFcn to populate axes7
% % Hint: place code in OpeningFcn to populate axes6
% % % I=imread('corelab_beyaz.jpg');
% imshow(imread('corelab_beyaz.jpg'))
% axis off
% axis image
% 
% 
% % --- Executes during object creation, after setting all properties.
% 
% % --- Executes during object creation, after setting all properties.
% function axes9_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to axes9 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: place code in OpeningFcn to populate axes9
% I = imread('corelab_beyaz.jpg');
% imshow(I)
% % imshow(imread('corelab_beyaz.jpg'),'InitialMagnification',100) % 100 percent
% axis off
% axis image


% --- Executes during object creation, after setting all properties.
function axes10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes10
I = imread('ku3.jpg');
imshow(I)
% imshow(imread('corelab_beyaz.jpg'),'InitialMagnification',100) % 100 percent
axis off
axis image


% --- Executes during object creation, after setting all properties.
function axes13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes11
I = imread('ku3.jpg');
imshow(I)
% imshow(imread('corelab_beyaz.jpg'),'InitialMagnification',100) % 100 percent
axis off
axis image


% --- Executes during object creation, after setting all properties.
function axes14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes12
I = imread('corelab_beyaz.jpg');
imshow(I)
% imshow(imread('corelab_beyaz.jpg'),'InitialMagnification',100) % 100 percent
axis off
axis image
