function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises
%      the existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 11-Oct-2013 13:17:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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

% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

initialize_gui(hObject, handles, false);

% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.xpc_gui);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% refresh default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function initialize_gui(fig_handle, handles, isreset)


function PopulateTableValues(table)
tg = evalin('base','tg');
if ~exist('tg')
    errordlg('Target machine variable ''tg'' not found!','Error');
    return;
end

% Get table data
data = get(table, 'Data');

%Loop through parameters and load values
%[block,param] = tg.getparamname(index); to get param info
for i=1:size(data,1)
    try
        value = getparam(tg, getparamid(tg,data{i,1},data{i,2}));
        data{i,3} = value(1); % does not work for matrix values
    catch err
        %disp(sprintf('Failed to get value for %s\n',data{i,1}));
    end
    if any(size(data{i,3}) ~= [1 1])
        %disp(sprintf('Failed to get value for %s\n',data{i,1}));
        data{i,2} = 0;
    end
end
set(table, 'Data', data);


% --- Executes on button press in refresh.
function refresh_Callback(hObject, eventdata, handles)
% hObject    handle to refresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PopulateTableValues(handles.parameter_table);


% --- Executes during object creation, after setting all properties.
function parameter_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to parameter_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
%data = cell(27,2);
data = {'','dx_avg_desired',0
        '','k_placement',0
        '','kp_motor',0
        '','kd_motor',0
        '','kp_sea_torque',0
        '','k_ff',0
        '','alpha_apex_modifier',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        '','',0
        };
set(hObject, 'Data', data);
PopulateTableValues(hObject);


% --- Executes when entered data in editable cell(s) in parameter_table.
function parameter_table_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to parameter_table (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% Ignore edits to first and second columns
if eventdata.Indices(2) == 1 || eventdata.Indices(2) == 2
    return
end

% Retrieve value from cell
set_value = str2num(eventdata.EditData);

% Retrieve table
data = get(hObject,'Data');

% Check if value is a valid number
if isnan(set_value) || (~isinteger(set_value) && ~isfloat(set_value))
    data{eventdata.Indices(1),eventdata.Indices(2)} = eventdata.PreviousData;
    set(hObject, 'Data', data);
    errordlg('Input must be a number','Error');
elseif (set_value == eventdata.PreviousData)
    return;
elseif (strcmp(questdlg(sprintf('Confirm change from %g to %g ?',eventdata.PreviousData, set_value), data{eventdata.Indices(1),1}, 'Yes', 'No', 'No'),'Yes'))
    tg = evalin('base','tg');
    if ~exist('tg')
        data{eventdata.Indices(1),eventdata.Indices(2)} = eventdata.PreviousData;
        set(hObject, 'Data', data);
        errordlg('Target machine variable ''tg'' not found!','Error');
    else
        try
            setparam(tg, getparamid(tg,data{eventdata.Indices(1),1},data{eventdata.Indices(1),2}), set_value); % Send value to xpc target
            data{eventdata.Indices(1),eventdata.Indices(2)} = set_value;
            set(hObject, 'Data', data);
        catch err
            data{eventdata.Indices(1),eventdata.Indices(2)} = eventdata.PreviousData;
            set(hObject, 'Data', data);
            errordlg(sprintf('Failed to set value for %s\n',data{eventdata.Indices(1),1}),'Error');
        end
    end
else % User selected 'No'
    data{eventdata.Indices(1),eventdata.Indices(2)} = eventdata.PreviousData;
    set(hObject, 'Data', data);
end


% --- Executes on button press in reload_model.
function reload_model_Callback(hObject, eventdata, handles)
% hObject    handle to reload_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tg = evalin('base','tg');
tg.load(tg.Application);
evalin('base','init_scopes');
PopulateTableValues(handles.parameter_table);


% --- Executes on button press in start_model.
function start_model_Callback(hObject, eventdata, handles)
% hObject    handle to start_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tg = evalin('base','tg');
tg.start


% --- Executes on button press in stop_model.
function stop_model_Callback(hObject, eventdata, handles)
% hObject    handle to stop_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tg = evalin('base','tg');
tg.stop

% --- Executes on button press in plot_data.
function plot_data_Callback(hObject, eventdata, handles)
% hObject    handle to plot_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.xpc_gui, 'HandleVisibility', 'off');
evalin('base','plot_file_scopes');
