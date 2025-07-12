%global B_stop_flag;
function [sys,x0,str,ts,simStateCompliance] = flyCooprationBarrier_sfunction(t,x,u,flag)
global B_stop_flag;
global A_stop_flag;
global A_stage_flag;
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0.01 0];
flyCooprationBarrierInitial()
% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

fig = get_param(gcbh,'UserData');
if ishghandle(fig, 'figure'),
  if strcmp(get(fig,'Visible'),'on'),
    ud = get(fig,'UserData');
    PlaneSets(t,ud,u);
  end
end
sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
x0=4;
y0=0;
sys=[];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate


function flyCooprationBarrierInitial()
global B_stop_flag;
global A_stop_flag;
global A_stage_flag;
A_stop_flag=0;
B_stop_flag=0;
A_stage_flag=0;
sys = get_param(gcs,'Parent');
barrier_r=50;
barrier_x=0;
barrier_y=0;
stationA_x=100;
stationA_y=0;
stationA_r=10;
stationB_x=-350;
stationB_y=0;
stationB_r=10;

x=[-350 100];
y=[0 0];
L=5;
w=5;
theta=[0 180]* pi / 180;
plane_shape_x=zeros(2,2,2);
plane_shape_y=zeros(2,2,2);
for i = 1:2
   plane_shape_x(i,:,:)=[x(i)+L*cos(theta(i))+w*0.5*cos(theta(i)+pi/2) x(i)+L*cos(theta(i))-w*0.5*cos(theta(i)+pi/2);...
                         x(i)-L*cos(theta(i))+w*cos(theta(i)+pi/2) x(i)-L*cos(theta(i))-w*cos(theta(i)+pi/2)];
   plane_shape_y(i,:,:)=[y(i)+L*sin(theta(i))+w*0.5*sin(theta(i)+pi/2) y(i)+L*sin(theta(i))-w*0.5*sin(theta(i)+pi/2);...
                         y(i)-L*sin(theta(i))+w*sin(theta(i)+pi/2) y(i)-L*sin(theta(i))-w*sin(theta(i)+pi/2)];              
end

Fig = get_param(gcbh,'UserData');
Fig = figure(...
  'Units',           'pixel',...
  'Position',        [500 30 700 700],...
  'Name',            'FlyManage');
AxesH = axes(...
  'Parent',  Fig,...
  'Units',   'pixel',...
  'Position',[50 50 600 600],...
  'CLim',    [1 64], ...
  'Xlim',    [-450 200],...
  'Ylim',    [-325 325],...
  'Visible', 'on');

rectangle('Position', [barrier_x-barrier_r,barrier_y-barrier_r,2*barrier_r,2*barrier_r], 'Curvature', [1 1],'FaceColor',[0.8500 0.85 0.85],'EdgeColor', 'k');
rectangle('Position', [stationA_x-stationA_r,stationA_y-stationA_r,2*stationA_r,2*stationA_r], 'Curvature', [1 1],'FaceColor',[0.9290 0.9 0.1250],'EdgeColor',[0.9290 0.9 0.1250]);
rectangle('Position', [stationB_x-stationB_r,stationB_y-stationB_r,2*stationB_r,2*stationB_r], 'Curvature', [1 1],'FaceColor',[0.9290 0.9 0.1250],'EdgeColor',[0.9290 0.9 0.1250]);

%line = plot([x(1), x(2)], [y(1), y(2)], 'g'); 

plane1 = surface(...
    'Parent',   AxesH,...
    'XData',    [plane_shape_x(1,1,1) plane_shape_x(1,1,2);plane_shape_x(1,2,1) plane_shape_x(1,2,2)],...
    'YData',    [plane_shape_y(1,1,1) plane_shape_y(1,1,2);plane_shape_y(1,2,1) plane_shape_y(1,2,2)],...
    'ZData',    zeros(2),...
    'CData',    11*ones(2),...
    'EdgeColor', 'g');
plane2 = surface(...
    'Parent',   AxesH,...
    'XData',    [plane_shape_x(2,1,1) plane_shape_x(2,1,2);plane_shape_x(2,2,1) plane_shape_x(2,2,2)],...
    'YData',    [plane_shape_y(2,1,1) plane_shape_y(2,1,2);plane_shape_y(2,2,1) plane_shape_y(2,2,2)],...
    'ZData',    zeros(2),...
    'CData',    11*ones(2),...
    'EdgeColor', 'g');
line=surface(...
    'Parent',   AxesH,...
    'XData',    [x(1) x(1);x(2) x(2)],...
    'YData',    [y(1) y(1)+0.1;y(2) y(2)-0.1],...
    'ZData',    zeros(2),...
    'CData',    11*ones(2),...
    'LineStyle', '-',...
    'FaceColor','none',...
    'EdgeColor', 'g');
variable_text = text(-300, 0.5+100, '', 'FontSize', 14);
variable_text2 = text(-300, 20+100, '', 'FontSize', 14);
variable_text3 = text(-300, -20+100, '', 'FontSize', 14);
variable_text4 = text(-300, -40+100, '', 'FontSize', 14);
variable_text5 = text(-300, 40+100, '', 'FontSize', 14);
FigUD.variable_text  =variable_text;
FigUD.variable_text2  =variable_text2;
FigUD.variable_text3  =variable_text3;
FigUD.variable_text4  =variable_text4;
FigUD.variable_text5  =variable_text5;
FigUD.plane1         = plane1;
FigUD.plane2         = plane2;
FigUD.line         = line;
FigUD.AxesH=AxesH;
set(Fig,'UserData',FigUD);

% line([5 5],[0,40]);

drawnow
%
% store the figure handle in the animation block's UserData
%
set_param(gcbh,'UserData',Fig);
function PlaneSets(t,ud,u)
global B_stop_flag;
global A_stop_flag;
global A_stage_flag;
barrier_r=50;
barrier_x=0;
barrier_y=0;
stationA_x=100;
stationA_y=0;
stationA_r=10;
stationB_x=-350;
stationB_y=0;
stationB_r=10;

x=[-350 100];
y=[0 0];
v=[19.012*3 10*3];
L=5;
w=5;
theta=[0 180]* pi / 180;
plane_shape_x=zeros(2,2,2);
plane_shape_y=zeros(2,2,2);
stage=0;
% 将变量的值格式化为字符串
value_str = sprintf('B站距离障碍物圆心距离：%.2f米', abs(x(1)*10));
set(ud.variable_text, 'String', value_str);
value_str2 = sprintf('B无人机速度：%.4fm/s', v(1)/3);
set(ud.variable_text2, 'String', value_str2);
for i = 1:2
    ["enter"]
   position=calculate_position(i,t);
   x(i)=position(1);
   y(i)=position(2);
   if i==1
        stage=position(3);
   end
   ["exit"]
   plane_shape_x(i,:,:)=[x(i)+L*cos(theta(i))+w*0.5*cos(theta(i)+pi/2) x(i)+L*cos(theta(i))-w*0.5*cos(theta(i)+pi/2);...
                         x(i)-L*cos(theta(i))+w*cos(theta(i)+pi/2) x(i)-L*cos(theta(i))-w*cos(theta(i)+pi/2)];
   plane_shape_y(i,:,:)=[y(i)+L*sin(theta(i))+w*0.5*sin(theta(i)+pi/2) y(i)+L*sin(theta(i))-w*0.5*sin(theta(i)+pi/2);...
                         y(i)-L*sin(theta(i))+w*sin(theta(i)+pi/2) y(i)-L*sin(theta(i))-w*sin(theta(i)+pi/2)];              
end


set(ud.plane1,...
    'XData',    [plane_shape_x(1,1,1) plane_shape_x(1,1,2);plane_shape_x(1,2,1) plane_shape_x(1,2,2)],...
    'YData',    [plane_shape_y(1,1,1) plane_shape_y(1,1,2);plane_shape_y(1,2,1) plane_shape_y(1,2,2)]);
set(ud.plane2,...
    'XData',    [plane_shape_x(2,1,1) plane_shape_x(2,1,2);plane_shape_x(2,2,1) plane_shape_x(2,2,2)],...
    'YData',    [plane_shape_y(2,1,1) plane_shape_y(2,1,2);plane_shape_y(2,2,1) plane_shape_y(2,2,2)]);

% var=sym('var');
% f1=(y(1)-y(2))/(x(1)-x(2))*(var-x(1))+y(1);
% f=(y(1)-y(2))^2*(var-x(1))+(x(1)-x(2))*(y(1)-y(2))*y(1)+(x(1)-x(2))^2*var;
% eqn= f==0;
% edge_x=solve(eqn,var);
% edge_y=subs(f1,var,edge_x);
h=abs(-x(1)*y(2)+x(2)*y(1))/sqrt((x(1)-x(2))^2+(y(1)-y(2))^2);
if h<=50
    set(ud.line,...
    'XData',    [x(1) x(1);x(2) x(2)],...
    'YData',    [y(1) y(1)+0.1;y(2) y(2)-0.1],...
    'EdgeColor', 'g');
else
    set(ud.line,...
    'XData',    [x(1) x(1);x(2) x(2)],...
    'YData',    [y(1) y(1)+0.1;y(2) y(2)-0.1],...
    'EdgeColor', 'r');
end


% value_str3 = sprintf('B_stop：%.d', B_stop_flag);
% set(ud.variable_text3, 'String', value_str3);
% value_str4 = sprintf('A_stop：%.d', A_stop_flag);
% set(ud.variable_text4, 'String', value_str4);
% value_str5 = sprintf('A的x坐标：%.2f', x(2));
% set(ud.variable_text5, 'String', value_str5);

pause(0)
drawnow 
sys=[]

function f=calculate_position(i,t)
    global B_stop_flag;
    global A_stop_flag;
    global A_stage_flag;
    
    x=[-350 100];
    y=[0 0];
    v=[19.012*3 10*3];
    w=[19.012*3/50 10*3/50];
    move_x=0;
    move_y=0;
    stage=0;

    %B_stage1_x_endPoint=-1*(350-50*cos(pi/180*acosd(50/abs(x(1)))));
    B_stage1_x_endPoint=-50*50/abs(x(1));
    B_stage1_spend_time=50*tan(pi/180*acosd(50/abs(x(1))))/v(1);
    B_stage2_spend_time=2*pi*50/v(1)*(180-60-acosd(50/abs(x(1))))/360;
    B_stage2_x_endPoint=50*cos(pi/180*60);
    B_stage2_y_endPoint=50*sin(pi/180*60);
    
    A_stage1_x_endPoint=B_stage2_x_endPoint;
    A_stage1_spend_time=50*tan(pi/180*acosd(50/abs(x(2))))/v(2);
    A_stage2_spend_time=2*pi*50/v(2)*(180-60-acosd(50/abs(x(1))))/360;
    A_stage2_x_endPoint=-50*cos(pi/180*acosd(50/abs(x(1))));
    A_stage2_y_endPoint=-50*sin(pi/180*acosd(50/abs(x(1))));
    
    if i==1
        
        if (x(1)+v(1)*t*cos(pi/180*asind(50/abs(x(1)))) < B_stage1_x_endPoint) && (B_stop_flag==0)
            move_x=x(1)+v(1)*t*cos(pi/180*asind(50/abs(x(1))));
            move_y=-tan(pi/180*asind(50/x(1)))*(v(1)*t*cos(pi/180*asind(50/x(1))));
            stage=1;
        elseif (-50*cos((t-B_stage1_spend_time)*w(1)+acos(50/abs(x(1)))) <50*cos(pi/180*60) )&& (B_stop_flag==0)
            move_x=-50*cos((t-B_stage1_spend_time)*w(1)+acos(50/abs(x(1))));
            temp_x=abs( -50*cos( (t-B_stage1_spend_time)*w(1)+acos(50/abs(x(1))) ) );
            move_y=sqrt(50^2-temp_x^2);
            stage=2;
        elseif (B_stage2_x_endPoint+v(1)*(t-B_stage1_spend_time-B_stage2_spend_time)*sin(pi/180*60)<100 ) && (B_stop_flag==0)
            move_x=B_stage2_x_endPoint+v(1)*(t-B_stage1_spend_time-B_stage2_spend_time)*sin(pi/180*60);
            move_y=B_stage2_y_endPoint-v(1)*(t-B_stage1_spend_time-B_stage2_spend_time)*cos(pi/180*60);
        else
            move_x=100;
            move_y=0;
            B_stop_flag=1;
        end
    elseif i==2
        
        if (x(2)-v(2)*t*cos(pi/180*asind(50/abs(x(2)))) > A_stage1_x_endPoint) && (A_stage_flag<=1)
            move_x=x(2)-v(2)*t*cos( pi/180*asind( 50/abs(x(2)) ) );
            move_y=-tan(pi/180*asind(50/x(2)))*(v(2)*t*cos(pi/180*asind(50/abs(x(2)))));
            A_stage_flag=1;
        elseif (50*cos((t-A_stage1_spend_time)*w(2)+acos(50/abs(x(2)))) >-50*cos(pi/180*acosd(50/abs(x(1)))) )&& (A_stage_flag<=2)
            move_x=50*cos((t-A_stage1_spend_time)*w(2)+acos(50/abs(x(2))));
            temp_x=abs( 50*cos((t-A_stage1_spend_time)*w(2)+acos(50/abs(x(2)))) );
            move_y=-sqrt(50^2-temp_x^2);
            A_stage_flag=2;
        elseif (A_stage2_x_endPoint-v(2)*(t-A_stage1_spend_time-A_stage2_spend_time)*sin(pi/180*acosd(50/abs(x(1))))>x(1) ) && (A_stage_flag<=3)
            move_x=A_stage2_x_endPoint-v(2)*(t-A_stage1_spend_time-A_stage2_spend_time)*sin(pi/180*acosd(50/abs(x(1))));
            move_y=A_stage2_y_endPoint+v(2)*(t-A_stage1_spend_time-A_stage2_spend_time)*cos(pi/180*acosd(50/abs(x(1))));
            A_stage_flag=3;     
        else
            move_x=x(1);
            move_y=0;
            A_stop_flag=4;
        end
        
    end

    
    
    f=[move_x,move_y,stage];
sys=[]
