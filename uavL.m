function [sys,x0,str,ts] = uavL(t,x,u,flag)
%---------the author is HLJ。
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

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
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%

function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 12; %12个状态量
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 13; %自己定
sizes.NumInputs      = 4;  %4个输入量
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions 提供初始状态量
rad2deg=180/pi;
psi_hmr=0.0;
if (psi_hmr> 180)   
    psi_hmr = psi_hmr-360.0; end
if (psi_hmr<-180)   
    psi_hmr = psi_hmr+360.0; end

Vt =20; alpha_deg=4.8186; beta_deg =0; theta_deg=4.8186; H=100;
x0  = [Vt; alpha_deg/rad2deg; beta_deg/rad2deg; 0; 0;H; 0;0;0; 0; theta_deg/rad2deg;psi_hmr/rad2deg];
% 输入角度转化成弧度 
str = [];
ts  = [0 0];
% end mdlInitializeSizes


function sys=mdlDerivatives(t,x,u)

% ---状态量-------
Vt      = x(1);   PN = x(4);    P = x(7);   phi   = x(10);       
alpha   = x(2);   PE = x(5);    Q = x(8);   theta = x(11);        
beta    = x(3);    H = x(6);    R = x(9);   psi   = x(12); 
%输入量均为弧度制
% ---输入量-------
ele = u(1);      % elevator deflection angle (deg)
ail = u(2);      % aileron  deflection angle (deg)
rud = u(3);      % rudder   deflection angle (deg)
dTc = u(4);   

global pcl pcd pcm pcQ pele peng pwind pcweight pcG
pcl=1; pcd=1; pcm=1; pcQ=1; pele=1; peng=1; pwind=0; pcweight=0; pcG=0;

% ----质量数据-------
mass=17+pcweight; %起飞质量mass、翼展b、弦长cbar、机翼面积SA
Ix=1.71; Iy=5.54; Iz=4.15;  Ixz=0.00;  
cbar=0.423; SA=1.3536; b=3.2;
g=9.80665;
%修改后
%mass=100+pcweight; 
%Ix=1.71; Iy=5.54; Iz=4.15;  Ixz=0.00;  
%cbar=0.458; SA=2.940; b=6.4;
%g=9.80665;

% ---- 计算空气密度和动压----
[ru,mach] = UAV_density(H,Vt);   % [air density] [mach number]
qs = SA*(ru*Vt*Vt/2);         % [Dynamic pressure](kg/m^2)
% ---- 计算CL,L,CD,D,CY,Y----
rad2deg=180/pi;
[CL0,CL_ele] = UAV_CL(alpha*rad2deg);  
CL = CL0*pcl + CL_ele*ele*pele;
L = qs*(CL);
[CD] = UAV_CD(alpha*rad2deg);
D = qs*(CD*pcd);
CY_beta = -0.00909;
CY = CY_beta*beta*rad2deg;
Y = qs*(CY);
% ---- 计算推力----
% Pow =eng*peng/100*(mass*g/4.0); % 最大油门是重力的1/4（稳定类飞机推重比一般是1/4到1/2）
Pow = D*cos(alpha)-(L-mass*g)*sin(alpha)+dTc; % Pow的一部分用于克服阻力分量D*cos(alpha)和升力重力分量(L-mass*g)*sin(alpha)，另一部分用于改变高度和速度

salpha = sin(alpha);        calpha = cos(alpha);                  
sbeta = sin(beta);          cbeta = cos(beta);
sphi = sin(phi);        cphi = cos(phi);
stheta = sin(theta);	ctheta = cos(theta);
spsi = sin(psi);        cpsi = cos(psi);
% %----转换阵S，从机体坐标系到速度坐标系----

% S = [calpha*cbeta,  sbata,  salpha*cbeta;
%    -calpha*sbeta,  cbeta,  -salpha*sbeta;
%    -salpha;        0,      calpha];

 % ----转换阵B，从地面坐标系到机体坐标系----
                           
 B=[cpsi*ctheta,                  spsi*ctheta,                  -stheta;
    cpsi*stheta*spsi-spsi*cphi,   spsi*stheta*sphi+cpsi*cphi,   ctheta*sphi;
    cphi*stheta*cpsi+spsi*sphi,   spsi*stheta*sphi-cpsi*sphi,   ctheta*cphi];

% ---- 机体坐标系Fx,Fy,Fz ----                       
Fx = Pow-D*calpha*cbeta-Y*calpha*sbeta+L*salpha;
Fy = -D*sbeta+Y*cbeta;                              
Fz = -D*salpha*cbeta-Y*salpha*sbeta-L*calpha;

% ---- U,V,W ----
U = Vt*calpha*cbeta; 
V = Vt*sbeta;
W = Vt*salpha*cbeta;

%风的设置：
 Vwind=[0;0;0;];
 if(H<5) 
   Vwind=[pwind;0;0;];  
 end
 BVwind= B*Vwind;
 
 U=Vt*cos(alpha)*cos(beta)+BVwind(1);
 V=Vt*sin(beta)+BVwind(2);
 W=Vt*sin(alpha)*cos(beta)+BVwind(3);

% ----dU,dV,dW ----
dU = R*V-Q*W-g*stheta+Fx/mass;
dV = -R*U+P*W+g*sphi*ctheta+Fy/mass;
dW = Q*U-P*V+g*cphi*ctheta+Fz/mass;

% ---dVt,dbeta,dalpha ---
dVt = (U*dU+V*dV+W*dW)/Vt;
dbeta = (dV*Vt-V*dVt)/(Vt*Vt*cbeta);
dalpha = (U*dW-W*dU)/(U*U+W*W);

% ----CR,R,CM,M,CN,N----
CR_R = 0.01832;
CR_P = -0.52568;% 单位为1/rad
CR_beta = -0.00600;
CR_rud = -0.000144;
CR_ail = -0.003618;% 单位为1/deg

CM_ele = -0.02052;% 单位为1/deg
CM_Q = -9.3136;
CM_dalpha = -4.0258;% 单位为1/rad

CN_beta = 0.00235;% 单位为1/deg
CN_R = -0.15844;
CN_P = -0.01792;% 单位为1/rad
CN_rud = -0.00111;
CN_ail = 0.000132;% 单位为1/deg

%----气动力矩-----
CR = CR_beta*beta*rad2deg+(b*(CR_P*P+CR_R*R))/(2*Vt)+CR_rud*rud+CR_ail*ail;
Lbar = qs*b*CR;
CM0 = UAV_CM(alpha*rad2deg);
CM = CM0*pcm+CM_ele*ele*pele+cbar/(2*Vt)*(CM_Q*Q*pcQ+CM_dalpha*dalpha);
M = qs*cbar*CM+L*pcG;
CN = CN_beta*beta*rad2deg+(b*(CN_R*R+CN_P*P))/(2*Vt)+CN_rud*rud+CN_ail*ail;
N = qs*b*CN;

% ----dP dQ dR----
dP = (Iz*Lbar+Ixz*N+((Iy-Iz)*Iz-Ixz*Ixz)*R*Q+(Ix-Iy+Iz)*Ixz*P*Q)/(Ix*Iz-Ixz*Ixz);
dQ = ((Iz-Ix)*P*R+M)/Iy;
dR = ((Ix*(Ix-Iy)+Ixz*Ixz)*P*Q-((Ix-Iy+Iz)*Ixz)*R*Q+Ixz*Lbar+Ix*N)/(Ix*Iz-Ixz*Ixz);

% ----dphi dtheta dpsi----
dphi = P+tan(theta)*(Q*sphi+R*cphi);
dtheta = Q*cphi-R*sphi;
dpsi = (Q*sphi+R*cphi)/ctheta;

% ----dPN dPE dH----
dPN = U*cpsi*ctheta+V*(-spsi*cphi+cpsi*stheta*sphi)+W*(spsi*sphi+cpsi*stheta*cphi);
dPE = U*ctheta*spsi+V*(cphi*cpsi+sphi*stheta*spsi)+W*(-sphi*cpsi+cphi*stheta*spsi);
dH = U*stheta-V*sphi*ctheta-W*cphi*ctheta;

% ---状态量的微分量-------
sys = [dVt;dalpha;dbeta;dPN;dPE;dH;dP;dQ;dR;dphi;dtheta;dpsi;]; 
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

% ---状态量-------
Vt      = x(1);   PN = x(4);    P = x(7);   phi   = x(10);       
alpha   = x(2);   PE = x(5);    Q = x(8);   theta = x(11);        
beta    = x(3);    H = x(6);    R = x(9);   psi   = x(12);  

rad2deg=180/pi;
%-------偏航角解算为真航向---
psi_hmr=psi*rad2deg;
if (psi_hmr<   0.0) psi_hmr=psi_hmr+360.0; end
if (psi_hmr>=360.0) psi_hmr=psi_hmr-360.0; end
%-------输出量--------------
sys = [Vt;alpha*rad2deg;beta*rad2deg;PN;PE;H;P*rad2deg;Q*rad2deg;R*rad2deg;phi*rad2deg;theta*rad2deg;psi*rad2deg;(theta-alpha)*rad2deg];

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

%=============================================================================
%UAV_density密度和马赫数计算
%air density and mach number respect to the altitude and airspeed
%=============================================================================
function [ru,mach]=UAV_density(H,Vt)

if H<11000
   temp=1-0.0225569*H/1000;
   ru  =0.12492*9.8*exp(4.255277*log(temp));
   mach=Vt/340.375/sqrt(temp);
else
   temp=11-H/1000;
   ru  =0.03718*9.8*exp(temp/6.318);
   mach=Vt/295.188;
end
% end UAV_density

%=============================================================================
%UAV_CL  气动升力系数/导数计算
%=============================================================================
function [CL0,CL_ele]=UAV_CL(alpha_deg)
    IDX_alpha = [    -4;   -2;    0;    2;    4;    8;   12;   16;   20];
    TBL_CL0 =   [-0.219;-0.04;0.139;0.299;0.455;0.766;1.083;1.409;1.743];
    CL0 =interp1d(TBL_CL0,IDX_alpha,alpha_deg);
    CL_ele = 0.00636;
  
%end UAV_CL
%=============================================================================
%UAV_CD  气动阻力系数/导数计算
%=============================================================================
function [CD] = UAV_CD(alpha_deg)
    IDX_alpha = [-4;-2;0;  2;4;8;12;16;20];
    TBL_CD0 = [ 0.026;0.024;0.024;0.028;0.036;0.061;0.102;0.141;0.173];
    CD = interp1d(TBL_CD0,IDX_alpha,alpha_deg);
%end UAV_CD
%=============================================================================
%UAV_CM  俯仰力矩系数/导数计算
%=============================================================================
function [CM] = UAV_CM(alpha_deg) 
    IDX_alpha = [-4;-2;0;  2;4;8;12;16;20];
    TBL_CM0 =[0.1161;0.0777;0.0393;0.0009;-0.0375;-0.0759;-0.1527;-0.2295;-0.3063];
    CM =interp1d(TBL_CM0,IDX_alpha,alpha_deg);
%end UAV_CL
%=============================================================================
%interp1d  一维插值
%=============================================================================
function  y = interp1d(A,idx,xi)

if xi<idx(1)
   r=1;
elseif xi<idx(end)
   r = max(find(idx <= xi));      
else
   r = length(idx)-1;
end

DA = (xi-idx(r))/(idx(r+1)-idx(r));
y = A(r) + (A(r+1)-A(r))*DA;
% END interp1d


