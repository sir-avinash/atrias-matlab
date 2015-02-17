function [eStop, u, userOut, xym, dxy] = controller3D2(q, dq, userIn)

persistent x y tp xc yc ycp xcp xcpp ycpp dxp dyp t0 RefLeg Tcp Tcpp dypr dxpr dypl dxpl sdxl sdyl sdxr sdyr dx_c dy_c x_fpc ts dx_fp dy_fp FRFL FLFR qd rFp lFp
persistent ti; if isempty(ti); ti = 0; else, ti = ti + 0.001; end

k=2950;
n=50;
IT=3.305565;
leg_offset = 0.1831;
mT=17.5;
mLeg=(62-17.5)/2;
d=0.1831; %leg offset, lateral
%torso com from the center of the hip
yG=-(0.2225-0.1577);
yG=yG*22/62;
% zG=0.318-.05;
zG=0.110887850467290;
% yG=0;
% zG=0.110887850467290;

RIGHT=1;
LEFT=2;

if isempty(tp)
    rFp=0;
    lFp=0;
    qd=[0;pi/2-acos(.9); pi/2+acos(.9)];
    dx_fp=0;
    dy_fp=0;
    x_fpc=[0; 0];
    dx_c=0;
    dy_c=0;
    sdxr=0;
    sdyr=0;
    sdxl=0;
    sdyl=0;
    dxpr=0;
    dypr=-.5;
    dxpl=0;
    dypl=.5;
    Tcp=1;
    Tcpp=1;
    RefLeg=1;
    t0=0;
    tp=0;
    ts=0;
    x=0;
    y=0;
    xc=0;
    yc=0;
    xcp=0;
    ycp=0;
    xcpp=0;
    ycpp=0;
    dxp=0;
    dyp=0;
    FRFL=[0; -2*d; 0];
    FLFR=[0; 2*d; 0];
end

dx_des=0;
dy_des=0;
T0=.4;
l0=.9;
l0sw=.9;
lmax0=.9;
kpT=2000;
kdT=200;
k_fp=.05;
k_fp_h=.2;
k_fp_l=.3;
dl_po=.04;
s=.75;
lmin=.77;
qHmax=.3;
f_th=250;
f_th0=150;
f2f_min=.2;
z_sw=.13;

qTc=0;
dqTc=0;
ddqTc=0;






rF=-k*((q(2)-q(4))-(q(1)-q(3)))/sin((q(1)-q(3))/2);
lF=-k*((q(6)-q(8))-(q(5)-q(7)))/sin((q(5)-q(7))/2);

if abs(rF-rFp)>200
  rF=rFp;
end

if abs(lF-lFp)>200
  lF=lFp;
end

rFp=rF;
lFp=lF;



% RefLegF=RefLeg;
% if lF-rF>f_th || (lF>f_th && rF<f_th)
%     RefLegF=LEFT;
% end
% if rF-lF>f_th || (rF>f_th && lF<f_th)
%     RefLegF=RIGHT;
% end

RefLegF2=RefLeg;
if RefLeg==LEFT && rF-lF>f_th0
    RefLegF2=RIGHT;
elseif RefLeg==RIGHT && lF-rF>f_th0
    RefLegF2=LEFT;
end
% 
% if rF>lF
%     RefLegF=RIGHT;
% else
%     RefLegF=LEFT;
% end
% 

dqd=[0;0;0];
xFd=0;
dx_f=0;
dy_f=0;
f2f=2*d;
xy_st=[0 0];
T=T0;
phiL=0;
phiR=0;
dx=0;
dy=0;
q_hip_d=0;
uT=0;
q21d=acos(.9);
q23d=q21d;
q24d=pi/2;

pitch=q(13);
dpitch=dq(13);
yaw=q(12);
dyaw=dq(12);
roll=q(11);
droll=dq(11);


phiR=q(9);
phiL=q(10);
dphiR=dq(9);
dphiL=dq(10);

dq_hip_d=0;


if RefLeg == RIGHT
    % 1,2 == Stance leg
    % 3,4 == Trailing leg or swing leg
    % All angles are in world coordinates
    % Leg angles
    theta1=pitch+q(3)-pi/2;
    theta2=pitch+q(1)-pi/2;
    theta3=pitch+q(7)-pi/2;
    theta4=pitch+q(5)-pi/2;
    % Motor angles
    thetam1=pitch+q(4)-pi/2;
    thetam2=pitch+q(2)-pi/2;
    thetam3=pitch+q(8)-pi/2;
    thetam4=pitch+q(6)-pi/2;
    % Leg velocities
    dtheta1=dpitch+dq(3);
    dtheta2=dpitch+dq(1);
    dtheta3=dpitch+dq(7);
    dtheta4=dpitch+dq(5);
    % Motor velocities
    dthetam1=dpitch+dq(4);
    dthetam2=dpitch+dq(2);
    dthetam3=dpitch+dq(8);
    dthetam4=dpitch+dq(6);
    % Hip angles
    qH1=q(9);
    qH2=q(10);
    dqH1=dq(9);
    qH2=dq(10);
else % Left leg
    theta3=pitch+q(3)-pi/2;
    theta4=pitch+q(1)-pi/2;
    theta1=pitch+q(7)-pi/2;
    theta2=pitch+q(5)-pi/2;
    thetam3=pitch+q(4)-pi/2;
    thetam4=pitch+q(2)-pi/2;
    thetam1=pitch+q(8)-pi/2;
    thetam2=pitch+q(6)-pi/2;
    dtheta3=dpitch+dq(3);
    dtheta4=dpitch+dq(1);
    dtheta1=dpitch+dq(7);
    dtheta2=dpitch+dq(5);
    dthetam3=dpitch+dq(4);
    dthetam4=dpitch+dq(2);
    dthetam1=dpitch+dq(8);
    dthetam2=dpitch+dq(6);
    % Hip angles
    qH2=q(9);
    qH1=q(10);
    dqH2=dq(9);
    dqH1=dq(10);
end

F_l=-2*k*(cos((theta2-theta1)/2)-cos((thetam2-thetam1)/2))/sqrtc(1-cos((theta2-theta1)/2)^2);

if F_l<f_th0
    fc=0;
elseif F_l<f_th
    fc=.7*(F_l-f_th0)/(f_th-f_th0);
else
    fc=.7;
end

ll_R=cos((q(1)-q(3))/2);
ll_L=cos((q(5)-q(7))/2);
theta=(theta1+theta2)/2;

% phi1=pi/2-pitch+(theta1+theta2)/2;
% phi2=pi/2-pitch+(theta3+theta4)/2;

% offsetL=[0; leg_offset; 0];
% R3_L=[1 0 0; 0 cos(q(10)) sin(q(10)); 0 -sin(q(10)) cos(q(10))];
% offsetR=[0; -leg_offset; 0];
% R3_R=[1 0 0; 0 cos(q(9)) sin(q(9)); 0 -sin(q(9)) cos(q(9))];
% R1=[1 0 0; 0 cos(q(11)) sin(q(11)); 0 -sin(q(11)) cos(q(11))];
% R2=[cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
% R4_R=[cos((q(1)+q(3))/2) 0 -sin((q(1)+q(3))/2); 0 1 0; sin((q(1)+q(3))/2) 0 cos((q(1)+q(3))/2)];
% R4_L=[cos((q(5)+q(7))/2) 0 -sin((q(5)+q(7))/2); 0 1 0; sin((q(5)+q(7))/2) 0 cos((q(5)+q(7))/2)];

% CoM_coordR=R1'*R2'*(R3_R'*(R4_R'*[0; 0; ll_R]+offsetR));
% y01=CoM_coordR(3);
% CoM_coordL=R1'*R2'*(R3_L'*(R4_L'*[0; 0; ll_L]+offsetL));
% y02=CoM_coordL(3);
% 
% xTFR = [0.1e1 / (2 + mT / mLeg) * (-0.2e1 * (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + 0.2e1 * (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + mT / mLeg * (-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG));
%     0.1e1 / (2 + mT / mLeg) * (-0.2e1 * (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + 0.2e1 * (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + mT / mLeg * (-(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + cos(yaw) * cos(roll) * yG + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * zG));
%     0.1e1 / (2 + mT / mLeg) * (0.2e1 * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + 0.2e1 * (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d + mT / mLeg * (cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + sin(roll) * yG + cos(roll) * cos(pitch) * zG));];

xOFR=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
xOFL=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];


JvOR=zeros(3,6);
JvOL=JvOR;

JvOR(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * cos(roll) * cos(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (cos(yaw) * cos(roll) * sin(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); 0;];
JvOR(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * sin(roll) * cos(phiR) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(yaw) * sin(roll) * sin(phiR) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * sin(roll) * cos(phiR) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (cos(yaw) * sin(roll) * sin(phiR) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -sin(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(roll) * cos(phiR) - sin(roll) * cos(pitch) * sin(phiR)) * d + (-cos(roll) * sin(phiR) - sin(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
JvOR(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); cos(roll) * cos(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * sin(phiR) * d - cos(roll) * sin(pitch) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
JvOR(:,4)=[(sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (sin(yaw) * cos(roll) * cos(phiR) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (-cos(yaw) * cos(roll) * cos(phiR) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * d + (-sin(roll) * cos(phiR) - cos(roll) * cos(pitch) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
JvOR(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
JvOR(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];

JvOL(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * cos(roll) * cos(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (cos(yaw) * cos(roll) * sin(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); 0;];
JvOL(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * sin(roll) * cos(phiL) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(yaw) * sin(roll) * sin(phiL) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * sin(roll) * cos(phiL) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (cos(yaw) * sin(roll) * sin(phiL) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -sin(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(roll) * cos(phiL) - sin(roll) * cos(pitch) * sin(phiL)) * d + (-cos(roll) * sin(phiL) - sin(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
JvOL(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); cos(roll) * cos(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * sin(pitch) * sin(phiL) * d - cos(roll) * sin(pitch) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
JvOL(:,4)=[-(sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (sin(yaw) * cos(roll) * cos(phiL) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (-cos(yaw) * cos(roll) * cos(phiL) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * d + (-sin(roll) * cos(phiL) - cos(roll) * cos(pitch) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
JvOL(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
JvOL(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];






if RefLegF2==RIGHT
    
    
    JvGR=zeros(3,6);
    
    JvGR(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * cos(roll) * cos(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (cos(yaw) * cos(roll) * sin(phiR) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
    
    JvGR(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * sin(roll) * cos(phiR) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(yaw) * sin(roll) * sin(phiR) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-cos(yaw) * sin(roll) * cos(phiR) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiR)) * d + (cos(yaw) * sin(roll) * sin(phiR) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(roll) * cos(phiR) - sin(roll) * cos(pitch) * sin(phiR)) * d + (-cos(roll) * sin(phiR) - sin(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
    JvGR(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiR) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * sin(phiR) * d - cos(roll) * sin(pitch) * cos(phiR) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
    JvGR(:,4)=[(sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (sin(yaw) * cos(roll) * cos(phiR) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * d + (-cos(yaw) * cos(roll) * cos(phiR) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1); (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * d + (-sin(roll) * cos(phiR) - cos(roll) * cos(pitch) * sin(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);];
    
    JvGR(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
    
    JvGR(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * sin(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) / 0.2e1;];
    
    
    vGR=JvGR*[dyaw, droll, dpitch, dphiR, dq(3), dq(1)]';
    vG=vGR;
    
    %position of center of the hip wrt the stance foot
    x_st=-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);
    y_st=-(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1);
    
    
    

    
    
    
    
    
else
    
    JvGL=zeros(3,6);
    
    JvGL(:,1)=[-(-sin(yaw) * cos(pitch) - cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * cos(roll) * cos(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (cos(yaw) * cos(roll) * sin(phiL) + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * cos(roll) * yG + (-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * zG; -(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - sin(yaw) * cos(roll) * yG + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * zG; 0;];
    
    
    JvGL(:,2)=[sin(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * sin(roll) * cos(phiL) + sin(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(yaw) * sin(roll) * sin(phiL) + sin(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + sin(yaw) * sin(roll) * yG + sin(yaw) * cos(roll) * cos(pitch) * zG; -cos(yaw) * cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-cos(yaw) * sin(roll) * cos(phiL) - cos(yaw) * cos(roll) * cos(pitch) * sin(phiL)) * d + (cos(yaw) * sin(roll) * sin(phiL) - cos(yaw) * cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(yaw) * sin(roll) * yG - cos(yaw) * cos(roll) * cos(pitch) * zG; -sin(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(roll) * cos(phiL) - sin(roll) * cos(pitch) * sin(phiL)) * d + (-cos(roll) * sin(phiL) - sin(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * yG - sin(roll) * cos(pitch) * zG;];
    
    JvGL(:,3)=[-(-cos(yaw) * sin(pitch) - sin(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * zG; -(-sin(yaw) * sin(pitch) + cos(yaw) * sin(roll) * cos(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(phiL) * d + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * zG; cos(roll) * cos(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) + cos(roll) * sin(pitch) * sin(phiL) * d - cos(roll) * sin(pitch) * cos(phiL) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - cos(roll) * sin(pitch) * zG;];
    
    JvGL(:,4)=[-(sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (sin(yaw) * cos(roll) * cos(phiL) - (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * d + (-cos(yaw) * cos(roll) * cos(phiL) - (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1); -(-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * d + (-sin(roll) * cos(phiL) - cos(roll) * cos(pitch) * sin(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);];
    
    
    JvGL(:,5)=[(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
    
    
    JvGL(:,6)=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1; cos(roll) * sin(pitch) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 - cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * sin(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1 + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) / 0.2e1;];
    
    
    vGL=JvGL*[dyaw, droll, dpitch, dphiL, dq(7), dq(5)]';
    vG=vGL;
    
    
    %position of center of the hip wrt the stance foot
    x_st=-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);
    y_st=-(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1);
    
    
    
end


if max(rF,lF)>f_th
    dx=vG(1);
    dy=vG(2);
else
    dx=dxp;
    dy=dyp;
end

dxp=dx;
dyp=dy;

dz=vG(3);


dx=0.0009995*dx+0.999*dx_fp;
dy=0.0009995*dy+0.999*dy_fp;
dx_fp=dx;
dy_fp=dy;

T=max(.25,T0*(1+min(-.6*(abs(dy_des-dy)),-.6*(abs(dx_des-dx)))));
% lmax=lmax0*(1+min((abs(dy_des)-abs(ycp/Tcp)),.3*(abs(dx_des)-abs(xcp/Tcp))));
l0=l0*(1-(.02*(-1)^RefLeg*(dy_des-dy)+.02*sign(pi/2-theta)*(dx_des-dx)));
lmax=l0;
l0sw=l0sw*(1-(-0*.02*(-1)^RefLeg*(dy_des-dy)-0*.02*(dx_des-dx)));


t=ti-t0;

if abs(dx-dx_des)>.6 || abs(dy-dy_des)>.8
    ts=ti;
    x=0;
    y=0;
    xc=0;
    yc=0;
else
    x=x+dx*(ti-tp);
    y=y+dy*(ti-tp);
    xc=xc+dx*(ti-tp);
    yc=yc+dy*(ti-tp);
end





y_fp0=d;%d-yG/2;
if RefLeg==RIGHT
    %     x_fp=[0; yG/2]+[k_fp_l; k_fp_h].*[dx; dy]+0*.1*[dx-dxpl; dy-dypl]+.02*[sdxr; sdyr];
    xs=(dx+dxpl);
    ys=(dy+dypl);
    xs=0;
    ys=-d+yG+y_fp0+0*(dy-dypl);
%     ys=yG;
%     ys=yG/2;
    %     xs=.05*sdxr;
    %     ys=.05*sdyr;
else
    %     x_fp=[0; yG/2]+[k_fp_l; k_fp_h].*[dx; dy]+0*.1*[dx-dxpr; dy-dypr]+.02*[sdxl; sdyl];
    xs=(dx+dxpr);
    ys=(dy+dypr);
    xs=0;
    ys=d+yG-y_fp0+0*(dy-dypr);
%     ys=yG;
%     ys=yG/2;
    %     xs=.05*sdxl;
    %     ys=.05*sdyl;
end


%x_fp wrt the hip when hip angle is zero
if sign(dx_des)==sign(dx)
    x_fp0=.2*dx_des;
else
    x_fp0=.2*dx_des;
end
% x_fp=[x_fp0; 0]+[k_fp_l; k_fp_h].*[dx-dx_des; dy*(abs(dy))-dy_des*(dy_des)]+0*[0; yG]+[0*.2*clamp(.05*x,-.05,.05); .1*clamp(.05*y,-.05,.05)]+...
%     0*[.1*(0*xc/max(t,T/2)+xcp/Tcp); .1*(0*yc/max(t,T/2)+ycp/Tcp)]+1*[xs;ys]+0*.05*[xcp/Tcp-dx_des;0]; %[0*.05*(dx_c-dx); 0*.05*(dy_c-dy)];
x_fp=[x_fp0; .2*dy_des]+[k_fp_l; k_fp_h].*[dx-dx_des; dy-dy_des]+[xs;ys]...
    +[0;0*.1].*[xcp/Tcp-dx_des;ycp/Tcp-dy_des]...
    +0*[2*clamp(.05*(x-dx_des*(ti-ts)),-.1,.1); 2*clamp(.05*(y-dy_des*(ti-ts)),-.1,.1)]...
    +0*.1*[xcp/Tcp+xcpp/Tcpp-2*dx_des; ycp/Tcp+ycpp/Tcpp-2*dy_des]...
    ;
x_fp(1)=clamp(x_fp(1),-.5-x_st,.5-x_st);
x_fp(2)=clamp(x_fp(2),-.5-y_st,.5-y_st);
x_fp(2)=clamp(x_fp(2),-.35,.35);

if RefLeg==RIGHT
    if phiR>0
        f2f=sqrt((x_fp(1)+x_st)^2+(x_fp(2)+d+y_st)^2);
        if f2f<f2f_min
            x_fp=f2f_min/f2f*(x_fp+[x_st; y_st+d])-[x_st; y_st+d];
        end
    else
        x_fp(2)=clamp(x_fp(2),-2*d+f2f_min,inf);
    end
end
            
if RefLeg==LEFT
    if phiL<0
        f2f=sqrt((x_fp(1)+x_st)^2+(x_fp(2)-d+y_st)^2);
        if f2f<f2f_min
            x_fp=f2f_min/f2f*(x_fp+[x_st; y_st-d])-[x_st; y_st-d];
        end
    else
        x_fp(2)=clamp(x_fp(2),-inf,2*d-f2f_min);
    end
end

% x_fp(1)=clamp(x_fp(1),-.1,.1);
%% INITIALIZE =========================================================

% Set default emergency stop to false
eStop = false;

% Parse user input arguments into convient local variables
state = userIn(1); % Main controller state
u_lim = clamp(userIn(2), 0, 600); % Motor torque limit (N*m)
kp_leg = clamp(userIn(3), 0, 5000); % Leg motor proportional gain (N*m/rad)
kd_leg = clamp(userIn(4), 0, 500); % Leg motor differential gain (N*m*s/rad)
kp_hip = clamp(userIn(5), 0, 2000); % Hip motor proportional gain (N*m/rad)
kd_hip = clamp(userIn(6), 0, 200); % Hip motor differential gain (N*m*s/rad)

% Initialize input vector to zeros
u = zeros(6,1);

% Initialize user ouput vector
userOut = zeros(1,1);

%% MAIN CONTROLLER ====================================================

switch state
    case 0 % STAND --------------------------------------------------------
        % Target leg actuator positions (standing with legs split)
        %         q0_leg = pi + 0*[-0.2; -0.2; 0.2; 0.2] + [-acos(.7); acos(.7); -acos(.9); acos(.9)];
        %
        %         % Target leg actuator velocities
        %         dq0_leg = zeros(4,1);
        %
        %         % Leg actuator torques from PD controller
        %         u([2 1 5 4]) = (q0_leg - q([4 2 8 6]))*kp_leg + (dq0_leg - dq([4 2 8 6]))*kd_leg;
        %
        
        
        
        
        %         if t<=T/2
        %             q21d=acos(l0);
        %             dq21d=0;
        %             [q23d,dq23d]=spline3(t,T/2,acos(l02),0,acos(lmin),0);
        %         else
        %             q21d=acos(l0+dl_po*(t-T/2));
        %             dq21d=-dl_po/sqrt(1-(l0+dl_po*(t-T/2))^2);
        %             if t<=T
        %                 [q23d,dq23d]=spline3(t-T/2,T/2,acos(lmin),0,acos(l0),0);
        %             else
        %                 q23d=acos(l0);
        %                 dq23d=0;
        %             end
        %         end
        
        if t<=T/2
            q21d=acos(l0);
            dq21d=0;
        else
            q21d=acos(clamp(-4*(lmax-l0)/T^2*(t-T/2)^2+4*(lmax-l0)/T*(t-T/2)+l0,-1,1));
            dq21d=(8*(lmax-l0)/T^2*(t-T/2)-4*(lmax-l0)/T)/abs(sin(q21d));
        end
        
        uT=clamp(ddqTc+kdT*(dqTc-dpitch)+kpT*(qTc-pitch),-fc*F_l,fc*F_l);%+(u3+u4)/n/IT;
        
        
        q22d=-uT/k+(theta1+theta2)/2;
        dq22d=(dtheta1+dtheta2)/2;
        
        %q23d=acos(a1*t^3+a2*t^2+a3*t+a4)
        
%         [yaw, roll, pitch, phiR, theta1R, theta2R]
%         [yaw, roll, pitch, phiL, theta1L, theta2L]
%       

        Tsw=.9*T;
        T1=1.5*Tsw/10;
        T2=9*Tsw/10;
        
        if ti>T2 && tp<=T2
            if RefLeg==LEFT
                qd=[phiR; thetam3; thetam4];
                FRFL=xOFL-xOFR;
            else
                qd=[phiL; thetam3; thetam4];
                FLFR=xOFR-xOFL;
            end
        end
        
        if t<=T1
            [xFd,vFd]=spline3(t,T1,0,1,z_sw,0);
            if RefLeg==LEFT
                
                dqd=JvOR(:,4:6)\(JvOL*[dyaw, droll, dpitch, dphiL, dq(7), dq(5)]'-[0;0;vFd]-JvOR(:,1:3)*[dyaw, droll, dpitch]');
                q_hip_d=phiR;
            else
%                 [xFd,vFd]=spline3(t,T1,FLFR,[0;0;0],(FLFR+[0;0;zmax]),[0;0;0]);
                dqd=JvOL(:,4:6)\(JvOR*[dyaw, droll, dpitch, dphiR, dq(3), dq(1)]'-vFd-JvOL(:,1:3)*[dyaw, droll, dpitch]');
                q_hip_d=phiL;
            end
            qd=qd+dqd*(ti-tp);
%             q_hip_d=qd(1);
            dq_hip_d=dqd(1);
            dq_hip_d=0;
            thetam3d=qd(2);
            thetam4d=qd(3);
            dthetam3d=dqd(2);
            dthetam4d=dqd(3);
            thetam3d=thetam3;
            thetam4d=thetam4;
        elseif t<=T2
            if floor(ti/T)==0
                a4=.9;
            else
                a4=lmax;
                a4=l0sw;
            end
            a1=(4*(l0sw - a4))/Tsw^3;
            a2=-(4*(l0sw - 2*a4 + lmin))/Tsw^2;
            a3=(l0sw - 5*a4 + 4*lmin)/Tsw;
            
            
            q23d=acos(clamp(a1*t^3+a2*t^2+a3*t+a4,-1,1));
            dq23d=-(3*a1*t^2+2*a2*t+a3)/abs(sin(q23d));
            
            
            %         q24d=pi/2;
            q24d=acos(clamp(x_fp(1)/cos(q23d),-1,1));
            dq24d=0;
            thetam3d=q24d-q23d;
            thetam4d=q24d+q23d;
            dthetam3d=dq24d-dq23d;
            dthetam4d=dq24d+dq23d;
        else
             if RefLeg==LEFT
                [xFd,vFd]=spline3(t-T2,Tsw-T2,FRFL(3),-.2,0,-.2);
                dqd=JvOR(:,4:6)\(JvOL*[dyaw, droll, dpitch, dphiL, dq(7), dq(5)]'-vFd-JvOR(:,1:3)*[dyaw, droll, dpitch]');
                q_hip_d=phiR;
            else
                [xFd,vFd]=spline3(t-T2,Tsw-T2,FLFR(3),-.2,0,-.2);
                dqd=JvOL(:,4:6)\(JvOR*[dyaw, droll, dpitch, dphiR, dq(3), dq(1)]'-vFd-JvOL(:,1:3)*[dyaw, droll, dpitch]');
                q_hip_d=phiL;
            end
            qd=qd+dqd*(ti-tp);
%             q_hip_d=qd(1);
            dq_hip_d=dqd(1);
            dq_hip_d=0;
            thetam3d=qd(2);
            thetam4d=qd(3);
            dthetam3d=dqd(2);
            dthetam4d=dqd(3);
            thetam3d=thetam3;
            thetam4d=thetam4;
        end
        
        
        
        
        
%         if t>T/2 && (pi/2-q24d)>(theta-pi/2)
%             q24d=pi-theta;
%         end
        
        u([2 1 5 4])=kp_leg*([q22d-q21d q22d+q21d thetam3d thetam4d]'-[thetam1 thetam2 thetam3 thetam4]')+...
            kd_leg*([dq22d-dq21d dq22d+dq21d dthetam3d dthetam4d]'-[dthetam1 dthetam2 dthetam3 dthetam4]');
        
        if RefLeg==LEFT
            u([2 1 5 4])=u([5 4 2 1]);
        end
        
        %         u([2 1])= [0 0]';
        
        %% HIP CONTROLLER =====================================================
        
        % Hip target position to counteract boom rotation
        %     q0_hip = 0*0.1271;% (-q(11) + 0.1271);
        if T1<t && t<=T2
            
            delta_y=x_fp(2)*(-1)^RefLeg;
            coss=[(d*(d+delta_y)+cos(q23d)*sin(q24d)*sqrtc(d^2-(d+delta_y)^2+(cos(q23d)*sin(q24d))^2))/(d^2+(cos(q23d)*sin(q24d))^2),...
                (d*(d+delta_y)-cos(q23d)*sin(q24d)*sqrtc(d^2-(d+delta_y)^2+(cos(q23d)*sin(q24d))^2))/(d^2+(cos(q23d)*sin(q24d))^2)];
            %      fprintf('coss1: %f\n',coss(1))
            %           fprintf('coss2: %f\n',coss(2))
            
            jc=find(coss>=0);
            %      fprintf('jc1: %f\n',jc(1))
            %      if length(jc)>1
            %          fprintf('jc2: %f\n',jc(2))
            %      end
            % coss(coss<0)=0;
            % coss(jc)=[];
            if isempty(jc)
                coss=[];
            else
                %         coss1=coss(jc);
                %         coss=coss1;
                coss=coss(jc);
            end
            
            jc=(find(abs(coss)<=1));
            if isempty(jc)
                coss=[];
            else
                %         coss1=coss(jc);
                %         coss=coss1;
                coss=coss(jc);
            end
            
            %     if length(coss)>1
            %         fprintf('coss1: %f\n',coss(1))
            %           fprintf('coss2: %f\n',coss(2))
            %     elseif length(coss)==1
            %         fprintf('coss1: %f\n',coss(1))
            %     else
            %         fprintf('coss1: empty\n')
            %     end
            if isempty(coss)
                q_hip_d=qHmax*sign(x_fp(2))-roll;
                %         jl=0;
                %         j1=0;
            else
                %         jl=length(coss);
                %         coss2=zeros(1,2*jl(1));
                %         phit=[acos(coss) -acos(coss)]-qH2;
                phit1=acos(coss)-qH2;
                phit2=-acos(coss)-qH2;
                [m1,j1]=min(abs(phit1));
                [m2,j2]=min(abs(phit2));
                %             if m1(1)>m2(1)
                %                 q_hip_d=-acos(coss(j2(1)));
                %             else
                %                 q_hip_d=acos(coss(j1(1)));
                %             end
                
                q_hip_d=sign(x_fp(2))*acos(coss(j1(1)))-roll;
                if RefLeg==RIGHT
                    q_hip_d=clamp(q_hip_d,-.15,.3);
                else
                    q_hip_d=clamp(q_hip_d,-.3,.15);
                end
                %         coss3=abs(coss-qH2);
                %         coss4=coss2-qH2;
                %         phia=abs(phit);
                %         phis=sort(phia);
                %         ja=(phia==phis(1));
                %         [m1,j1]=min(coss3);
                %         [m,j]=min(phia);
                %     qj1=size(j1,1);
                %     qj2=size(j1,2);
                %         fprintf('qj1: %f\n',qj1)
                %         fprintf('qj2: %f\n',qj2)
                
                %         if j1(1)<=jl(1)
                %             q_hip_d=acos(coss(j));
                %         else
                %             q_hip_d=-acos(coss(j1-jl));
                %         end
            end
        end
        
        
        
        %     q_hip_d=acos(clamp((d*(d+delta_y)+cos(q23d)*sin(q24d)*sqrtc(d^2-(d+delta_y)^2+(cos(q23d)*sin(q24d))^2))/(d^2+(cos(q23d)*sin(q24d))^2),-1,1));
        %     q_hip_d=x_fp(2);
        % Hip target velocity
        dq0_hip = zeros(2,1);
        
        % Hip feedforward torque for gravity compensation
        u0_hip = 35;
        
        % Hip actuator torques from PD controller with feedforward term
        %     u([3 6]) = u0_hip + (q0_hip - q([9 10]))*kp_hip + (dq0_hip - dq([9 10]))*kd_hip;
        
        if RefLeg==RIGHT
            u(3)=-u0_hip +clamp(kp_hip*roll+kd_hip*droll,-fc*F_l,fc*F_l);
            u(6)=u0_hip+(q_hip_d-q(10))*kp_hip + (dq_hip_d - dq(10))*kd_hip;
        else
            u(6)=u0_hip+clamp(kp_hip*roll+kd_hip*droll,-fc*F_l,fc*F_l);
            u(3)=-u0_hip+(q_hip_d-q(9))*kp_hip + (dq_hip_d - dq(9))*kd_hip;
        end
        
        %     u(3)=-200;
        
        
        
    
        %% 
    case 1
    
    
    otherwise % RELAX -----------------------------------------------------
        % Leg actuator torques computed to behave like virtual dampers
        u([2 1 5 4]) = (0 - dq([4 2 8 6]))*kd_leg;
        u([3 6]) = (0 - dq([9 10]))*kd_hip;
end % switch

% Limit absolute torque commands
u = clamp(u, -u_lim, u_lim);




if (ti-t0)>=T || ((ti-t0)>T/2 && RefLegF2~=RefLeg)
    Tcpp=Tcp;
    Tcp=ti-t0;
    t0=ti;
    
    if RefLeg==RIGHT
        RefLeg=LEFT;
        dxpl=dx;
        dypl=dy;
        sdxl=sdxl+dx;
        sdyl=sdyl+dy;
        FRFL=xOFL-xOFR;
        qd=[phiR; thetam1; thetam2];
    else
        RefLeg=RIGHT;
        dxpr=dx;
        dypr=dy;
        sdxr=sdxr+dx;
        sdyr=sdyr+dy;
        FLFR=xOFR-xOFL;
        qd=[phiL; thetam1; thetam2];
    end
    
    dy_c=dy;
    dx_c=dx;
    
    xcpp=xcp;
    ycpp=ycp;
    xcp=xc;
    ycp=yc;
    xc=0;
    yc=0;
end

tp=ti;



RefLeg1=RefLeg;
q21=(thetam2-thetam1)/2;
dxy=[dx dy];
dxy_f=[dx_f dy_f];
t01=t0;
F=[rF lF]/1000;
ym=y;
xm=x;
xym=[xm ym];
xy_st=[x_st y_st];
vcm=ycp/T;
% vz=vxyz(3);
% vx=vxyz(1);
% 


end % controller

%% LOCAL FUNCTIONS ========================================================

function b = clamp(a, lim1, lim2)
%CLAMP Clamp value between two bounds.

% Find which limit is min and max
a_min = min(lim1, lim2);
a_max = max(lim1, lim2);

% Clamp value between limits
b = max(min(a, a_max), a_min);
end % clamp

function [y,dy]=spline3(t,tf,q0,dq0,q1,dq1)

% Spline start (q0,dq0)
% Spline end (q1, dq1)
if t<tf
    a0=q0;
    a1=dq0;
    a2=(3*(q1-q0)-(2*dq0+dq1)*tf)/tf^2;
    a3=(2*(q0-q1)+(dq0+dq1)*tf)/tf^3;
    
    y=a0+a1*t+a2*t^2+a3*t^3;
    dy=a1+2*a2*t+3*a3*t^2;
else
    y=q1;
    dy=dq1;
end
end

function y=sqrtc(x)

if x>0
    y=sqrt(x);
else
    y=0;
end
end