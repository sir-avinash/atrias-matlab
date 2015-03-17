function [eStop, u, userOut, dxy_f, dyaw_a] = controller3Dhopping(q, dq, userIn)



% Walking control for ATRIAS

% By: Siavash Rezazadeh, March 2015


%***************************************************

persistent x y tp xc yc ycp xcp xcpp ycpp dxp dyp t0 RefLeg Tcp Tcpp...
    dx_fp dy_fp dx_des_p dy_des_p q22d0...
    lFp rFp x_fpt0 x_f2f T y_fp f2f0 q24d xI yI xId yId xd yd ycppp Tcppp
persistent ti; if isempty(ti); ti = 0; else, ti = ti + 0.0009977; end


simOrRobot=2; %1:sim 2:robot



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

% Parse user input arguments into convient local variables
state = userIn(1); % Main controller state
% if ti> .0005
%     state=1;
% end
u_lim = clamp(userIn(2), 0, 600); % Motor torque limit (N*m)
kp_leg = clamp(userIn(3), 0, 5000); % Leg motor proportional gain (N*m/rad)
kd_leg = clamp(userIn(4), 0, 500); % Leg motor differential gain (N*m*s/rad)
kp_hip = clamp(userIn(5), 0, 2000); % Hip motor proportional gain (N*m/rad)
kd_hip = clamp(userIn(6), 0, 200); % Hip motor differential gain (N*m*s/rad)

if isempty(tp) || state~=1
    q22d0=0;
    dx_des_p=0;
    dy_des_p=0;
    ycppp=0;
    Tcppp=1;
    xId=0;
    yId=0;
    xd=0;
    yd=0;
    xI=0;
    yI=0;
    q24d=pi/2;
    f2f0=0;
    y_fp=0;
    T=.35;
    x_fpt0=[0;0];
    x_f2f=[0; 2*d];
    lFp=0;
    rFp=0;
    dx_fp=0;
    dy_fp=0;
   
    
   
    Tcp=1;
    Tcpp=1;
    RefLeg=1;
    t0=ti;
    tp=ti;
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
end


t_stb=4;

vs=.5;


if simOrRobot==1
    k_time=0*.1;
    kI=.5;
    kD=0*.2;
    k_fp_h_e=.1;
    k_fp_l_e=.1;
    k_fp_h_v=.2 -0*.05*(ti>t_stb);
    k_fp_l_v=.2-0*.05*(ti>t_stb);
    
    dx_des=0*(ti>t_stb);
    dy_des=0*(ti>t_stb);
    
    xs0=0;
    yss=.1-0*.01*(ti>t_stb)-0*.01*(ti>7)-0*.01*(ti>10);
    ys0=0;
    
%     T0=.37-.02-.05*(ti>t_stb);
    T0=.35;
    a_max=.3;
    ctrltest=userIn(7);
    c_yaw=2;
    lpo=.01*(ti>t_stb)+.01*(ti>7)+.01*(ti>10)+.01*(ti>13);

else
    ctrltest=userIn(15); %default=0
%     if ctrltest==2
        k_fp_h_e=clamp(userIn(7),0,.4);
        k_fp_l_e=clamp(userIn(8),0,.4);
        T0=clamp(userIn(9),.1,2);
        k_time=userIn(11);
        kI=clamp(userIn(10),0,5);
        kD=clamp(userIn(12),0,5);
        
        dx_des=clamp(userIn(13),0,3);
        dy_des=clamp(userIn(14),0,2);
        
        
        xs0=clamp(userIn(16),-.1,.1);
        yss=clamp(userIn(17),-.2,.2);
        ys0=clamp(userIn(18),-.08,.08);
        k_fp_h_v=clamp(userIn(19),0,.4);
        k_fp_l_v=clamp(userIn(20),0,.4);
        a_max=clamp(userIn(21),0,2);
        c_yaw=clamp(userIn(22),-9,9);
        lpo=clamp(userIn(23),-.08,.08);
%     else
%         k_fp_h_e=userIn(7);
%         k_fp_l_e=userIn(8);
%         T0=userIn(9);
%         k_time=userIn(11);
%         kI=userIn(10);
%         kD=userIn(12);
%         
%         dx_des=userIn(13);
%         dy_des=userIn(14);
%         
%         xs0=userIn(16);
%         yss=userIn(17);
%         ys0=userIn(18);
%         k_fp_h_v=userIn(19);
%         k_fp_l_v=userIn(20);
%         a_max=userIn(21);
%     end
end

% if ctrltest==3
    a_max=a_max/max(min(dx_des_p,1.1),.5);
% end

T0=clamp(T0-.05*(dx_des-vs)*(dx_des>vs),.2,T0);

dx_des=dx_des+(c_yaw*q(12)+0*3*dq(12)+0*.16*(1+0*dx_fp))*(-1)^RefLeg; %*(ti>t_stb);


if abs(dx_des)>vs || abs(dy_des)>.2
    dx_des=clamp(dx_des,dx_des_p-a_max*(ti-tp),dx_des_p+a_max*(ti-tp));
    dy_des=clamp(dy_des,dy_des_p-a_max*(ti-tp),dy_des_p+a_max*(ti-tp));
end


if abs(dx_des)<vs
    walkCmd=0;
else
    walkCmd=1;
end

ldev=.015*(dx_des-vs)*walkCmd-0*clamp(0.03*q(12),-01,.01)*(-1)^RefLeg*(ti>t_stb);

l0=.9;

l0=l0+0*(ti>t_stb)*clamp(0*-.06*q(12)*(-1)^RefLeg-.003*(-1)^RefLeg,-.04,.04);

l0sw=l0;
lmax0=l0;
kpT=1500;
kdT=100;



roll_d=-0*5*pi/180;



f_th=260;
f_th0=160;


% if ctrltest==6
%     lmin=.7;
% else
%     lmin=.7;
% end

lmin=.7;

kl0_y=0*.02;
kl0_x=0*.02;



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



RefLegF2=RefLeg;
if RefLeg==LEFT && lF<f_th && rF-lF>0
    RefLegF2=RIGHT;
elseif RefLeg==RIGHT && rF<f_th && lF-rF>0
    RefLegF2=LEFT;
end

x_f2f_t=0;
deltaLx=0;
lmax=.9;
dq22d=0;
x_f2f_t=2*d;
dq23d=0;
dq24d=0;
q22d=0;
dq21d=0;
x_fpt=[0;0];
dx_fpt=[0;0];
dqd=[0;0;0];
xFd=0;

f2f=2*d;
xy_st=[0 0];
phiL=0;
phiR=0;
dx=0;
dy=0;
q_hip_d=0;
uT=0;
q21d=acos(.9);
q23d=q21d;
% q24d=pi/2;

pitch=q(13);
dpitch=dq(13);
yaw=0*q(12);
dyaw=0*dq(12);
roll=q(11);
droll=dq(11);

yaw_a=q(12);
dyaw_a=dq(12);

phiR=q(9);
phiL=q(10);
dphiR=dq(9);
dphiL=dq(10);

dq_hip_d=0;


% absolute angles
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

ll1=cos((theta2-theta1)/2);
ll2=cos((theta4-theta3)/2);

F_l=2*k*(((theta2-theta1)/2)-((thetam2-thetam1)/2))/sqrtc(1-cos((theta2-theta1)/2)^2);
F_l2=2*k*(((theta4-theta3)/2)-((thetam4-thetam3)/2))/sqrtc(1-cos((theta4-theta3)/2)^2);

F_l3=k/2*((abs(theta2-thetam2))+abs((theta1-thetam1)));
F_l4=k/2*((abs(theta4-thetam4))+abs((theta3-thetam3)));


% if ctrltest==1
%     
%     if F_l3<30
%         fc=0;
%     elseif F_l3<60
%         fc=(F_l3-30)/(60-30);
%     else
%         fc=1;
%     end
%     
%     if F_l4<30
%         fc2=0;
%     elseif F_l4<60
%         fc2=(F_l4-30)/(60-30);
%     else
%         fc2=1;
%     end
% else
    if F_l<f_th0
        fc=0;
    elseif F_l<f_th
        fc=(F_l-f_th0)/(f_th-f_th0);
    else
        fc=1;
    end
    if F_l2<f_th0
        fc2=0;
    elseif F_l2<f_th
        fc2=(F_l2-f_th0)/(f_th-f_th0);
    else
        fc2=1;
    end
% end


t=ti-t0;


ml_R=cos((q(2)-q(4))/2);
ml_L=cos((q(6)-q(8))/2);
ll_R=cos((q(1)-q(3))/2);
ll_L=cos((q(5)-q(7))/2);
theta=(theta1+theta2)/2;
dtheta=(dtheta1+dtheta2)/2;

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

xHLFR=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (sin(yaw) * cos(roll) * sin(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d + (-cos(yaw) * cos(roll) * sin(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d; cos(roll) * sin(pitch) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * cos((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d + (-sin(roll) * sin(phiR) + cos(roll) * cos(pitch) * cos(phiR)) * cos(-(q(1)-pi/2) / 0.2e1 + (q(3)-pi/2) / 0.2e1) * sin((q(3)-pi/2) / 0.2e1 + (q(1)-pi/2) / 0.2e1) + (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d;];
xHRFL=[-(cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (sin(yaw) * cos(roll) * sin(phiL) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (-sin(yaw) * cos(roll) * cos(phiR) + (cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d; -(sin(yaw) * cos(pitch) + cos(yaw) * sin(roll) * sin(pitch)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiL)) * d + (-cos(yaw) * cos(roll) * sin(phiL) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (cos(yaw) * cos(roll) * cos(phiR) + (sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch)) * sin(phiR)) * d; cos(roll) * sin(pitch) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * cos((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiL) + cos(roll) * cos(pitch) * sin(phiL)) * d + (-sin(roll) * sin(phiL) + cos(roll) * cos(pitch) * cos(phiL)) * cos(-(q(5)-pi/2) / 0.2e1 + (q(7)-pi/2) / 0.2e1) * sin((q(7)-pi/2) / 0.2e1 + (q(5)-pi/2) / 0.2e1) - (sin(roll) * cos(phiR) + cos(roll) * cos(pitch) * sin(phiR)) * d;];

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


%     dy_uf=(cos((theta2-theta1)/2)*cos(qH1 + q(11)) + d*(-1)^RefLeg*sin(qH1 + q(11)))*(dqH1 + dq(11)) + zG*cos(q(11))*dq(11);


if ctrltest==1
        dx_uf=1/2*(sin(theta1)*dtheta1+sin(theta2)*dtheta2) + zG*cos(pitch)*dpitch;
else
        dx_uf=vG(1);
end


dy_uf=vG(2);

% dx_uf=dx_uf0;

% if ctrltest==2
%     if max(rF,lF)>f_th
%         
%         if abs(dx_uf-dxp)>2*(ti-tp)/.001
%             dx_uf=dxp;
%         end
%         if abs(dy_uf-dyp)>2*(ti-tp)/.001
%             dy_uf=dyp;
%         end
%     else
%         dx_uf=dxp;
%         dy_uf=dyp;
%     end
%     dx=0.01976*dxp+0.9802*dx_fp; %tf: 20/(s+20)
%     dy=0.01976*dyp+0.9802*dy_fp;
% elseif ctrltest==3
%     if max(rF,lF)>f_th
%         
%         if abs(dx_uf-dxp)>2*(ti-tp)/.001
%             dx_uf=dxp;
%         end
%         if abs(dy_uf-dyp)>2*(ti-tp)/.001
%             dy_uf=dyp;
%         end
%     else
%         dx_uf=dxp;
%         dy_uf=dyp;
%     end
%     dx=0.02463*dxp+0.9754*dx_fp; %tf: 25/(s+25)
%     dy=0.02463*dyp+0.9754*dy_fp;
%     
% else
    if max(rF,lF)>f_th
        
        if abs(dx_uf-dxp)>2*(ti-tp)/.001
            dx_uf=dxp;
        end
        if abs(dy_uf-dyp)>2*(ti-tp)/.001
            dy_uf=dyp;
        end
    else
        dx_uf=dxp;
        dy_uf=dyp;
    end
    dx=0.01485*dxp+0.9851*dx_fp; %tf: 1/(s/15+1)
    dy=0.01485*dyp+0.9851*dy_fp;
   
% end





dx_fp=dx;
dy_fp=dy;

dxp=dx_uf;
dyp=dy_uf;


l0=l0*(1-(kl0_y*(-1)^RefLeg*(dy_des-dy)+kl0_x*sign(pi/2-theta)*(dx_des-dx)));
lmax=lmax0;
l0sw=l0sw*(1-(-0*.02*(-1)^RefLeg*(dy_des-dy)-0*.02*(dx_des-dx)));




if abs(dx-dx_des)>10*(0.2*dx_des+.4) || abs(dy-dy_des)>1*(0.2*dy_des+.4) || kI==0 %|| ~walkCmd 
    ts=ti;
    x=0;
    y=0;
    xc=0;
    yc=0;
    xI=0;
    yI=0;
    xd=0;
    yd=0;
    xId=0;
    yId=0;
else
    x=x+dx*(ti-tp);
    y=y+dy*(ti-tp);
    xc=xc+dx*(ti-tp);
    yc=yc+dy*(ti-tp);
    xI=xI+x*(ti-tp);
    yI=yI+y*(ti-tp);
    xd=xd+dx_des*(ti-tp);
    yd=yd+dy_des*(ti-tp);
    xId=xId+xd*(ti-tp);
    yId=yId+yd*(ti-tp);
end




xs=0+xs0;
ys=-(-1)^RefLeg*yss+yG-ys0;



%x_fp wrt the hip when hip angle is zero

% dy_des=dy_des-(xcp/Tcp+xcpp/Tcpp)/2;
x_fp=[k_fp_l_v*dx; k_fp_h_v*dy]+[k_fp_l_e; k_fp_h_e].*[dx-dx_des; dy-dy_des]+[xs;ys]...
    +[1;.5].*[clamp(kI*.05*(1+dx_des)*(x-0*xc-xd),-.05,.05); clamp(2*kI*.05*(1+dy_des)*(y-0*yc-yd),-.05,.05)]...%*walkCmd...
    +0*clamp(.0005*[0;1].*[xcp/Tcp+xcpp/Tcpp-2*dx_des; ycp/Tcp+ycpp/Tcpp-2*dy_des],-.02,.02)*walkCmd...
    +0*[0; -.015]*walkCmd...
    -kD*[1;0].*clamp([.1*(xcp/Tcp-dx); .1*(ycp/Tcp-dy)],-.05,.05)*walkCmd...
    +0*clamp(.02*[0;yI-yId],-.1,.1)...
    +0*clamp(kD*[0;2].*[0; 0-(.7*yc/max(t,T/2)-.7*ycp/Tcp-0*.3*ycpp/Tcpp)],-.04,.04)*walkCmd...
    +0*[0;.5].*clamp([0; ycp+.6*ycpp+0*.3*ycppp],-.05,.05)*(ti>t_stb);%*walkCmd;

% x_fp(1)=x_fp(1)+c_yaw*(-1)^RefLeg;

%% INITIALIZE =========================================================

% Set default emergency stop to false
eStop = false;



% Initialize input vector to zeros
u = zeros(6,1);


%% MAIN CONTROLLER ====================================================

switch state
    case 0 % STAND ----------------------------------------------------------
    
    
    u([2 1 5 4])=kp_leg*[2^.5 2^.5 2^.5 2^.5]'.*([pi-acos(l0) pi+acos(l0) pi-acos(l0) pi+acos(l0)]'-[q(4) q(2) q(8) q(6)]')+...
                     kd_leg*[2^.5 2^.5 2^.5 2^.5]'.*(zeros(4,1)-[dq(4) dq(2) dq(8) dq(6)]');

   
    u([3 6]) = ([0; 0] - [q(9); q(10)])*kp_hip + (zeros(2,1) - [dq(9); dq(10)])*kd_hip;

    
    case 1 % WALK --------------------------------------------------------
        
        x_f2f_d = clamp(x_fp(1)-(cos(theta2)+cos(theta1))/2, -0.15, 0.4);
        
       
        deltaLx=lpo*(t>T/2)+(ldev*sign(x_st)+0*(x_st>0));%*(dx<dx_des)*walkCmd;

        T=T0;
        if t<=T/2
            q21d=acos(clamp(l0+deltaLx,.7,.96));
            dq21d=0;
        else
            q21d=acos(clamp(-4*(lmax-l0)/T^2*(t-T/2)^2+4*(lmax-l0)/T*(t-T/2)+l0+deltaLx,.7,.96));
            dq21d=(8*(lmax-l0)/T^2*(t-T/2)-4*(lmax-l0)/T)/abs(sin(q21d));
        end
        
        uT=clamp(ddqTc+kdT*(dqTc-dpitch)+kpT*(qTc-pitch),-.7*fc*F_l,.7*fc*F_l);%+(u3+u4)/n/IT;
        uT2=clamp(ddqTc+kdT*(dqTc-dpitch)+kpT*(qTc-pitch),-.7*fc2*F_l2,.7*fc2*F_l2);%+(u3+u4)/n/IT;

        
        q22d=-uT/k+(theta1+theta2)/2;
        dq22d=(dtheta1+dtheta2)/2;
        
      
        Tsw=T;
        T2=70*Tsw/100;
        T3=80*Tsw/100;
%         if ctrltest==2
%             T1=10*Tsw/100;
%             T4=90*Tsw/100;
%         else
            T1=1*Tsw/100;
            T4=81*Tsw/100;
%         end
        
        
        
        
         [x_f2f_t,dx_f2f_t] = spline3(t, T2, f2f0, 0, x_f2f_d, 0);

        
        
        if t>T3 && (t-(ti-tp))<=T3
            if RefLeg==RIGHT
                x_f2f=x_fp+xHLFR(1:2,1);
            else
                x_f2f=x_fp+xHRFL(1:2,1);
            end
           
        end
        
        if t<=T3
            [x_fpt,dx_fpt]=spline3(t,T3,x_fpt0,[0;0],x_fp,[0;0]);
        else
            if RefLeg==RIGHT
                x_fpt=x_f2f-xHLFR(1:2,1);
                dx_fpt=-[dx;dy];
            else
                x_fpt=x_f2f-xHRFL(1:2,1);
                dx_fpt=-[dx;dy];
            end
        end
       

        
               
        if t<=T1
            [q23d,dq23d]=spline3(t,T1,acos(l0),0,acos(lmin),0);
%             [q23d,dq23d]=spline3(t,T1,acos(l0),0,acos(lmin),0);
%             q24d=x_fpt(1)/cos(q23d);
%             dq24d=0;
%            
        elseif t<=T3
            q23d=acos(lmin);
            dq23d=0;
%             q24d=spline3(t,T3,q240,0,acos(clamp(x_fp(1)/cos(q23d),-1,1)),0);
%             dq24d=0;
        elseif t<=T4
            [q23d,dq23d]=spline3(t-T3,T4-T3,acos(lmin),0,acos(l0),0);
%             q24d=spline3(t,T3,q240,0,acos(clamp(x_fp(1)/cos(q23d),-1,1)),0);
%             dq24d=0;
        else
            q23d=acos(l0);
            dq23d=0;
%             q24d=acos(clamp(x_fpf(1)/l0,-1,1));
%             dq24d=0;
            
        end
        
        if t<((T3+T4)/2)
            y_fp = x_fp(2);
        end
        
        
        q24d=acos(clamp((x_f2f_t+(cos(theta2)+cos(theta1))/2)/(l0+deltaLx),-1,1));

        dq24d=0;
        
%         if ctrltest==3
%             c_h=clamp(2*t/T,0,1);
%         else
            c_h=clamp(t/T,0,1);
%         end
        
        u([2 1 5 4])=kp_leg*[2^.5 2^.5 1/2^.5 1/2^.5]'.*([(q22d-q21d) (q22d+q21d) q24d-q23d-fc2*uT2/k q24d+q23d-fc2*uT2/k]'-[thetam1 thetam2 thetam3 thetam4]')+...
                     kd_leg*[2^.5 2^.5 1/2^.5 1/2^.5]'.*([dq22d-dq21d dq22d+dq21d dq24d-dq23d dq24d+dq23d]'-[dthetam1 dthetam2 dthetam3 dthetam4]');
        
        if RefLeg==LEFT
            u([2 1 5 4])=u([5 4 2 1]);           
        end
        
        %         u([2 1])= [0 0]';
        
        %% HIP CONTROLLER =====================================================
        
        % Hip target position to counteract boom rotation
       
       
        
%     if ctrltest==4
%         q_hip_d=clamp(asin(clamp(y_fp/sqrt(ll1^2+d^2),-1,1))+atan2(d,ll1)*(-1)^RefLeg-roll,.13*(-1)^RefLeg,-.28*(-1)^RefLeg);
%     else
        q_hip_d=clamp(asin(clamp(y_fp/sqrt((l0)^2+d^2),-1,1))+atan2(d,l0)*(-1)^RefLeg-roll,.13*(-1)^RefLeg,-.28*(-1)^RefLeg);
%     end
    dq_hip_d=0;
     
        
        
    
        % Hip feedforward torque for gravity compensation
        u0_hip = 35;
        
            
        if RefLeg==RIGHT
            u(3)=clamp( -u0_hip +kp_hip*(fc/2*(roll-roll_d))+kd_hip*(fc/2*droll),-.7*fc*F_l-u0_hip,.7*fc*F_l+u0_hip)...
                +(1-fc)*(kp_hip*(q22d0-q(9))*kd_hip*(-dq(9)));
            u(6)=u0_hip+((q_hip_d-q(10))*(1-fc2)*c_h+fc2/2*(roll-roll_d))*kp_hip + ((dq_hip_d - dq(10))*(1-fc2)*c_h+fc2/2*droll)*kd_hip;
        else
            u(6)=clamp(u0_hip+kp_hip*(fc/2*(roll-roll_d))+kd_hip*(fc/2*droll),-.7*fc*F_l-u0_hip,.7*fc*F_l+u0_hip)...
                +(1-fc)*(kp_hip*(q22d0-q(10))*kd_hip*(-dq(10)));
            u(3)=-u0_hip+((q_hip_d-q(9))*(1-fc2)*c_h+fc2/2*(roll-roll_d))*kp_hip + ((dq_hip_d - dq(9))*(1-fc2)*c_h+fc2/2*droll)*kd_hip;
        end
        
        
        
        
    
        %% 
    
    
    otherwise % RELAX -----------------------------------------------------
        % Leg actuator torques computed to behave like virtual dampers
        u([2 1 5 4]) = (0 - dq([4 2 8 6]))*kd_leg;
        u([3 6]) = (0 - dq([9 10]))*kd_hip;
end % switch

% Limit absolute torque commands
u = clamp(u, -u_lim, u_lim);




if (ti-t0)>=T+0*.5*(dx_des-vs)*dx_des*walkCmd*(abs(dx-dx_des)<.5*dx_des) || ((ti-t0)>T/2 && RefLegF2~=RefLeg)
    Tcppp=Tcpp;
    Tcpp=Tcp;
    Tcp=ti-t0;
    t0=ti;
    f2f0=-1/2*(cos(theta4)+cos(theta3)-cos(theta2)-cos(theta1));
    if RefLeg==RIGHT
        RefLeg=LEFT;
    else
        RefLeg=RIGHT;
    end
    q22d0=q24d;
        
    x_fpt0=cos((theta2-theta1)/2)*[cos((theta1+theta2)/2); sin((theta1+theta2)/2)*sin(qH1)];
    
    ycppp=ycpp;
    xcpp=xcp;
    ycpp=ycp;
    xcp=xc;
    ycp=yc;
    xc=0;
    yc=0;
end

tp=ti;

qp=q;
dqp=dq;

dx_des_p=dx_des;
dy_des_p=dy_des;

% dxy=[dy dy_uf];
dxy=vG(1:2)';
dxy_f=[dx dy];

ym=y;
xm=x;
xym=[xm ym];
xy_st=[x_st y_st];
x_f2fo=x_f2f;
To=T;


userOut=[xym dxy q21d dq21d q22d dq22d q23d dq23d q24d dq24d q_hip_d x_f2f_t y_fp fc fc2 t T RefLeg dx_uf dy_uf ];

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