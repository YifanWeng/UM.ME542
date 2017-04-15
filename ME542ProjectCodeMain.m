%% W2017 ME 542 Project: Formula 1 car racing Main script
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Main script
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;clc;close all;
%% Car information
load('F1CarData.mat')
Car=CarParameter;
%% Track Information
load('CircuitOfAmerica.mat');
%% Start point set
s_start=0; % start from the race start and end at the same location after one lap

%%%% If you want to start and end somewhere else in the middle  %%%%%%%%%%%
%%%% (for testing perpuse) uncommon and edit the following %%%%%%%%%%%%%%%%
% start_index=11; % 1 to 597
% end_index=21; % 1 to 597
% % start at a certain location 
% s_start=Track.arc_s(start_index);
% Track.bstl=Track.bl(:,start_index);
% Track.bstr=Track.br(:,start_index);
% Track.bfl=Track.bl(:,end_index);
% Track.bfr=Track.br(:,end_index);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
figure(1)
hold on;axis equal;box on
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k','Markersize',5);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k','Markersize',5);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','Markersize',5);
for i=1:length(Track.bl(1,:))
    if mod(i-1,20)==0
plot3([Track.bl(1,i) Track.br(1,i)],[Track.bl(2,i) Track.br(2,i)],[Track.bl(3,i) Track.br(3,i)],'b','LineWidth',1)
    end

end

Track.arc_s;

%%%%%%%%% initial condition %%%%%%%%%%%%%%%
XX0=Track.center(s_start);
v0=sqrt(Car.R_max/Car.k); % initial longitudinal speed top speed
x0=[XX0(1:2);Track.ftheta(s_start);v0];


%% Present your control design here
%% Open loop control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Driving force
Rcontrol=@(t) Car.R_max+0*t;

% steering angle
gamma=@(t) 0*t;
% % % For gammadot 
% % % you can either either provide a numerical expression
gammadot=@(t) 0*t;
% % % or rely on numerical estimation
% dt=1e-6;
% diff_num=@(f,dt,t) (f(t+dt)-f(t-dt))/(2*dt);
% gammadot=@(t) diff_num(gamma,dt,t); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% make sure you set a time long enough %%%%%%%%%%%%%%%%%%%

Time=200;
sim_step=0.05;
t_plot=0:sim_step:10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Check for the input constraints
[Rcontrol_real,gamma_real,gammadot_real]=InputChecker(Rcontrol,gamma,gammadot,Car,Time,sim_step);
%%
usize=9;
%%%  By default, the car RWD will give out dX and 9 values for monitoring purpose
%%% They are [R;tgamma;dtgamma;s0;sf;sr;n0;nf;nr]; at time t
% R driving force. tan(gamma), d(tan(gamma))/dt,
% s0 reference to the centerline of the center of mass,
% sf reference to the centerline of the front axle 
% sr reference to the centerline of the rear axle
% n0 distance to the centerline from the center of mass,
% nf distance to the centerline from the front axle 
% nr distance to the centerline from the rear axle
% IF you define more value to be monitored in car_RWD, you need to change
% this number accordingly

car_dynamics=@(t,x,y,psi,sigma,s) car_RWD(t,x,y,psi,sigma,gamma_real(t),gammadot_real(t),Rcontrol_real(t),Track,Car,s);                    
sys2=@(t,x,para) car_dynamics(t,x(1),x(2),x(3),x(4),para);

%% Run
Animation=1; % Animation on
% Animation=0; % Animation off
[t2,y2,u2]=CarSimRealTime(sys2,[0 Time],x0,s_start,sim_step,usize,Track,Car,Animation); 

%% Lateral Forces
[Ffl_ana,Frl_ana]=Force_rwd(y2(:,4),u2(:,1),u2(:,2),u2(:,3),Car.m,Car.m0,Car.b,Car.w);   

%% Show the trajectories
L=[];
Height=8;
Width=5;
FontSize=16;
showzoom=1;
figure(101);
set(gcf,'units','inches');
pos_default = get(gcf,'pos');
pos1=pos_default;
pos1(1)=pos1(1)+(pos1(3)-Width)/2;
pos1(2)=pos1(2)-(Height-pos1(4));
pos1(3)=Width;
pos1(4)=Height;
figure(101);
set(gcf,'pos',pos1)
subplot(4,1,1)
plot(t2,y2(:,1));L=[L,ylabel('$x$[m]')];
subplot(4,1,2)
plot(t2,y2(:,2));L=[L,ylabel('$y$[m]')];
subplot(4,1,3)
plot(t2,y2(:,3));L=[L,ylabel('$\psi$[rad]')];
subplot(4,1,4)
hold on;box on;
plot(t2,y2(:,4));L=[L,ylabel('$v$[m/s]')];L=[L,xlabel('$t$[sec]')];
RR=u2(:,1);
sss=u2(:,4);
nnn=u2(:,7);
delta2=u2(:,2);
figure(102)
subplot(4,1,1)
plot(t2,RR);L=[L,ylabel('$R$[N]')];
subplot(4,1,2)
plot(t2,sss);L=[L,ylabel('$s$[m]')];
subplot(4,1,3);hold on
plot(t2,nnn);L=[L,ylabel('$n$[m]')];
subplot(4,1,4)
plot(t2,delta2);L=[L,ylabel('$\delta$[rad]')];L=[L,xlabel('$t$[sec]')];
figure(103)
subplot(2,1,1);
plot(t2,Ffl_ana); L=[L,ylabel('$F_{\rm F}$[N]')];
subplot(2,1,2);
plot(t2,Frl_ana);L=[L,ylabel('$F_{\rm R}$[N]')];L=[L,xlabel('$t$[sec]')];
%% Show route togather with track
figure(3)
hold on;box on;axis equal;
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k-','LineWidth',1);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k-','LineWidth',1);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','LineWidth',1);
plot(y2(:,1),y2(:,2),'LineWidth',2);
set(L,'Interpreter','latex');
