clc
clear all
% Modified Points Mass Model


%% inital conditions
D=0.155;  %Diameter in meter
lat=39;
Azimuth0=0;
wind=[0;0;0]; % constant wind assumption 
time_step=0.01;
Vmuzzle0=900; % Muzzle velocity in meter/second
m0=43.096; % mass kg
Ix0=0.1426; % axial inertia kg.m^2
elevation0=70; % elevation angle in degree
BB=0;
target_position=trajectory(Vmuzzle0,D,m0,Ix0,lat,Azimuth0,elevation0,wind,time_step,BB);
target_velocity=[0;0;0];


%% Unguided Monte Carlo

Position_MonteCarlo=[];

Vmuzzle_Record=[];
m_Record=[];
Ix_Record=[];
elevation_Record=[];
Azimuth_Record=[];
for j=1:5

Vmuzzle=Vmuzzle0+Vmuzzle0*0.01/3*randn(1); % Muzzle velocity in meter/second
m=m0+m0*0.01/3*randn(1); % mass kg
Ix=Ix0+Ix0*0.001/3*randn(1); % axial inertia kg.m^2
elevation=elevation0+elevation0*0.0005/3*randn(1); % elevation angle in degree
Azimuth=Azimuth0+45*0.001/3*randn(1);
Position = trajectory(Vmuzzle,D,m,Ix,lat,Azimuth,elevation,wind,time_step,BB);

Position_MonteCarlo=[Position_MonteCarlo;Position(end,1) Position(end,3)];
Vmuzzle_Record=[Vmuzzle_Record;Vmuzzle];
m_Record=[m_Record;m];
Ix_Record=[Ix_Record;Ix];
elevation_Record=[elevation_Record;elevation];
Azimuth_Record=[Azimuth_Record;Azimuth];
j
end

Position_MonteCarlo=Position_MonteCarlo- ones(j,1)*[target_position(end,1) target_position(end,3)];

%% Guidance
Position_Guided=[];
cs=1; % g capacity of control section
for k=1:10

Vmuzzle=Vmuzzle0+Vmuzzle0*0.01/3*randn(1); % Muzzle velocity in meter/second
m=m0+m0*0.01/3*randn(1); % mass kg
Ix=Ix0+Ix0*0.001/3*randn(1); % axial inertia kg.m^2
elevation=elevation0+elevation0*0.0005/3*randn(1); % elevation angle in degree
Azimuth=Azimuth0+45*0.001/3*randn(1);
Position=guidance(Vmuzzle,D,m,Ix,lat,Azimuth,elevation,wind,time_step,target_position(end,:)',target_velocity,cs,BB);
Position_Guided=[Position_Guided;Position(end,1) Position(end,3)];
k
end

Position_Guided=Position_Guided- ones(k,1)*[target_position(end,1) target_position(end,3)];
%% Plots
figure
plot3(target_position(:,1),target_position(:,3),target_position(:,2))
ylabel('Çapraz Menzil')
xlabel('Menzil')
zlabel('Ýrtifa')
% hold on
% plot3(Position(:,1),Position(:,3),Position(:,2),'.')
 grid on
% legend('Ideal Trajectory', 'Guided Trial')
title('Yörünge')


% CEP Unguided
radius_vector=sqrt(Position_MonteCarlo(:,1).^2+Position_MonteCarlo(:,2).^2);
radius_vector=sort(radius_vector);
CEP_radius=radius_vector(floor(j/2),1)
th=0:pi/100:2*pi;
CEP_x=CEP_radius*cos(th);
CEP_y=CEP_radius*sin(th);
Range=sqrt(target_position(end,1)^2+target_position(end,3)^2)

% CEP Guided
radius_vector_guided=sqrt(Position_Guided(:,1).^2+Position_Guided(:,2).^2);
radius_vector_guided=sort(radius_vector_guided);
CEP_radius_guided=radius_vector_guided(floor(k/2),1)
CEP_x_guided=CEP_radius_guided*cos(th);
CEP_y_guided=CEP_radius_guided*sin(th);

figure
plot(0,0,'rh')
hold on
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'gx')
axis equal
xlabel('CrossRange')
ylabel('DownRange')
hold on
plot(CEP_x,CEP_y,'black')
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'b+')
grid on
legend('Target','Unguided Trials','Unguided Trials CEP','Guided Trials')
title('Monte Carlo Trials for CEP')


figure
plot(Vmuzzle_Record)
hold on
plot(Vmuzzle0*ones(j,1))
title('Muzzle Velocity Distribution')

figure
plot(m_Record)
hold on
plot(m0*ones(j,1))
title('Projectie Mass Distribution')

figure
plot(Ix_Record)
hold on
plot(Ix0*ones(j,1))
title('Moment of Inertia Distribution')

figure
plot(elevation_Record)
hold on
plot(elevation0*ones(j,1))
title('Elevation Angle Distribution')

figure
plot(Azimuth_Record)
hold on
plot(Azimuth0*ones(j,1))
title('Azimuth Angle Distribution')







%% Türkçe Tez

figure
plot(0,0,'rh')
hold on
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'.')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
grid on
axis([-400 400 -400 400])
legend('Hedef Noktasý','Güdümsüz Mühimmat Çarpma Noktasý')
title('Monte Carlo Denemeleri')


figure
plot(0,0,'rh')
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'.')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
grid on
axis([-1 1 -1 1])
legend('Hedef Noktasý','Güdümlü Mühimmat Çarpma Noktasý')
title('Monte Carlo Denemeleri')


figure
plot(0,0,'rh')
hold on
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'gx')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
grid on
axis([-400 400 -400 400])
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'b+')
grid on
legend('Hedef Noktasý','Güdümsüz Mühimmat Çarpma Noktasý','Güdümlü Mühimmat Çarpma Noktasý')
title('Monte Carlo Denemeleri')

%% Türkçe tez Son deðerlendirme
figure
plot(0,0,'rh','LineWidth',2)
hold on
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'gx')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
hold on
plot(CEP_x,CEP_y,'black')
grid on
legend('Hedef','Güdümsüz Mühimmat','Güdümsüz Mühimmat CEP')
title('Güdümsüz Mühimmat Daðýlýmý')


figure
plot(0,0,'rh','LineWidth',2)
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
hold on
plot(CEP_x_guided,CEP_y_guided,'magenta','LineWidth',2)
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'b+')
grid on
legend('Hedef','Güdümlü Mühimmat CEP','Güdümlü Mühimmat')
title('Güdümlü Mühimmat Daðýlýmý')


figure
plot(0,0,'rh','LineWidth',2)
hold on
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'gx')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
hold on
plot(CEP_x,CEP_y,'black')
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'b+')
hold on
plot(CEP_x_guided,CEP_y_guided,'magenta','LineWidth',2)
grid on
legend('Hedef','Güdümsüz Mühimmat','Güdümsüz Mühimmat CEP',...
    'Güdümlü Mühimmat','Güdümlü Mühimmat CEP')
title('Güdümsüz ve Güdümlü Mühimmatlarýn Daðýlýmý')




figure
plot(Position_MonteCarlo(:,2),Position_MonteCarlo(:,1),'gx')
axis equal
xlabel('Çapraz Menzil')
ylabel('Menzil')
hold on
plot(CEP_x,CEP_y,'black')
hold on
plot(Position_Guided(:,2),Position_Guided(:,1),'b+')
hold on
plot(CEP_x_guided,CEP_y_guided,'magenta','LineWidth',2)
hold on
plot(0,0,'rh','LineWidth',2)
axis([-75 75 -75 75])
grid on
legend('Güdümsüz Mühimmat','Güdümsüz Mühimmat CEP',...
    'Güdümlü Mühimmat','Güdümlü Mühimmat CEP','Hedef')
title('Güdümsüz ve Güdümlü Mühimmatlarýn Daðýlýmý')

%% 2D

figure
plot(target_position(:,1),target_position(:,2))
xlabel('Menzil')
ylabel('Ýrtifa')
grid on
title('Yörünge')
