function Position = guidance(Vmuzzle,D,m,Ix,lat,Azimuth,elevation,wind,time_step,target_position,target_velocity,cs,BB)

aero_coef=[...
0.010 0.1414 1.78 0 1.889 3.342 -8.6 0.000 -0.02768 -0.71 -1.16 0 0;
0.400 0.1416 1.78 0 1.89 3.339 -8.7 0.000 -0.02794 -0.71 -1.16 0 0;
0.600 0.1428 1.79 0 1.901 3.369 -8.5 0.000 -0.02782 -0.71 -1.16 0 0;
0.700 0.1436 1.96 0 1.909 3.391 -8.4 0.000 -0.02769 -0.72 -1.26 0 0;
0.750 0.1449 2.07 0 1.916 3.43 -8.5 0.000 -0.0275 -0.72 -1.28 0 0;
0.800 0.1485 2.18 0 1.933 3.497 -8.4 0.000 -0.02747 -0.74 -1.23 0 0;
0.850 0.1496 2.36 0 1.941 3.6 -8.4 0.000 -0.02751 -0.76 -1 0 0;
0.875 0.1596 2.48 0 1.977 3.673 -8.8 0.000 -0.02735 -0.77 -0.84 0 0;
0.900 0.1754 2.57 0 2.033 3.741 -9.3 0.000 -0.02706 -0.79 -0.8 0 0;
0.925 0.195 2.74 0 2.112 3.819 -10 0.000 -0.02652 -0.89 -0.71 0 0;
0.950 0.2288 2.9 0 2.194 3.826 -10.8 0.000 -0.02618 -1.01 -0.55 0 0;
0.975 0.2733 3.08 0 2.269 3.756 -11.1 0.000 -0.0262 -0.9 -0.32 0 0;
1.000 0.3237 3.27 0 2.328 3.622 -11.4 0.000 -0.02638 -0.83 -0.16 0 0;
1.025 0.3573 3.46 0 2.385 3.523 -11.3 0.000 -0.02596 -0.8 -0.01 0 0;
1.050 0.3795 3.69 0 2.425 3.457 -11.3 0.000 -0.02566 -0.76 0.05 0 0;
1.100 0.3835 4.19 0 2.445 3.416 -11.6 0.000 -0.0253 -0.71 0.13 0 0;
1.200 0.3812 4.69 0 2.476 3.408 -12.6 0.000 -0.02504 -0.65 0.15 0 0;
1.350 0.3626 4.21 0 2.533 3.377 -12.9 0.000 -0.02448 -0.58 0.23 0 0;
1.500 0.3462 3.72 0 2.621 3.27 -13 0.000 -0.02362 -0.56 0.31 0 0;
1.750 0.3215 3.25 0 2.724 3.141 -13 0.000 -0.02274 -0.54 0.31 0 0;
2.000 0.2972 2.74 0 2.812 3.059 -12.4 0.000 -0.02204 -0.51 0.33 0 0;
2.250 0.2764 2.45 0 2.866 2.975 -12 0.000 -0.0212 -0.5 0.32 0 0
2.500 0.2607 2.18 0 2.865 2.928 -12 0.000 -0.02028 -0.5 0 0 0;
3.000 0.2329 1.73 0 2.821 2.874 -12 0.000 -0.01905 -0.5 0 0 0;
];


gt=500;
% inital conditions

yaw_response=[0;0;0];
twist=20; % twist in neden 20 olarak seçildiðinin anlaþýlmasý?
R=[0;-6.356766*10^6;0];

EAS=7.292115*10^(-5); % Earth Angular Speed

spin=2*pi*Vmuzzle/(twist*D);



position=[0;0;0];
velocity=Vmuzzle*([cosd(elevation)*cosd(Azimuth);...
sind(elevation);cosd(elevation)*sind(Azimuth)]);
acc=[0;0;0];

Position=[position'];
V=[velocity'];
Mach_Record=[];
a_Record=[];
for i=1:100000

% vair=[velocity(1,1)+wind(1);velocity(2,1);velocity(3,1)+wind(2)]; 
vair=velocity-wind;
[T, SS, P, rho] = atmosisa(position(2,:));

%Aerodynamic Coefficients
mach=norm(vair)/SS;
Mach_Record=[Mach_Record mach];
cdo=interp1(aero_coef(:,1),aero_coef(:,2),mach);
cda2=interp1(aero_coef(:,1),aero_coef(:,3),mach);
cla=interp1(aero_coef(:,1),aero_coef(:,5),mach);
cla3=interp1(aero_coef(:,1),aero_coef(:,12),mach);
cma=interp1(aero_coef(:,1),aero_coef(:,6),mach);
cma3=interp1(aero_coef(:,1),aero_coef(:,12),mach);
clp=interp1(aero_coef(:,1),aero_coef(:,9),mach);
cypa=interp1(aero_coef(:,1),aero_coef(:,10),mach);
cypa3=interp1(aero_coef(:,1),aero_coef(:,13),mach);
if(BB==1)
    if(500<i<5000)
       cdo=cdo*0.85;
    end
end
% Stanag 4355 Gravity
r=position-R;
g0=9.80665*(1-0.0026*cosd(2*lat));
g=-g0*(norm(R)^2/norm(r)^3)*r;


% Stanag 4355 Coriolis
w=[EAS*cosd(lat)*cosd(Azimuth);EAS*sind(lat);-EAS*cosd(lat)*sind(Azimuth)];
C=-2*cross(w,velocity);


% Stanag 4355 Drag
DF=(-1*pi*rho*(D^2)/8)*(cdo+(cda2*(norm(yaw_response)^2)))*norm(velocity)*vair;

% Stanag 4355 Magnus
MF=((-1)*pi*(D^3)*rho/8)*spin*cypa*cross(yaw_response,vair);

% Stanag 4355 Lift
LF=(pi*(D^2)*rho/8)*(cla+cla3*(norm(yaw_response)^2))*(norm(velocity)^2)*yaw_response;



% Total Acceleration according to Stanag 4355
if(i<gt)
acc=C+g+(DF+MF+LF)/m; 
else
    % Proportional Navigation
a_PN = PN(position,velocity,target_position,target_velocity);
a_CNTL_wT=a_PN-(C+g+(DF+MF+LF)/m);
control_projection=dot(a_CNTL_wT,velocity)*velocity/(norm(velocity)^2);
a_CNTL=a_CNTL_wT-control_projection;
if(mach>1)
    LD=4*(mach+3)/mach;
else
    LD=cla/cdo;
end
a_drag=norm(a_CNTL)/LD*(DF/norm(DF));


acc=a_CNTL+a_drag+C+g+(DF+MF+LF)/m;

a_Record=[a_Record norm(a_CNTL)];
end
velocity=velocity+ acc*time_step;
position=position+ velocity*time_step;
if(i>gt)
    spin=0;
end
% Stanag 4355 Spin
spin_rate=pi*rho*D^4*spin*clp*norm(velocity)/(16*Ix);
spin=spin+spin_rate*time_step;

% Stanag 4355 Yaw Response
yaw_response=(-8)*Ix*spin*cross(vair,acc)/(pi*rho*D^3*cma*norm(velocity)^4);


% Data Record
Position=[Position;position'];
V=[V; velocity'];

if (norm(target_position-position)<2)
    norm(target_position-position)
    break
end
if (position(2,1)<0)
    break
end
end


end