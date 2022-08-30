function a_PN = PN(position,velocity,target_position,target_velocity)


Rtm=target_position-position;
Vtm=target_velocity-velocity;

Vc=-(Rtm(1,1)*Vtm(1,1)+Rtm(2,1)*Vtm(2,1)+Rtm(3,1)*Vtm(3,1))/(norm(Rtm));
tgo=norm(Rtm)/Vc;

ZEM=Rtm+Vtm*tgo;

ZEMdotRTM=(ZEM(1,1)*Rtm(1,1)+ZEM(2,1)*Rtm(2,1)+ZEM(3,1)*Rtm(3,1))/norm(Rtm);

ZEMpar=ZEMdotRTM/norm(Rtm)*(Rtm);

ZEMper=ZEM-ZEMpar;

a_PN=3*ZEMper/(tgo^2);

end