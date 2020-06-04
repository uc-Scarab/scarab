indx1=1;
indx2=2;
indx3=3;
indx4=4;
d2r = pi/180; 


l1=0.5;
l2=2;
l3=2.5;
l4=4;

links(indx1).jType = 'revolute';
links(indx1).THETA = 0;
links(indx1).DISTANCE = 0.15;
links(indx1).A_DISTANCE = 0.6;
links(indx1).ALPHA = pi/2;
links(indx1).QLIM = [-30*d2r,30*d2r];
bioGrp(indx1) = Link(links(indx1).jType, 'd', links(indx1).DISTANCE, 'a', links(indx1).A_DISTANCE, 'alpha', links(indx1).ALPHA,'qlim',links(indx1).QLIM,'standard');
% %
% %
links(indx2).jType = 'revolute';
links(indx2).THETA = -(pi/16);
links(indx2).DISTANCE = 0;
links(indx2).A_DISTANCE = 0.85;
links(indx2).ALPHA = -(pi);
links(indx2).QLIM = [-40*d2r,40*d2r];
bioGrp(indx2) = Link(links(indx2).jType, 'd', links(indx2).DISTANCE, 'a', links(indx2).A_DISTANCE, 'alpha', links(indx2).ALPHA,'qlim',links(indx2).QLIM,'standard');
% %
% %
links(indx3).jType = 'revolute';
links(indx3).THETA = (5*pi/16);
links(indx3).DISTANCE = 0;
links(indx3).A_DISTANCE = 0.95;
links(indx3).ALPHA = pi;
links(indx3).QLIM = [-40*d2r,40*d2r];
bioGrp(indx3) = Link(links(indx3).jType, 'd', links(indx3).DISTANCE, 'a', links(indx3).A_DISTANCE, 'alpha', links(indx3).ALPHA,'qlim',links(indx3).QLIM,'standard');
% %
% %
links(indx4).jType = 'revolute';
links(indx4).THETA = -(5*pi/2);
links(indx4).DISTANCE = 0;
links(indx4).A_DISTANCE = 0.8;
links(indx4).ALPHA = -pi/2;
links(indx4).QLIM = [-50*d2r,50*d2r]; 
bioGrp(indx4) = Link(links(indx4).jType, 'd', links(indx4).DISTANCE, 'a', links(indx4).A_DISTANCE, 'alpha', links(indx4).ALPHA,'qlim',links(indx4).QLIM,'standard');
leg = SerialLink(bioGrp,'name','Scarab');

frob0 = [0, 0, 0.5236, -1.3963];
F = -3;

refresh; 
leg.plot(frob0, 'floorlevel', F);

