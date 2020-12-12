function plotcom(p,R,size,k)
% function plotcom(p,R) plots a center of mass symbol at position p with
% orientation R
close all
switch k
    case 'coarse'
        a=.44433292853227944784221559874174;
        v = [ 0 0         1;
            sin(pi/8) 0 cos(pi/8);0         sin(pi/8) cos(pi/8);
            sin(pi/4) 0 sin(pi/4);a a sqrt(1-2*a^2);0 sin(pi/4) sin(pi/4);
            cos(pi/8) 0 sin(pi/8);sqrt(1-2*a^2) a a;a sqrt(1-2*a^2) a;0 cos(pi/8) sin(pi/8);
            1 0 0;cos(pi/8) sin(pi/8) 0;sin(pi/4) sin(pi/4) 0;sin(pi/8) cos(pi/8) 0;0 1 0]./50;
        f  = [1 2 3;2 4 5;2 5 3;3 5 6;4 7 8;4 8 5;5 8 9;5 9 6;6 9 10;7 11 12;7 12 8;8 12 13;8 13 9;9 13 14;9 14 10;10 14 15];
        v0 = R*[v;[-v(:,1) -v(:,2) v(:,3)];[v(:,1) -v(:,2) -v(:,3)];[-v(:,1) v(:,2) -v(:,3)]]';
        v4 = [ v0(1,:)+p(1); v0(2,:)+p(2); v0(3,:)+p(3)].';
        v5 = [-v0(1,:)+p(1);-v0(2,:)+p(2);-v0(3,:)+p(3)].';
        f1 = [f;f+15;f+30;f+45];
    case 'smooth'
        % smoother com: what is the correct value for b?
        % b=5275338780780258*2^(-54);
        b  = 5271341053858645*2^(-54);
        c  = 5020924469873901*2^(-53);
        e  = 5590528873889244*2^(-54);
        a  = [sin(pi/12) cos(pi/12)  sin(pi/6) cos(pi/6) sqrt(.5) sqrt(1-2*b^2) sqrt(1-c^2-e^2)];
        v  = .01*[0 0 1;a(1) 0 a(2);0 a(1) a(2);a(3) 0 a(4);b b a(6);0 a(3) a(4);
            a(5) 0 a(5);c e a(7);e c a(7);0 a(5) a(5);a(4) 0 a(3) ;a(7) e c;[1 1 1]/sqrt(3);
            e a(7) c;0 a(4) a(3);a(2) 0 a(1) ;a(6) b b;a(7) c e;c a(7) e;b a(6) b;0 a(2) a(1);
            1 0 0;a(2) a(1) 0;a(4) a(3) 0;a(5) a(5) 0;a(3) a(4) 0;a(1) a(2) 0;0 1 0]
        f  = [1 2 3;2 4 5;2 5 3;3 5 6;4 7 8;4 8 5;5 8 9;5 9 6;6 9 10;7 11 12;7 12 8;8 12 13;8 13 9;9 13 14;
            9 14 10;10 14 15;11 16 17;11 17 12;12 17 18;12 18 13;13 18 19;13 19 14;14 19 20;14 20 15;15 20 21;
            16 22 23;16 23 17;17 23 24;17 24 18;18 24 25;18 25 19;19 25 26;19 26 20;20 26 27;20 27 21;21 27 28];
        v0 = R*[v;[-v(:,1) -v(:,2) v(:,3)];[v(:,1) -v(:,2) -v(:,3)];[-v(:,1) v(:,2) -v(:,3)]]';
        v4 = [ v0(1,:)+p(1); v0(2,:)+p(2); v0(3,:)+p(3)].';
        v5 = [-v0(1,:)+p(1);-v0(2,:)+p(2);-v0(3,:)+p(3)].';
        f1 = [f;f+28;f+56;f+84];
end

h=patch('Vertices',v4,'Faces',f1);
set(h,'FaceColor',[0 0 0],'EdgeAlpha',0,'FaceLighting','phong','DiffuseStrength',1,'FaceAlpha',0.9)
h=patch('Vertices',v5,'Faces',f1);
set(h,'FaceColor',[1 1 1],'EdgeAlpha',0,'FaceLighting','flat','DiffuseStrength',1,'FaceAlpha',0.9)

set(gca,'DataAspectRatio',[1 1 1])
light('Position',[1 0 0],'Style','infinite');
light('Position',[0 1 0],'Style','infinite');
light('Position',[0 0 1],'Style','infinite');
light('Position',[-1 0 0],'Style','infinite');
light('Position',[0 -1 0],'Style','infinite');
light('Position',[0 0 -1],'Style','infinite');
view([39.5 0])
