// clear all 
xdel(winsid());
clear;

//user inputs
Nrays = 250;
Aperture = %pi;
r_max = 20;

//create lines in the environment
map

//generate a scan
for i=1:Nrays
    azimuth = -Aperture/2 + (i/Nrays)*Aperture;
    
end

//invent a set of points + noise
points = [-1 2 3 4 5 6;-1 2 3 4 5 6];
//points = points(:,2:6);
points = points + rand(points,"normal")*0.1;
[xx N] = size(points);

//build the system : Ax=0. Matrix A = a_ij
a_00 = sum( points(1,:).^2 );
a_01 = sum( points(1,:).*points(2,:) );
a_02 = sum( points(1,:) );
a_10 = a_01;
a_11 = sum( points(2,:).^2 );
a_12 = sum( points(2,:) );
a_20 = a_02;
a_21 = a_12;
a_22 = N;
A = [a_00 a_01 a_02; a_10 a_11 a_12; a_20 a_21 a_22; 0 0 1];

//solve
line = pinv(A)*[zeros(3,1);1];
m = -line(1)/line(2);
xc = -line(3)/line(2);
disp("line: ");disp(line);
disp("m: ");disp(m);
disp("xc: ");disp(xc);

//error
//line_norm = line./line(3);
err = 0;
for i=1:N
    err = err + abs(line'*[points(:,i);1])/sqrt(line(1)^2+line(2)^2);
end
disp("error: "); disp(err/N);

//plot
fig1 = figure(0);
fig1.background = 8;
plot(points(1,:),points(2,:),"g.");
point1 = [points(1,1) m*points(1,1)+xc];
point2 = [points(1,N) m*points(1,N)+xc];
xpoly([points(1,1) points(1,N)],[m*points(1,1)+xc m*points(1,N)+xc]);



