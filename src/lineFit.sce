// clear all 
xdel(winsid());
clear;

//user inputs
Nrays = 250;
Aperture = %pi;
r_max = 20;
r_stdev = 0.1;
Nw = 5; //window size

//init
result_lines = zeros(7,1);

//create lines in the environment
//map

//generate a scan
// for i=1:Nrays
//     azimuth = -Aperture/2 + (i/Nrays)*Aperture;
// end

//invent a set of points + noise
points = [1 2 3 4 5 6 7 6 5 4 3 2 1 0;1 2 3 4 5 6 7 8 9 10 11 12 13 14];
points = points + rand(points,"normal")*r_stdev;
[xx Np] = size(points);

//Main loop. Runs over a sliding window of Nw points
for (i = Nw:Np)
    //set the window
    points_w = points(:,(i-Nw+1):Nw)

    //build the system : Ax=0. Matrix A = a_ij
    a_00 = sum( points_w(1,:).^2 );
    a_01 = sum( points_w(1,:).*points_w(2,:) );
    a_02 = sum( points_w(1,:) );
    a_10 = a_01;
    a_11 = sum( points_w(2,:).^2 );
    a_12 = sum( points_w(2,:) );
    a_20 = a_02;
    a_21 = a_12;
    a_22 = Nw;
    A = [a_00 a_01 a_02; a_10 a_11 a_12; a_20 a_21 a_22; 0 0 1];

    //solve
    line = pinv(A)*[zeros(3,1);1];
//     m = -line(1)/line(2);
//     xc = -line(3)/line(2);
//     disp("line: ");disp(line);
//     disp("m: ");disp(m);
//     disp("xc: ");disp(xc);
    
    //compute error
    err = 0;
    for i=1:Nw
        err = err + abs(line'*[points_w(:,i);1])/sqrt(line(1)^2+line(2)^2);
    end
    err = err/Nw;
    disp("error: "); disp(err);
    
    //if error below stdev, add line to result set
    if (err < r_stdev)
        result_lines = [result_lines [line;points_w(:,1);points_w(:,$)];
    end    
    
end
    
    
//Set figure
fig1 = figure(0);
fig1.background = 8;

//plot points
plot(points(1,:),points(2,:),"g.");

//plot lines
[xx Nl] = size(result_lines);
for i=2:Nl
    m = -result_lines(1)/result_lines(2);
    xc = -result_lines(3)/result_lines(2);
    point1 = [points(1,1) m*points(1,1)+xc];
    point2 = [points(1,Np) m*points(1,Np)+xc];
    xpoly([points(1,1) points(1,Np)],[m*points(1,1)+xc m*points(1,Np)+xc]);
end



