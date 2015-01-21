//plot log data from ceres test

// clear all 
xdel(winsid());
clear;

//load log file
filename = getenv('HOME');
filename = filename + '/Desktop/log_data.txt';
data = read(filename,-1,15);

//plot
fig1 = figure();
fig1.background = 8;
plot(data(:,1),data(:,2),"b-");
plot(data(:,10),data(:,11),"g.");
plot(data(:,13),data(:,14),"r-");
plot(data(:,4),data(:,5),"k-");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$Optimization$";"$GPS$";"$odom\ prior$";"$Ground\ Truth$"],1);
lh.font_size = 3;                             
title("Vehicle trajectory");
ah.title.font_size = 4;
