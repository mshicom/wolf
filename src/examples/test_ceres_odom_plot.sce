//plot log data from ceres test

// clear all 
xdel(winsid());
clear;

//load log file
data = read('/home/acoromin/Desktop/log_file.txt',-1,12);

//plot
fig1 = figure();
fig1.background = 8;
plot(data(:,1),data(:,2),"b-");
plot(data(:,4),data(:,5),"r-");
plot(data(:,10),data(:,11),"g.");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$Optimization$";"$Ground\ Truth$";"$GPS$"],1);
lh.font_size = 3;                             
title("Vehicle trajectory");
ah.title.font_size = 4;
