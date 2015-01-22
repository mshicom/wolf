//plot log data from ceres test

// clear all 
xdel(winsid());
clear;

//load log file
data = read('~/Desktop/log_file.txt',-1,14);

//plot
fig1 = figure();
fig1.background = 8;
plot(data(2:$,1),data(2:$,2),"b-");
plot(data(2:$,4),data(2:$,5),"r-");
plot(data(2:$,10),data(2:$,11),"g.");
plot(data(2:$,13),data(2:$,14),"c--");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$Optimization$";"$Ground\ Truth$";"$GPS$";"$ODOM$"],1);
lh.font_size = 3;
title(strcat(["Vehicle trajectory - Time: ",string(data(1,1))," s"]));
ah.title.font_size = 4;

//load log file
data2 = read('~/Desktop/log_file_2.txt',-1,14);

//plot
fig2 = figure();
fig2.background = 8;
plot(data2(2:$,1),data2(2:$,2),"b-");
plot(data2(2:$,4),data2(2:$,5),"r-");
plot(data2(2:$,10),data2(2:$,11),"g.");
plot(data2(2:$,13),data2(2:$,14),"c--");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$Optimization$";"$Ground\ Truth$";"$GPS$";"$ODOM$"],1);
lh.font_size = 3;                             
title(strcat(["Vehicle trajectory wrapper - Time: ",string(data2(1,1))," s"]));
ah.title.font_size = 4;
