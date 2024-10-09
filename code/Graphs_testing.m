%%
n=1000;
x=zeros(n,1);
t=zeros(n,1);

for i=1:n
    DesignProblem03('Controller','datafile','data.mat','display',false);
    load('data.mat');
    x(i)= processdata.x(end);
    t(i)= processdata.t(end);
    figure(1)
    plot(processdata.x,processdata.z)
    grid on
    hold on
    i

end

figure(1)
x_label('Time')
ylabel('Height')
title('Flight Paths')
X=sum(x);
T=sum(t);
t_avg= T/length(t);

x_max=max(x)
x_avg= mean(x)
x_med= median(x)
s=std(x)

%%

nbins= 50;
figure(2)
hist(x,nbins)
line([x_avg,x_avg],[0,90],'linewidth',2,'Color','b')
line([x_med,x_med],[0,90],'linewidth',2,'Color','g')
xlabel('Distance')
ylabel('Instances')
legend('Distance Flown','Average Distance','Median Distance','Location','northwest')
title('Histogram of Distances Flown- Nico Alba')
grid on
grid minor



%%
j=1;
for i=1:length(x)
    if x(i)>10
        z(i)=x(i);
        x_new(j)=z(i);
        j=j+1;
    else
        z(i)=nan;
    end
end

x_new_max= max(x_new)
x_new_avg= mean(x_new)
x_new_med= median(x_new)
s_new=std(x_new)
figure(3)
hist(x_new,50)
line([x_new_avg,x_new_avg],[0,70],'linewidth',2,'Color','b')
line([x_new_med,x_new_med],[0,70],'linewidth',2,'Color','g')
xlabel('Distance')
ylabel('Instances')
legend('Distance Flown','Average Distance','Median Distance','Location','northwest')
title('Histogram of Distances Flown- No Outliers')
grid on
grid minor