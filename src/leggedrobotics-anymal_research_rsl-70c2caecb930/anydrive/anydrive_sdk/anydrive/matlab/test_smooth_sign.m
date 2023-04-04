clear all
close all
clc

x = -2:0.01:2;
for i = 1:length(x)
    y(i) = smooth_sign(x(i), 0.05);
end
plot(x,y)