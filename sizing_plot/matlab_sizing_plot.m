%% Generate sizing plot, using the data from GP code

addpath('carpetplot_class_102'); 

%% Input Data

L_D_array = [10.00	10.80	11.60	12.40	13.20	14.00	
10.00	10.80	11.60	12.40	13.20	14.00	
10.00	10.80	11.60	12.40	13.20	14.00	
10.00	10.80	11.60	12.40	13.20	14.00	
10.00	10.80	11.60	12.40	13.20	14.00	
10.00	10.80	11.60	12.40	13.20	14.00];%dimensionless

T_A_array = [4.50	4.50	4.50	4.50	4.50	4.50	
6.40	6.40	6.40	6.40	6.40	6.40	
8.30	8.30	8.30	8.30	8.30	8.30	
10.20	10.20	10.20	10.20	10.20	10.20	
12.10	12.10	12.10	12.10	12.10	12.10	
14.00	14.00	14.00	14.00	14.00	14.00];%lbf/ft^2

MTOW_array = [3025.99	2877.69	2761.04	2666.88	2589.29	2524.24	
3088.17	2933.87	2812.72	2715.07	2634.68	2567.36	
3143.70	2983.95	2858.71	2757.90	2675.00	2605.63	
3194.88	3030.01	2900.96	2797.20	2711.96	2640.68	
3242.93	3073.21	2940.53	2833.97	2746.51	2673.43	
3288.65	3114.23	2978.07	2868.82	2779.23	2704.42];%lbf

cptpp_array = [57.05	53.17	50.15	47.80	45.89	44.29	
58.55	54.51	51.37	48.89	46.91	45.25	
59.90	55.70	52.45	49.86	47.81	46.10	
61.14	56.81	53.45	50.77	48.64	46.88	
62.31	57.85	54.39	51.63	49.42	47.62	
63.43	58.83	55.28	52.45	50.16	48.31];%lbf

SPL_array = [62.02	61.80	61.62	61.47	61.34	61.23	
65.17	64.95	64.76	64.61	64.48	64.37	
67.50	67.28	67.09	66.94	66.80	66.69	
69.37	69.14	68.95	68.79	68.65	68.54	
70.91	70.68	70.49	70.33	70.19	70.08	
72.24	72.00	71.81	71.65	71.51	71.39];%dB

SPL_A_array = [63.13	62.90	62.70	62.54	62.40	62.28	
65.95	65.70	65.49	65.31	65.16	65.03	
67.91	67.64	67.42	67.23	67.07	66.93	
69.40	69.12	68.88	68.68	68.52	68.37	
70.59	70.29	70.05	69.84	69.67	69.52	
71.58	71.27	71.01	70.80	70.62	70.46];%dBA

%% Plotting commands

figure('Position',[0 50 800 700]);
hold on;

carpetplot(L_D_array,T_A_array,cptpp_array,SPL_A_array,'k-','LineWidth',1.5);

%L/D labels
for i = 1:1:length(L_D_array(1,:))
    label = strcat(['L/D = ',num2str(L_D_array(1,i),'%0.1f')]);
    x = cptpp_array(1,i); y = SPL_A_array(1,i);
    t = text(x+1,y-0.5,label,'FontSize',14);
    set(t,'Rotation',-45);
end

%T/A labels
for i = 1:1:length(T_A_array(:,1))
    label = strcat(['T/A = ',num2str(T_A_array(i,1),'%0.1f'),' lbf/ft^2']);
    x = cptpp_array(i,end); y = SPL_A_array(i,1);
    t = text(x-6,y+0.5,label,'FontSize',14);
    set(t,'Rotation',-30);
end

xl = xlim; xmin = xl(1); xmax = xl(2);
yl = ylim; ymin = yl(1); ymax = yl(2);
xlim([xmin-4 xmax]);
ylim([ymin-1.5 ymax]);

hold off; grid on; 
xlabel('Passenger cost per trip ($)','FontSize',16);
xtickformat('usd')
ylabel('SPL (dBA)','FontSize',16);
title('Sizing Plot (Lift + Cruise Configuration)','FontSize',16);