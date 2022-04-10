t_0 = 0;
v_f = 1;
v_i = 0;
tau = 0.5;

t = -1:0.01:4;

v = v_f - (v_f - v_i)*exp(-(t - t_0) / tau);
v(1:100) = zeros(1, 100);
 
a =  [zeros(1, 100), zeros(1, 401) + 1];

figure('DefaultAxesFontSize',35);
plot(t, a, 'LineWidth', 2);
hold on;
plot(t, v, 'LineWidth', 2);
ylim([0 1.2])
ylabel("u[V]");
xlabel("t[s]");
legend({'Perfect response','Simulated response'})