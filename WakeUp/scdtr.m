data = readtable('tau.txt');
m_mean = array2table(horzcat(data.Var1, movmean(data.Var2, 125)));

number_of_levels = 8;
max_level_value = 4096;
step = 6000;
min_index = 500;

tau = zeros(1, number_of_levels);
max_index = step * number_of_levels;

for i=0:number_of_levels-1
    range = (i*step + min_index):((i+1)*step);

    min1 = min(m_mean.Var2(range));
    max1 = max(m_mean.Var2(range));

    target1_0 = min1 + ((max1-min1)*0.10);
    t1_0 = find(m_mean.Var2(range) > target1_0);

    target1_1 = min1 + ((max1-min1)*0.63);
    t1_1 = find(m_mean.Var2(range) > target1_1);
    
    tau(i+1) = (data.Var1(range(1) +  t1_1(1)) - data.Var1(range(1) + t1_0(1)))/10^6; 
end

tau = array2table(horzcat((max_level_value/8:max_level_value/8:max_level_value)', tau'));
tau_fit=fit(tau.Var1,tau.Var2,'smoothingspline');

figure('DefaultAxesFontSize',35);
plot(data.Var1, data.Var2, 'LineWidth', 2);
hold on;
plot(m_mean.Var1, m_mean.Var2, 'LineWidth', 4);
xlim([data.Var1(1) data.Var1(end)]);
ylabel("u[V]");
xlabel("t[\mus]");
legend({'Data','Moving average'})

figure('DefaultAxesFontSize',35);
plot(tau.Var1, tau.Var2);
hold on;
plot(tau_fit);