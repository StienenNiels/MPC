function plot_cost_function(time,Vf,l,inSet)

% Plots the terminal cost Vf and stage cost l over time

ind = find(inSet, 1, 'first');

figure(3);
clf;

plot([time(ind) time(ind)], [-1e-2 -1e7],"LineWidth",2, "LineStyle","--","Color","#77AC30",'DisplayName',"In $$X_f$$ from here");hold on
stairs(time(1:end-1), -l(2:end), "LineWidth",1.2, "LineStyle","-","Color","#0072BD",'DisplayName',"$$l(x,u)$$"); hold on
stairs(time(1:end-1), Vf(2:end)-Vf(1:end-1), "LineWidth",1.2, "LineStyle","-","Color","#D95319",'DisplayName',"$$V(f(x,u))-V(x)$$"); hold on
grid();
ylim([-1e5 -1e-2]);
yscale("log");
ylabel('$Cost$','interpreter','latex');
legend("Location","southeast", 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time [s]', 'Interpreter', 'latex');

end