function plot_cost_function(time,Vf,l)

% Plots the terminal cost Vf and stage cost l over time

margin = 1.1;
Vf_max = max(abs(Vf));
l_max = max(abs(l));

figure(3);
clf;

subplot(1,2,1);
stairs(time, Vf, 'b-','DisplayName',"Terminal cost V_f"); hold on
stairs(time(1:end-1), Vf(1:end-1)-Vf(2:end), 'm-','DisplayName',"V_{f(k+1)}-V_{f(k)}");  grid();
% ylim([-Vf_max*margin,Vf_max*margin])
ylim([-50,500])
ylabel('$V_f$','interpreter','latex');
legend()
xlabel('Time [s]');

subplot(1,2,2);
stairs(time, l, 'b-','DisplayName',"Stage cost l"); hold on
stairs(time(1:end-1), l(1:end-1)-l(2:end), 'm-','DisplayName',"l_{(k+1)}-l_{(k)}");  grid();
% ylim([-l_max*margin,l_max*margin])
ylim([-10,100])
ylabel('$l$','interpreter','latex');
legend()
xlabel('Time [s]');

end