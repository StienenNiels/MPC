function plot_2D_plots(time, states_trajectory, control_inputs)

    % Show 12 States
    show_states = true;
    if show_states
        figure(1);
        clf;

        subplot(6,2,1);
        stairs(time, states_trajectory(:,1), 'm-');  grid();
        ylabel('$u$ [m/s]','interpreter','latex');

        subplot(6,2,3);
        stairs(time, states_trajectory(:,2), 'm-');  grid();
        ylabel('$v$ [m/s]','interpreter','latex');

        subplot(6,2,5);
        stairs(time, states_trajectory(:,3), 'b-');  grid();
        ylabel('$w$ [m/s]','interpreter','latex');

        subplot(6,2,7);
        stairs(time, states_trajectory(:,4), 'b-');  grid();
        ylabel('$p$ [rad/s]','interpreter','latex');

        subplot(6,2,9);
        stairs(time, states_trajectory(:,5), 'k-');  grid();
        ylabel('$q$ [rad/s]','interpreter','latex');

        subplot(6,2,11);
        stairs(time, states_trajectory(:,6), 'k-');  grid();
        ylabel('$r$ [rad/s]','interpreter','latex');

        xlabel('Time [s]');

        subplot(6,2,8);
        stairs(time, states_trajectory(:,7), 'm-');  grid();
        ylabel('$\phi$ [rad]','interpreter','latex');

        subplot(6,2,10);
        stairs(time, states_trajectory(:,8), 'm-');  grid();
        ylabel('$\theta$ [rad]','interpreter','latex');

        subplot(6,2,12);
        stairs(time, states_trajectory(:,9), 'b-');  grid();
        ylabel('$\psi$ [rad]','interpreter','latex');

        xlabel('Time [s]');

        subplot(6,2,2);
        stairs(time, states_trajectory(:,10), 'b-');  grid();
        ylabel('$X_b$ [m]','interpreter','latex');

        subplot(6,2,4);
        stairs(time, states_trajectory(:,11), 'k-');  grid();
        ylabel('$Y_b$ [m]','interpreter','latex');

        subplot(6,2,6);
        stairs(time, states_trajectory(:,12), 'k-');  grid();
        ylabel('$Z_b$ [m]','interpreter','latex');

    end

    % Show 4 inputs
    show_inputs = true;
    if show_inputs
        figure(2);
        clf;

        subplot 411;
        stairs(time, control_inputs(:,1), 'm-');  grid();
        ylabel('$\Omega_1$ [rpm]','interpreter','latex');

        subplot 412;
        stairs(time, control_inputs(:,2), 'm-');  grid();
        ylabel('$\Omega_2$ [rpm]','interpreter','latex');

        subplot 413;
        stairs(time, control_inputs(:,3), 'b-');  grid();
        ylabel('$\Omega_3$ [rpm]','interpreter','latex');

        subplot 414;
        stairs(time, control_inputs(:,4), 'b-');  grid();
        ylabel('$\mu$ [rad]','interpreter','latex');

        xlabel('Time [s]');
    end 
end