function plot_trj(time, states_trajectory, control_inputs, trim, params)

    % Function plotting the 12 states and 4 inputs of the tricopter
    
    % True for centering around the actual value instead of linearized
    % show_full_info = false;

    % Scale of plot. 1.0 means the abs max value lies on the edge
    margin = 1.5;
    mean_x = max(abs(states_trajectory));
    mean_u = max(abs(control_inputs));
    lw = 1.2; % Linewidth

    % trim values
    phi = params.trim.phi;
    u = [params.trim.Omega1; params.trim.Omega2; params.trim.Omega3; params.trim.mu];

    % Show 12 States
    figure(1), clf;

    subplot(3,2,1);
    hold on
    stairs(time, states_trajectory(:,1), "LineWidth",lw);
    stairs(time, states_trajectory(:,2), "LineWidth",lw);
    stairs(time, states_trajectory(:,3), "LineWidth",lw); grid();
    legend("$u$","$v$","$w$", "Interpreter","latex")
    ylabel('[m/s]','interpreter','latex');
    ylim([-2, 2]);
    xlim([0,27.5]);

    subplot(3,2,2);
    hold on
    stairs(time, states_trajectory(:,7), "LineWidth",lw);
    stairs(time, states_trajectory(:,8), "LineWidth",lw);
    stairs(time, states_trajectory(:,9), "LineWidth",lw); grid();
    legend("$p$","$q$","$r$", "Interpreter","latex")
    ylabel('[rad/s]','interpreter','latex');
    ylim([-1.5, 1.5]);
    xlim([0,27.5]);

    subplot(3,2,3);
    hold on
    stairs(time, states_trajectory(:,10), "LineWidth",lw);
    stairs(time, states_trajectory(:,11), "LineWidth",lw);
    stairs(time, states_trajectory(:,12), "LineWidth",lw); grid();
    legend("$X_b$","$Y_b$","$Z_b$", "Interpreter","latex")
    ylabel('[m]','interpreter','latex');
    ylim([-2.5, 2.5]);
    xlim([0,27.5]);

    subplot(3,2,4);
    hold on
    stairs(time, states_trajectory(:,4)+trim(:,1), "LineWidth",lw); 
    stairs(time, states_trajectory(:,5), "LineWidth",lw);
    stairs(time, states_trajectory(:,6), "LineWidth",lw); grid();
    legend("$\phi$","$\theta$","$\psi$", "Interpreter","latex")
    ylabel('[rad]','interpreter','latex');
    ylim([-3.5, 3.5]);
    xlim([0,27.5]);
    
    subplot(3,2,5);
    hold on
    stairs(time, control_inputs(:,1)+trim(:,3), "LineWidth",lw);
    stairs(time, control_inputs(:,2)+trim(:,4), "LineWidth",lw);
    stairs(time, control_inputs(:,3)+trim(:,5), "LineWidth",lw);grid();
    legend("$\Omega_1$","$\Omega_2$","$\Omega_3$", "Interpreter","latex")
    ylabel('[rpm]','interpreter','latex');
    ylim([1250, 1650]);
    xlim([0,27.5]);
    xlabel('Time [s]','interpreter','latex');

    subplot(3,2,6);
    hold on
    stairs(time, control_inputs(:,4)+trim(:,2), "LineWidth",lw);  grid();
    legend("$\mu$", "Interpreter","latex")
    ylabel('[rad]','interpreter','latex');
    ylim([0.5, 0.55]);
    xlim([0,27.5]);
    xlabel('Time [s]','interpreter','latex');
end