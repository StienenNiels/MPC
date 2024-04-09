function plot_L(time, states_trajectory, control_inputs, trim, params, show_full_info)

    % Function plotting the 12 states and 4 inputs of the tricopter
    
    % True for centering around the actual value instead of linearized
    % show_full_info = false;

    % Scale of plot. 1.0 means the abs max value lies on the edge
    margin = 1.5;
    mean_x = max(abs(states_trajectory));
    mean_u = max(abs(control_inputs));
    lw = 1.2; % Linewidth

    % Show 12 States
    figure(1);

    subplot(3,2,1);
    hold on
    stairs(time, states_trajectory(:,10), "LineWidth",lw);  grid();
    ylabel('$X_b$ [m]','interpreter','latex');

    subplot(3,2,3);
    hold on
    stairs(time, states_trajectory(:,11), "LineWidth",lw);  grid();
    ylabel('$Y_b$ [m]','interpreter','latex');

    subplot(3,2,5);
    hold on
    if show_full_info == true
        stairs(time, states_trajectory(:,4)+trim(:,1), "LineWidth",lw);  grid();
    else
        stairs(time, states_trajectory(:,4), "LineWidth",lw);  grid();
    end
    ylabel('$\phi$ [rad]','interpreter','latex');

    xlabel('Time [s]','interpreter','latex');
    
    subplot(3,2,2);
    hold on
    stairs(time, control_inputs(:,1)+trim(:,3), "LineWidth",lw);  grid();
    ylabel('$\Omega_1$ [rpm]','interpreter','latex');

    subplot(3,2,4);
    hold on
    stairs(time, control_inputs(:,2)+trim(:,4), "LineWidth",lw);  grid();
    ylabel('$\Omega_2$ [rpm]','interpreter','latex');

    subplot(3,2,6);
    hold on
    stairs(time, control_inputs(:,4)+trim(:,2), "LineWidth",lw);  grid();
    ylabel('$\mu$ [rad]','interpreter','latex');

    xlabel('Time [s]','interpreter','latex');

    % Show 4 inputs
    show_inputs = false;
    if show_inputs
        figure(2);
        if show_full_info
            subplot 411;
            hold on
            stairs(time, control_inputs(:,1)+trim(:,3), "LineWidth",lw);  grid();
            ylabel('$\Omega_1$ [rpm]','interpreter','latex');
    
            subplot 412;
            hold on
            stairs(time, control_inputs(:,2)+trim(:,4), "LineWidth",lw);  grid();
            ylabel('$\Omega_2$ [rpm]','interpreter','latex');
    
            subplot 413;
            hold on
            stairs(time, control_inputs(:,3)+trim(:,5), "LineWidth",lw);  grid();
            ylabel('$\Omega_3$ [rpm]','interpreter','latex');
    
            subplot 414;
            hold on
            stairs(time, control_inputs(:,4)+trim(:,2), "LineWidth",lw);  grid();
            ylabel('$\mu$ [rad]','interpreter','latex');
        else
            subplot 411;
            hold on
            stairs(time, control_inputs(:,1), "LineWidth",lw);  grid();
            ylabel('$\Omega_1$ [rpm]','interpreter','latex');
    
            subplot 412;
            hold on
            stairs(time, control_inputs(:,2), "LineWidth",lw);  grid();
            ylabel('$\Omega_2$ [rpm]','interpreter','latex');
    
            subplot 413;
            hold on
            stairs(time, control_inputs(:,3), "LineWidth",lw);  grid();
            ylabel('$\Omega_3$ [rpm]','interpreter','latex');
    
            subplot 414;
            hold on
            stairs(time, control_inputs(:,4), "LineWidth",lw);  grid();
            ylabel('$\mu$ [rad]','interpreter','latex');
        end

        xlabel('Time [s]','interpreter','latex');
    end 
end