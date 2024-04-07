function plot_2D_plots_consecutive(time, states_trajectory, control_inputs, trim, params, show_full_info)

    % Function plotting the 12 states and 4 inputs of the tricopter
    
    % True for centering around the actual value instead of linearized
    % show_full_info = false;

    % Scale of plot. 1.0 means the abs max value lies on the edge
    margin = 1.5;
    mean_x = max(abs(states_trajectory));
    mean_u = max(abs(control_inputs));

    % Show 12 States
    show_states = true;
    if show_states
        figure(1);

        subplot(6,2,1);
        hold on
        stairs(time, states_trajectory(:,1));  grid();
        % ylim([-mean_x(1)*margin,mean_x(1)*margin])
        ylabel('$u$ [m/s]','interpreter','latex');

        subplot(6,2,3);
        hold on
        stairs(time, states_trajectory(:,2));  grid();
        % ylim([-mean_x(2)*margin,mean_x(2)*margin])
        ylabel('$v$ [m/s]','interpreter','latex');

        subplot(6,2,5);
        hold on
        stairs(time, states_trajectory(:,3));  grid();
        % ylim([-mean_x(3)*margin,mean_x(3)*margin])
        ylabel('$w$ [m/s]','interpreter','latex');

        subplot(6,2,7);
        hold on
        stairs(time, states_trajectory(:,7));  grid();
        % ylim([-mean_x(7)*margin,mean_x(7)*margin])
        ylabel('$p$ [rad/s]','interpreter','latex');

        subplot(6,2,9);
        hold on
        stairs(time, states_trajectory(:,8));  grid();
        % ylim([-mean_x(8)*margin,mean_x(8)*margin])
        ylabel('$q$ [rad/s]','interpreter','latex');

        subplot(6,2,11);
        hold on
        stairs(time, states_trajectory(:,9));  grid();
        % ylim([-mean_x(9)*margin,mean_x(9)*margin])
        ylabel('$r$ [rad/s]','interpreter','latex');

        xlabel('Time [s]');

        % Or always center around 0 instead of around trim?
        subplot(6,2,8);
        hold on
        if show_full_info == true
            stairs(time, states_trajectory(:,4)+trim(:,1));  grid();
            % ylim([params.trim.phi-mean_x(4)*margin,params.trim.phi+mean_x(4)*margin])
        else
            stairs(time, states_trajectory(:,4));  grid();
            % ylim([-mean_x(4)*margin,mean_x(4)*margin])
        end
        ylabel('$\phi$ [rad]','interpreter','latex');

        subplot(6,2,10);
        hold on
        stairs(time, states_trajectory(:,5));  grid();
        % ylim([-mean_x(5)*margin,mean_x(5)*margin])
        ylabel('$\theta$ [rad]','interpreter','latex');

        subplot(6,2,12);
        hold on
        stairs(time, states_trajectory(:,6));  grid();
        % ylim([-mean_x(6)*margin,mean_x(6)*margin])
        ylabel('$\psi$ [rad]','interpreter','latex');

        xlabel('Time [s]');

        subplot(6,2,2);
        hold on
        stairs(time, states_trajectory(:,10));  grid();
        % ylim([-mean_x(10)*margin,mean_x(10)*margin])
        ylabel('$X_b$ [m]','interpreter','latex');

        subplot(6,2,4);
        hold on
        stairs(time, states_trajectory(:,11));  grid();
        % ylim([-mean_x(11)*margin,mean_x(11)*margin])
        ylabel('$Y_b$ [m]','interpreter','latex');

        subplot(6,2,6);
        hold on
        stairs(time, states_trajectory(:,12));  grid();
        set ( gca, 'ydir', 'reverse' )
        % ylim([-mean_x(12)*margin,mean_x(12)*margin])
        ylabel('$Z_b$ [m]','interpreter','latex');

    end

    % Show 4 inputs
    show_inputs = true;
    if show_inputs
        figure(2);
        if show_full_info
            subplot 411;
            hold on
            stairs(time, control_inputs(:,1)+trim(:,3));  grid();
            % ylim([params.trim.Omega1-mean_u(1)*margin,params.trim.Omega1+mean_u(1)*2*margin])
            ylabel('$\Omega_1$ [rpm]','interpreter','latex');
    
            subplot 412;
            hold on
            stairs(time, control_inputs(:,2)+trim(:,4));  grid();
            % ylim([params.trim.Omega2-mean_u(2)*margin,params.trim.Omega2+mean_u(2)*2*margin])
            ylabel('$\Omega_2$ [rpm]','interpreter','latex');
    
            subplot 413;
            hold on
            stairs(time, control_inputs(:,3)+trim(:,5));  grid();
            % ylim([params.trim.Omega3-mean_u(3)*margin,params.trim.Omega3+mean_u(3)*2*margin])
            ylabel('$\Omega_3$ [rpm]','interpreter','latex');
    
            subplot 414;
            hold on
            stairs(time, control_inputs(:,4)+trim(:,2));  grid();
            % ylim([params.trim.mu-mean_u(4)*margin,params.trim.mu+mean_u(4)*margin])
            ylabel('$\mu$ [rad]','interpreter','latex');
        else
            subplot 411;
            hold on
            stairs(time, control_inputs(:,1));  grid();
            % ylim([-mean_u(1)*margin,mean_u(1)*margin])
            ylabel('$\Omega_1$ [rpm]','interpreter','latex');
    
            subplot 412;
            hold on
            stairs(time, control_inputs(:,2));  grid();
            % ylim([-mean_u(2)*margin,mean_u(2)*margin])
            ylabel('$\Omega_2$ [rpm]','interpreter','latex');
    
            subplot 413;
            hold on
            stairs(time, control_inputs(:,3));  grid();
            % ylim([-mean_u(3)*margin,mean_u(3)*margin])
            ylabel('$\Omega_3$ [rpm]','interpreter','latex');
    
            subplot 414;
            hold on
            stairs(time, control_inputs(:,4));  grid();
            % ylim([-mean_u(4)*margin,mean_u(4)*margin])
            ylabel('$\mu$ [rad]','interpreter','latex');
        end

        xlabel('Time [s]');
    end 
end