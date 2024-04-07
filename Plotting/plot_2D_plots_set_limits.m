function plot_2D_plots_set_limits(params)

    % Function setting the limits for each plot

    % Scale of plot. 1.0 means the abs max value lies on the edge
    margin = 1.1;

    % trim values
    phi = params.trim.phi;
    u = [params.trim.Omega1; params.trim.Omega2; params.trim.Omega3; params.trim.mu];

    % Show 12 States
    for i = 1:12
        figure(1);
        subplot(6,2,i);
        hold on
        if i ~= 10
            ylim([-max(abs(ylim))*margin, max(abs(ylim))*margin]);
        else
            ylim([phi-max(abs(ylim-phi))*margin, phi+max(abs(ylim-phi))*margin]);
        end
    end

    % Show 4 inputs
    for i = 1:4
        figure(2);
        subplot(4,1,i);
        hold on
        ylim([u(i)-max(abs(ylim-u(i)))*margin, u(i)+max(abs(ylim-u(i)))*margin]);
    end
end