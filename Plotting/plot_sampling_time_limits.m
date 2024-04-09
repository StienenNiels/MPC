function plot_sampling_time_limits(params)

    % Function setting the limits for each plot

    % Scale of plot. 1.0 means the abs max value lies on the edge
    margin = 1.1;

    % trim values
    phi = params.trim.phi;
    u = [params.trim.Omega1; params.trim.Omega2; params.trim.Omega3; params.trim.mu];

    % Show 12 States
    for i = 1:6
        figure(1);
        subplot(3,2,i);
        hold on
        if i ~= 1
            ylim([-max(abs(ylim))*margin, max(abs(ylim))*margin]);
        else
            ylim([phi-max(abs(ylim-phi))*margin, phi+max(abs(ylim-phi))*margin]);
        end
        xlim([0, 4]);
    end

    % % Show 4 inputs
    % for i = 1:4
    %     figure(2);
    %     subplot(4,1,i);
    %     hold on
    %     ylim([u(i)-max(abs(ylim-u(i)))*margin, u(i)+max(abs(ylim-u(i)))*margin]);
    % end
end