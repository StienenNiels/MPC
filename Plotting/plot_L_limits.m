function plot_L_limits(params)

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
        if i ~= 5
            if i == 2
                ylim([u(1)-max(abs(ylim-u(1)))*margin, u(1)+max(abs(ylim-u(1)))*margin]);
            elseif i == 4
                ylim([u(2)-max(abs(ylim-u(2)))*margin, u(2)+max(abs(ylim-u(2)))*margin]);
            elseif i == 6
                ylim([u(4)-max(abs(ylim-u(4)))*margin, u(4)+max(abs(ylim-u(4)))*margin]);
            else
                ylim([-max(abs(ylim))*margin, max(abs(ylim))*margin]);
            end
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