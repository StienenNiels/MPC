function visualize_tricopter_trajectory_vid(states_trajectory,control_input,params,variables_struc,pause_duration)
    % plots the trajectory of the tricopter given the 
    % provided states_trajectory and control_inputs
    
    payload = variables_struc.payload;
    payload_time = variables_struc.payload_time/variables_struc.dt;
    trj = variables_struc.trj;

    % X is the 6-states of the tricopter
    X = states_trajectory(:,[10 11 12 4 5 6]);
    X(:,3) = -X(:,3);
    U = control_input(:,4);
    [N,~] = size(X);

    % reference set point (center of plot)
    x_r = 0;
    y_r = 0;
    z_r = 0;

    % tricopter frame and circle drawings
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3; 
    mu = params.trim.mu;
    phi_trim = params.trim.phi;
    rc = 0.1;
    rx = rc*cos(linspace(0,2*pi,20));
    rx = [rx rx(1)];
    ry = rc*sin(linspace(0,2*pi,20));
    ry = [ry ry(1)];

    % Payload mass
    m_payload = params.m_payload;
    P1_acc = 0;
    P1_vel = 0;
    
    % init figure
    fig = figure(42);
    clf;
    
    % define plot axes limits
    w = 2.5;
    wz = 2.5;
    
    Ax = [-w+x_r w+x_r -w+y_r w+y_r 0 wz+z_r];


    % Get the screen size
    screen_size = get(0, 'ScreenSize');
    
    % Set the figure position and size to cover the whole screen
    set(fig, 'Position', [screen_size(1) screen_size(1) screen_size(4) screen_size(4)]);

    % Create a VideoWriter object to save the frames as a video
    video_filename = 'trajectory_video.mp4';
    writerObj = VideoWriter(video_filename, "MPEG-4");
    writerObj.FrameRate = 10; % Adjust the frame rate as needed
    open(writerObj);

    % loop through trajectory inputs
    for j = 1:N/1

        k = j*1;
        % obtain rotational matrix for current RPY angles
        Rt = R(X(k,4:6),phi_trim); 
        Rmu = R_mu(mu,U(k));

        % define each tricopter rotor
        R1 = Rt*([l1;0;0])+Rt*Rmu*([rx ; ry  ; zeros(size(rx)) ]) + X(k,1:3)';
        R2 = Rt*([ -l2+rx ; -l3+ry  ; zeros(size(rx)) ]) + X(k,1:3)';
        R3 = Rt*([ -l2+rx ; l3-ry ; zeros(size(rx)) ]) + X(k,1:3)';
        
        % define black arms
        A1 = Rt*([-l2 l1;0 0;0 0]) + X(k,1:3)';
        A2 = Rt*([-l2 -l2; -l3 l3;0 0]) + X(k,1:3)';

        % define payload
        if payload && k>payload_time
            if P1(3) > 0.1
                P1_acc = -m_payload*params.g;
                P1_vel = P1_acc*0.1 + P1_vel;
                P1 = P1 + [0;0;P1_vel*0.1];
            else
                P1 = [P1(1:2);0];
            end
        else
            P1 = X(k,1:3)';
        end
        
        % % plot coordinate reference
        % plot3( x_r,y_r,z_r,'r.');
        
        
        % plot full desired trajectory and actual trajectory up to k
        plot3(trj(10,:),trj(11,:),-trj(12,:), "LineStyle","--","Color","#666666", "LineWidth",1.2);
        hold on
        plot3(X(1:k,1),X(1:k,2),X(1:k,3), "Color","#7E2F8E","LineWidth",1.2);

        % plot tricopter frame cross and circles
        plot3(A1(1,:),A1(2,:),A1(3,:),'k',A2(1,:),A2(2,:),A2(3,:),'k');
        plot3(R1(1,:),R1(2,:),R1(3,:),'r',R2(1,:),R2(2,:),R2(3,:),'b',R3(1,:),R3(2,:),R3(3,:),'b');
        plot3(P1(1,:),P1(2,:),P1(3,:),'o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
        % set ( gca, 'zdir', 'reverse' )
        set (gca, 'ydir', 'reverse')

        hold off
        grid();
        
        % set axes
        axis(Ax);
        view(3); %3D view
        % view([15 25]);
        % view([0 90]); %Top down view

        % Set axes labels
        xlabel("X [m]",'interpreter','latex')
        ylabel("Y [m]",'interpreter','latex')
        zlabel("Z [m]",'interpreter','latex')
        
        set(gca,'box','on')
        drawnow   
        if (pause_duration > 0) 
            pause(pause_duration);
        end

        % Capture the current frame
        frame = getframe(gcf);
        
        % Write the frame to the video
        writeVideo(writerObj, frame);
    end

    % Close the video file
    close(writerObj);
    
    % Function to determine the rotation matrix for plotting
    function y = R(Xrot,phi_trim)
        
        phi = Xrot(1);
        theta = Xrot(2);
        psi = -Xrot(3);

        Rpsi = [cos(psi)  -sin(psi)   0;
                sin(psi)  cos(psi)    0;
                0         0           1];

        % rotation around y with theta
        Rtheta = [cos(theta)    0       sin(theta);
                  0             1       0;
                 -sin(theta)    0       cos(theta)];

        % rotation around x with phi 
        Rphi = [1       0           0;
                0       cos(phi+phi_trim)    -sin(phi+phi_trim);
                0       sin(phi+phi_trim)    cos(phi+phi_trim)];

        y=Rpsi*Rtheta*Rphi;
    end

    % Function for the extra rotor 1 rotation
    function y = R_mu(mu,U)

        y = [1       0           0;
                0       cos(mu+U)    -sin(mu+U);
                0       sin(mu+U)    cos(mu+U)];
    end
    
end