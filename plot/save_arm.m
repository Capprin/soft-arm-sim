% creates mp4 from arm data
function save_arm(joint_vecs,filename,FPS)
    cla;
    % sanitize
    if ~exist('filename','var')
        filename = 'arm.mp4';
    end
    if ~exist('fps','var')
        fps = 60;
    end
    % precompute points
    th = linspace(0, 2*pi, 20);
    rad = 0.1;
    cX = rad*cos(th); cY = rad*sin(th); cZ = zeros(size(th))-0.1;
    hold on;
    p = plot3(joint_vecs(1,:,1), joint_vecs(2,:,1), joint_vecs(3,:,1), 'Color', 'k',...
              'LineStyle', '-', 'Marker', '.', 'LineWidth', 5, 'MarkerSize', 30);
    s = scatter3(joint_vecs(1,:,1), joint_vecs(2,:,1), joint_vecs(3,:,1),...
                 100, [1 0 0; 0 1 0; 0 0 1; 0 0 0],'filled');
    c = patch(cX+joint_vecs(1,end,1), cY+joint_vecs(2,end,1), cZ, 'k');;
    patch(cX, cY-3/4, cZ, 'm');
    patch(cX-1/4, cY, cZ, 'c');
    hold off;
    % create saved video
    v = VideoWriter(filename,'MPEG-4');
    v.FrameRate = fps;
    open(v);
    for i = 1:size(joint_vecs,3)
        % generate frame
        set(p,'XData', joint_vecs(1,:,i));
        set(p,'YData', joint_vecs(2,:,i));
        set(p,'ZData', joint_vecs(3,:,i));
        set(s,'XData', joint_vecs(1,:,i));
        set(s,'YData', joint_vecs(2,:,i));
        set(s,'ZData', joint_vecs(3,:,i));
        set(c,'XData', 0.5*cX+joint_vecs(1,end,i));
        set(c,'YData', 0.5*cY+joint_vecs(2,end,i));
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    close(v);
end