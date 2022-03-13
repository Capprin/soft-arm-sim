% creates mp4 from arm data
function save_arm(joint_vecs,filename,FPS)
    % sanitize
    if ~exist('filename','var')
        filename = 'arm.mp4';
    end
    if ~exist('fps','var')
        fps = 60;
    end
    % precompute points, create saved video
    v = VideoWriter(filename,'MPEG-4');
    v.FrameRate = fps;
    open(v);
    for i = 1:size(joint_vecs,3)
        % generate frame
        plot3(joint_vecs(1,:,i), joint_vecs(2,:,i), joint_vecs(3,:,i), 'Color', 'k',...
              'LineStyle', '-', 'Marker', '.', 'LineWidth', 5, 'MarkerSize', 30);
        frame = getframe(gcf);
        writeVideo(v,frame);
        cla;
    end
    close(v);
end