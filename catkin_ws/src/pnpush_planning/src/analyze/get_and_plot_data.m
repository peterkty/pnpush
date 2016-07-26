function [object_pose, tip_pose, wrench] = get_and_plot_data(filename, shape_id, do_plot)

data = loadjson(filename);
object_pose = data.object_pose; 
tip_pose = data.tip_pose;
wrench = data.ft_wrench;

if do_plot
    
    figure('position', [275, 500, 1500, 1000]) 
    
    %Plot snapshots object and pusher pose
    subplot(1,2,1); hold on;
    tip_radius = 0.00475;
    gamma = linspace(0,2*pi);
    plot_interval = 50;
    for i = 1:plot_interval:length(tip_pose(:,1))
         plot(tip_radius*cos(gamma)+tip_pose(i,2),tip_radius*sin(gamma)+tip_pose(i,3), 'k'); 
    end
    shape = get_shape(shape_id); 
    for i = 1:plot_interval:length(object_pose(:,1))
        theta = object_pose(i,4);
        rotation_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
        oriented_shape = shape*0;
        for j = 1:size(shape,1)            
            oriented_shape(j,:) = shape(j,:)*rotation_matrix;
        end
        plot(oriented_shape(:,1)+object_pose(i,2), oriented_shape(:,2) + object_pose(i,3), 'r');
    end
    title('Motion of the object and the pusher in the robot frame')
    %Plot force over time
    subplot(1,2,2); hold on;

    plot(wrench(:,1)-wrench(1,1), wrench(:,2), 'b');
    plot(wrench(:,1)-wrench(1,1), wrench(:,3), 'g');
    plot(wrench(:,1)-wrench(1,1), wrench(:,4), 'r');
    legend('x direction', 'y direction', 'z direction')
    title('Force profile over time')
end
end