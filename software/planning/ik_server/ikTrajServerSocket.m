
host = 'localhost';
port = 30000;
buffersize = 8196;
tcpipServer = tcpip('localhost', port, 'NetworkRole', 'server');
set(tcpipServer,'OutputBufferSize', buffersize*10);
set(tcpipServer,'InputBufferSize', buffersize);
%set(tcpipServer,'Terminator', char(0));

warning('off','Drake:DisablingSimulinkAutosave')
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
addpath([getenv('PNPUSH_BASE'),'/software/externals/jsonlab-1.0/jsonlab']);  % for parsing json
addpath([getenv('PNPUSH_BASE'),'/software/externals/json']);                 % for writing json
javaaddpath([getenv('PNPUSH_BASE'),'/software/build/share/java/bot2-lcmgl.jar']) % for lcmgl

%% initialize robot here
tic
r = RigidBodyManipulator();
options.urdf_path = [getenv('PNPUSH_BASE'), '/catkin_ws/src/pnpush_config/models/IRB120/irb_120_drake.urdf']; 
options.base_offset = [0, 0, 0]';
options.base_rpy = [0, 0, 0]';   % pi/2 only needed for irb 120
fprintf('Loading the robot urdf: %s\n', options.urdf_path);
r = addRobotFromURDF(r, options.urdf_path, options.base_offset, options.base_rpy);
toc
%%

while true
    try
        fprintf('Waiting for client at %s:%d...\n', host, 30000);
        flushinput(tcpipServer);
        fopen(tcpipServer);
        tic
        fprintf('Connected\n');
        data_json = fscanf(tcpipServer);
        fprintf('Received data: %s\n', data_json);
        ret_json = ikTrajServer_internal(r, data_json, options);
        %fprintf('Returned data: %s\n', ret_json);
        fprintf(tcpipServer, ret_json);
        toc
        fclose(tcpipServer);
    catch me
        fprintf('Error: %s\n', me.getReport());
        fclose(tcpipServer);
    end
end
