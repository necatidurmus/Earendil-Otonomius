%% ml_send_waypoints.m — MATLAB'dan ROS2'ye waypoint gönderme örneği
%
% Bu script, mltest ortamı çalışırken MATLAB'dan waypoint gönderir.
% waypoint_receiver.py node'u bu waypoint'leri alıp Nav2'ye iletir.
%
% Kullanım:
%   1. mltest/run_leo.sh veya run_urban.sh vb. çalıştırın
%   2. Bu scripti MATLAB'da çalıştırın

function ml_send_waypoints(world)
    if nargin < 1
        world = 'urban';
    end

    fprintf('\n╔══════════════════════════════════════════╗\n');
    fprintf('║   MATLAB → ROS2 Waypoint Sender          ║\n');
    fprintf('╚══════════════════════════════════════════╝\n\n');

    % ROS2 node oluştur
    node = ros2node('/ml_matlab_sender');

    % Publisher'lar
    wpPub = ros2publisher(node, '/ml_waypoint', 'geometry_msgs/PoseStamped');
    missionPub = ros2publisher(node, '/ml_mission', 'std_msgs/String');
    controlPub = ros2publisher(node, '/ml_control', 'std_msgs/String');

    % Waypoint'leri seç
    wps = getWaypoints(world);
    fprintf('  Dünya: %s\n', world);
    fprintf('  Waypoint sayısı: %d\n\n', size(wps, 1));

    % JSON formatında mission gönder
    jsonStr = '[';
    for i = 1:size(wps, 1)
        if i > 1, jsonStr = [jsonStr, ',']; end
        jsonStr = [jsonStr, sprintf('{"x":%.2f,"y":%.2f,"yaw":%.1f}', ...
            wps(i,1), wps(i,2), wps(i,3))];
    end
    jsonStr = [jsonStr, ']'];

    msg = ros2message(missionPub);
    msg.data = jsonStr;
    send(missionPub, msg);
    fprintf('  ✅ Mission JSON gönderildi (%d waypoint)\n', size(wps, 1));

    % Kontrol komutu gönder
    pause(1);
    ctrlMsg = ros2message(controlPub);
    ctrlMsg.data = 'START';
    send(controlPub, ctrlMsg);
    fprintf('  ✅ START komutu gönderildi\n\n');

    fprintf('  Waypoint''ler sırayla Nav2''ye iletiliyor...\n');
    fprintf('  Durum için: ros2 topic echo /ml_status\n\n');
end

function wps = getWaypoints(world)
    switch world
        case 'leo'
            wps = [0 0 0; -8 -10 0; 8 -10 0; 12 12 0; -10 12 0; 0 0 0];
        case 'urban'
            wps = [0 0 0; 0 8 0; 8 8 0; 8 0 0; 8 -8 0; 0 -8 0; ...
                   -8 -8 0; -8 0 0; 0 0 0];
        case 'industrial'
            wps = [0 0 0; -4 0 0; -4 8 90; 4 8 0; 4 -8 270; -4 -8 180; 0 0 0];
        case 'earendil'
            wps = [0 0 0; 5 5 0; 5 15 0; -5 15 0; -5 5 0; 0 0 0];
        otherwise
            wps = [0 0 0; 5 0 0; 5 5 90; 0 5 180; 0 0 270];
    end
end
