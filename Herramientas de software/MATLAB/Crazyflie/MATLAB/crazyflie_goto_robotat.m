function crazyflie_goto_robotat(crazyflie, x, y, z, velocity, tcp_obj, agent_id)
    
    % Step 1: Move the drone to the specified relative position with the given velocity
    module_name = 'crazyflie_python_commands';
    py_module = py.importlib.import_module(module_name);
    py.importlib.reload(py_module);
    try        
        % Attempt to move the drone to the relative position
        py_module.move_to_position(crazyflie, x, y, z, velocity);
    catch ME
        warning('Error using crazyflie_python_commands>move_to_position');
        return; % Exit the function if an error occurs while moving the drone
    end

    % Step 2: Get the Crazyflie's pose using the TCP object and agent ID
    try
        if (min(agent_id) <= 0 || max(agent_id) > 100)
            error('ERROR: Invalid ID(s).');
        end

        % Prepare the request to get the pose
        s.dst = 1; % DST_ROBOTAT
        s.cmd = 1; % CMD_GET_POSE
        s.pld = round(agent_id);
        write(tcp_obj, uint8(jsonencode(s)));

        % Wait for the server's response with a maximum timeout
        timeout_count = 0;
        timeout_in100ms = 1 / 0.1;
        while (tcp_obj.BytesAvailable == 0 && timeout_count < timeout_in100ms)
            timeout_count = timeout_count + 1;
            pause(0.1);
        end

        if (timeout_count == timeout_in100ms)
            error('ERROR: Failed to receive data from the server.');
        end

        % Read the server's response and decode the data
        mocap_data = jsondecode(char(read(tcp_obj)));
        mocap_data = reshape(mocap_data, [7, numel(agent_id)])';
        % Extract the new position (x, y, z)
        new_x = mocap_data(1, 1);
        new_y = mocap_data(1, 2);
        new_z = mocap_data(1, 3);
    catch ME
        warning('Error obtaining the Crazyflie pose');
        return; % Exit the function if an error occurs while getting the pose
    end

    % Step 3: Update the drone's absolute position
    try
        py_module.set_position(crazyflie, new_x, new_y, new_z);
    catch ME
        warning('Error using crazyflie_python_commands>set_position');
        return; % Exit the function if an error occurs while updating the position
    end
end
