function crazyflie_detect_flow_deck(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    % Disconnect from Crazyflie and clear the scf from the workspace
    % Attempt to disconnect from Crazyflie with error handling
    try
        py_module.detect_flow_deck(scf);
    catch ME
        error('Error using crazyflie_python_commands>detect_flow_deck: %s', ME.message);
    end  
end