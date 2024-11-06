function pid_y = crazyflie_get_pid_y(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        pid_result = py_module.get_pid_y(scf);
        pid_y.P = double(pid_result{'P'});
        pid_y.I = double(pid_result{'I'});
        pid_y.D = double(pid_result{'D'});

    catch ME
        error('Error using crazyflie_python_commands>get_pid_y: %s', ME.message);
    end  
end
