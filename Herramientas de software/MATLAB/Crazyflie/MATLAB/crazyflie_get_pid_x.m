function pid_x = crazyflie_get_pid_x(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        pid_result = py_module.get_pid_x(scf);
        pid_x.P = double(pid_result{'P'});
        pid_x.I = double(pid_result{'I'});
        pid_x.D = double(pid_result{'D'});

    catch ME
        error('Error using crazyflie_python_commands>get_pid_x: %s', ME.message);
    end  
end
