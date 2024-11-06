function pid_z = crazyflie_get_pid_z(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        pid_result = py_module.get_pid_z(scf);
        pid_z.P = double(pid_result{'P'});
        pid_z.I = double(pid_result{'I'});
        pid_z.D = double(pid_result{'D'});

    catch ME
        error('Error using crazyflie_python_commands>get_pid_z: %s', ME.message);
    end  
end
