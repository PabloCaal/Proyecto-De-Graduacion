function pid_values = crazyflie_get_pid_values(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);
        
    try
        pid_result = py_module.get_pid_values(scf);
        pid_values.X = [double(pid_result{'X'}{1}), double(pid_result{'X'}{2}), double(pid_result{'X'}{3})];
        pid_values.Y = [double(pid_result{'Y'}{1}), double(pid_result{'Y'}{2}), double(pid_result{'Y'}{3})];
        pid_values.Z = [double(pid_result{'Z'}{1}), double(pid_result{'Z'}{2}), double(pid_result{'Z'}{3})];

    catch ME
        error('Error using crazyflie_python_commands>get_pid_values: %s', ME.message);
    end  

end
