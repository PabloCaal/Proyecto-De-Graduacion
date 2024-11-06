function crazyflie_set_pid_values(scf, p_gains, i_gains, d_gains)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_p_gains = py.dict(p_gains);
        py_i_gains = py.dict(i_gains);
        py_d_gains = py.dict(d_gains);
        py_module.set_pid_values(scf, py_p_gains, py_i_gains, py_d_gains);

    catch ME
        error('Error using crazyflie_python_commands>set_pid_values: %s', ME.message);
    end  
end