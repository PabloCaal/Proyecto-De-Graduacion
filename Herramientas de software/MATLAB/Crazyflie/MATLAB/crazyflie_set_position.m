function crazyflie_set_position(scf, x, y, z)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.set_position(scf, x, y, z);
    catch ME
        error('Error using crazyflie_python_commands>set_position: %s', ME.message);
    end  
end 