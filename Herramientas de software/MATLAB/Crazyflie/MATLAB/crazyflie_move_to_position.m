function crazyflie_move_to_position(scf, x, y, z, v)    
    if ~isnumeric(x) || ~isnumeric(y) || ~isnumeric(z)
        error('ERROR: x, y, and z must be numeric values.');
    end

    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.move_to_position(scf, x, y, z, v);
    catch ME
        error('Error using crazyflie_python_commands>move_to_position: %s', ME.message);
    end  
end