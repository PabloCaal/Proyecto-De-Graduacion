function crazyflie_set_pose(scf, x, y, z, qx, qy, qz, qw)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.set_pose(scf, x, y, z, qx, qy, qz, qw);
    catch ME
        error('Error using crazyflie_python_commands>set_pose: %s', ME.message);
    end  
end 