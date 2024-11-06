function pose = crazyflie_get_pose(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        pose = double(py_module.get_pose(scf));
    catch ME
        error('Error using crazyflie_python_commands>get_pose: %s', ME.message);
    end  
end 