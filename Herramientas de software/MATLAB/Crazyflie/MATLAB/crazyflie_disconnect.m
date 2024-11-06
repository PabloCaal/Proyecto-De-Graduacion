function crazyflie_disconnect(scf)
    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.disconnect(scf);
        evalin('base', ['clear ', inputname(1)]);
    catch ME
        error('Error using crazyflie_python_commands>disconnect_crazyflie: %s', ME.message);
    end  
end