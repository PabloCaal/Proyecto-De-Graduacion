function crazyflie_takeoff(scf, height, duration)
    if nargin < 2 || isempty(height)
        height = 0.5;  
    end
    if nargin < 3 || isempty(duration)
        duration = 1.0;  
    end
    
    if ~isnumeric(height) || height <= 0.1
        warning('Height must be greater than 0.1 meters. Using default height of 0.3 meter.');
        height = 0.5;  
    end
    if ~isnumeric(duration) || duration < 1.0
        warning('Duration must be at least 1 second. Using default duration of 2.0 seconds.');
        duration = 1.0;  
    end

    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.takeoff(scf, height, duration);
    catch ME
        error('Error using crazyflie_python_commands>takeoff: %s', ME.message);
    end  
end 