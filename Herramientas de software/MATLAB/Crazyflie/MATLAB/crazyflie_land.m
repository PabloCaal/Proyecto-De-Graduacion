function crazyflie_land(scf, height, duration)
    if nargin < 2 || isempty(height)
        height = 0.0; 
    end
    if nargin < 3 || isempty(duration)
        duration = 2.0;  
    end

    if ~isa(scf, 'py.cflib.crazyflie.syncCrazyflie.SyncCrazyflie')
        error('ERROR: Invalid SyncCrazyflie object.');
    end
    
    if ~isnumeric(height) || height < 0.0
        warning('Height must be 0 or greater for landing. Using default height of 0.0 meters.');
        height = 0.0;  
    end
    if ~isnumeric(duration) || duration < 2.0
        warning('Duration must be at least 2 seconds. Using default duration of 2.0 seconds.');
        duration = 2.0; 
    end

    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);

    try
        py_module.land(scf, height, duration);
    catch ME
        error('Error using crazyflie_python_commands>land: %s', ME.message);
    end  
end