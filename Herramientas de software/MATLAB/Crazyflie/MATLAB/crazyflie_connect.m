function scf = crazyflie_connect(drone_number)
    if ~isnumeric(drone_number) || mod(drone_number, 1) ~= 0 || drone_number < 1 || drone_number > 12
        error('ERROR: Invalid drone number. It must be an integer between 1 and 12.');
    end

    uris = {
        'radio://0/80/2M/E7E7E7E7E0', % Drone 1
        'radio://0/80/2M/E7E7E7E7E1', % Drone 2
        'radio://0/80/2M/E7E7E7E7E2', % Drone 3
        'radio://0/80/2M/E7E7E7E7E3', % Drone 4
        'radio://0/80/2M/E7E7E7E7E4', % Drone 5
        'radio://0/80/2M/E7E7E7E7E5', % Drone 6
        'radio://0/80/2M/E7E7E7E7E6', % Drone 7
        'radio://0/80/2M/E7E7E7E7E7', % Drone 8
        'radio://0/80/2M/E7E7E7E7D0', % Drone 9
        'radio://0/80/2M/E7E7E7E7D1', % Drone 10
        'radio://0/80/2M/E7E7E7E7D2', % Drone 11
        'radio://0/80/2M/E7E7E7E7D3'  % Drone 12
    };

    uri = uris{drone_number};

    current_folder = fileparts(mfilename('fullpath'));
    python_folder = fullfile(current_folder, '..', 'Crazyflie-Python');
    if count(py.sys.path, python_folder) == 0
        insert(py.sys.path, int32(0), python_folder);
    end

    module_name = 'crazyflie_python_commands'; 
    py_module = py.importlib.import_module(module_name);  
    py.importlib.reload(py_module);  

    try
        scf = py_module.connect(uri); 
    catch ME
        error('Error using crazyflie_python_commands>connect: %s', ME.message);
    end    
end
