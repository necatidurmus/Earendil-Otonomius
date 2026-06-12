%% 
classdef mission_control_v2_DENEME < matlab.apps.AppBase
    % ╔══════════════════════════════════════════════════════════════╗
    % ║   ZEYNEP MISSION CONTROL SUITE v2.0                         ║
    % ║   Professional ROS2/Nav2 Mission Planning Interface         ║
    % ║                                                              ║
    % ║   Tabs:                                                      ║
    % ║   1. Mission Designer   - Waypoint planning                 ║
    % ║   2. Parameter Tuner    - Nav2 parameters + presets         ║
    % ║   3. Live Monitor       - Real-time tracking                ║
    % ║   4. Results Analyzer   - Test history + PDF reports        ║
    % ║                                                              ║
    % ║   Original features preserved:                              ║
    % ║   - Waypoint x/y/theta input                                ║
    % ║   - CSV/GPS import                                          ║
    % ║   - Save Mission                                            ║
    % ║   - Let's Go Forward                                        ║
    % ║   - Emergency Stop                                          ║
    % ║   - Data Logging                                            ║
    % ║   - Generate Nav2 YAML                                      ║
    % ║   - Real-time velocity/distance gauges                      ║
    % ║   - Obstacle alert                                          ║
    % ╚══════════════════════════════════════════════════════════════╝

    %% ─── PUBLIC PROPERTIES (UI Components) ───
    properties (Access = public)
        % Main window
        UIFigure                matlab.ui.Figure
        TabGroup                matlab.ui.container.TabGroup
        
        % Tabs
        DesignerTab             matlab.ui.container.Tab
        TunerTab                matlab.ui.container.Tab
        MonitorTab              matlab.ui.container.Tab
        AnalyzerTab             matlab.ui.container.Tab
        
        % Status bar (top)
        StatusPanel             matlab.ui.container.Panel
        ROSLamp                 matlab.ui.control.Lamp
        ROSLabel                matlab.ui.control.Label
        ClockLabel              matlab.ui.control.Label
        EStopBtn                matlab.ui.control.Button
        
        % ─── TAB 1: MISSION DESIGNER ───
        InputPanel              matlab.ui.container.Panel
        XField                  matlab.ui.control.NumericEditField
        YField                  matlab.ui.control.NumericEditField
        ThetaField              matlab.ui.control.NumericEditField
        GPSModeSwitch           matlab.ui.control.Switch
        GPSModeLbl              matlab.ui.control.Label
        XLabel                  matlab.ui.control.Label
        YLabel                  matlab.ui.control.Label
        AddBtn                  matlab.ui.control.Button
        
        ActionPanel             matlab.ui.container.Panel
        ImportCSVBtn            matlab.ui.control.Button
        ExportCSVBtn            matlab.ui.control.Button
        SaveMissionBtn          matlab.ui.control.Button
        LoadMissionBtn          matlab.ui.control.Button
        
        WaypointTable           matlab.ui.control.Table
        DeleteWpBtn             matlab.ui.control.Button
        MoveUpBtn               matlab.ui.control.Button
        MoveDownBtn             matlab.ui.control.Button
        ClearAllBtn             matlab.ui.control.Button
        
        MapAxes                 matlab.ui.control.UIAxes
        WaypointCountLbl        matlab.ui.control.Label
        TotalDistLbl            matlab.ui.control.Label
        WorldDropDown           matlab.ui.control.DropDown
        WorldLbl                matlab.ui.control.Label
        SafetyLbl               matlab.ui.control.Label
        RecoverBtn              matlab.ui.control.Button
        
        % ─── Quick Mission Panel (Designer Tab) ───
        QuickMissionPanel       matlab.ui.container.Panel
        ScenarioDropDown        matlab.ui.control.DropDown
        QuickLoadBtn            matlab.ui.control.Button
        AutoParamsBtn           matlab.ui.control.Button
        QuickMissionInfoLbl     matlab.ui.control.Label
        
        % ─── Parameter Sweep Panel (Tuner Tab) ───
        SweepPanel              matlab.ui.container.Panel
        SweepParamDropDown      matlab.ui.control.DropDown
        SweepMinField           matlab.ui.control.NumericEditField
        SweepMaxField           matlab.ui.control.NumericEditField
        SweepStepField          matlab.ui.control.NumericEditField
        SweepPlanBtn            matlab.ui.control.Button
        SweepInfoLbl            matlab.ui.control.Label
        
        % ─── TAB 2: PARAMETER TUNER ───
        PresetPanel             matlab.ui.container.ButtonGroup
        SlowBtn                 matlab.ui.control.RadioButton
        NormalBtn               matlab.ui.control.RadioButton
        FastBtn                 matlab.ui.control.RadioButton
        CustomBtn               matlab.ui.control.RadioButton
        
        MaxVelSlider            matlab.ui.control.Slider
        MaxVelField             matlab.ui.control.NumericEditField
        MaxVelInfo              matlab.ui.control.Label
        
        MaxAccSlider            matlab.ui.control.Slider
        MaxAccField             matlab.ui.control.NumericEditField
        MaxAccInfo              matlab.ui.control.Label
        
        BrakingSlider           matlab.ui.control.Slider
        BrakingField            matlab.ui.control.NumericEditField
        BrakingInfo             matlab.ui.control.Label
        
        SafeDistSlider          matlab.ui.control.Slider
        SafeDistField           matlab.ui.control.NumericEditField
        SafeDistInfo            matlab.ui.control.Label
        
        GoalTolSlider           matlab.ui.control.Slider
        GoalTolField            matlab.ui.control.NumericEditField
        GoalTolInfo             matlab.ui.control.Label
        
        YAMLArea                matlab.ui.control.TextArea
        GenerateYAMLBtn         matlab.ui.control.Button
        SaveYAMLBtn             matlab.ui.control.Button
        
        % ─── TAB 3: LIVE MONITOR ───
        LiveMapAxes             matlab.ui.control.UIAxes
        VelGauge                matlab.ui.control.Gauge
        VelGaugeLbl             matlab.ui.control.Label
        DistGauge               matlab.ui.control.LinearGauge
        DistGaugeLbl            matlab.ui.control.Label
        ObstacleLamp            matlab.ui.control.Lamp
        ObstacleLbl             matlab.ui.control.Label
        NavModeLamp             matlab.ui.control.Lamp
        NavModeLbl              matlab.ui.control.Label
        NavModeValueLbl         matlab.ui.control.Label
        NavModeCountLbl         matlab.ui.control.Label
        LoggingSwitch           matlab.ui.control.Switch
        LoggingSwitchLbl        matlab.ui.control.Label
        LoggingLamp             matlab.ui.control.Lamp
        LoggingLampLbl          matlab.ui.control.Label
        StartMissionBtn         matlab.ui.control.Button
        ResetBtn                matlab.ui.control.Button
        ProgressGauge           matlab.ui.control.LinearGauge
        ProgressLbl             matlab.ui.control.Label
        MissionTimeLbl          matlab.ui.control.Label
        CurrentWPLbl            matlab.ui.control.Label
        
        % ─── TAB 4: RESULTS ANALYZER ───
        HistoryTable            matlab.ui.control.Table
        RefreshHistoryBtn       matlab.ui.control.Button
        LoadResultBtn           matlab.ui.control.Button
        DeleteResultBtn         matlab.ui.control.Button
        AnalysisAxes            matlab.ui.control.UIAxes
        MetricsTable            matlab.ui.control.Table
        GeneratePDFBtn          matlab.ui.control.Button
        ExportExcelBtn          matlab.ui.control.Button
        
        % ─── DATA (Public) ───
        LogData = []
        IsLogging = false
    end
    
    %% ─── PRIVATE PROPERTIES (Internal State) ───
    properties (Access = private)
        % ROS2 objects
        ROSNode
        TFBuffer
        OdomSub
        VelPub
        VelMsg
        GoalPub
        GoalMsg
        NavActionClient
        VelNavPub
        VelNavMsg
        EStopTimer
        EStopActive = false
        RobotX = 0
        RobotY = 0
        CurrentRosTime = []
        LastGoalSentTime = []
        LastPosition = []
        LastProgressTime = []
        
        % Mission state
        IsNavigating = false
        CurrentWaypointIdx = 1
        MissionStartTime
        RobotPlot
        RobotPlotLive
        SonCizimZamani
        OfflineMode = false
        
        % Directories
        ResultsDir = ''
        MissionsDir = ''
        
        % ═══ OBSTACLE DATABASE (per world) ═══
        % Each obstacle: [center_x, center_y, half_width, half_height] in metres.
        % Used to draw obstacles on the map and warn on unsafe waypoints.
        CurrentWorld = 'obstacles'
        SafeClearance = 1.0     % extra margin (m) beyond obstacle half-size before warning
        ObstacleDB              % struct populated in buildObstacleDB()
        MissionLibrary          % struct array — filtered missions for current world
        
        % Sweep state
        SweepValues = []
        SweepParamName = ''
        SweepCurrentIdx = 0
        SweepResults = []
        
        % Clock timer
        ClockTimer
        % Nav mode (GPS/SLAM hibrit)
        NavModeSub
        NavModeTimer
        LastNavMode = ''
        NavModeSwitchCount = 0
        % GPS spoofer control
        GPSSpooferPub
        GPSSpooferMsg
        % Local Odom position
        OdomX = 0
        OdomY = 0
        OdomYaw = 0
        SlamOdomStartX = []
        SlamOdomStartY = []
        SlamOdomStartYaw = []
        TargetX = 0
        TargetY = 0
        TargetFrame = 'map'
        % GPS datum (ROSçunun /fromLL ile kalibre edildi, <2cm hata)
        GPS_LAT0 = 39.925018
        GPS_LON0 = 32.836956
        GPS_YAW_DEG = 36.81
        GPSMode = false
    end
    
    %% ─── HELPER METHODS ───
    methods (Access = private)
        
        function setupDirs(app)
            app.ResultsDir = fullfile(getenv('HOME'), 'zeynep_results');
            app.MissionsDir = fullfile(getenv('HOME'), 'zeynep_missions_deneme');
            if ~exist(app.ResultsDir, 'dir'), mkdir(app.ResultsDir); end
            if ~exist(app.MissionsDir, 'dir'), mkdir(app.MissionsDir); end
        end
        
        function connectROS(app)
            try
                app.ROSNode = ros2node('/zeynep_mc_v2');
                try
                    app.TFBuffer = ros2tf(app.ROSNode);
                catch
                    app.TFBuffer = [];
                end
                app.OdomSub = ros2subscriber(app.ROSNode, '/odom', ...
                    'nav_msgs/Odometry', @app.onOdomCallback);
                app.VelPub = ros2publisher(app.ROSNode, '/cmd_vel', ...
                    'geometry_msgs/Twist');
                app.VelMsg = ros2message(app.VelPub);
                % cmd_vel_nav publisher — emergency stop'ta Nav2'nin komut bastığı
                % topic'e de sıfır hız basmak için (zincir: Nav2→cmd_vel_nav→smoother→cmd_vel).
                try
                    app.VelNavPub = ros2publisher(app.ROSNode, '/cmd_vel_nav', ...
                        'geometry_msgs/Twist');
                    app.VelNavMsg = ros2message(app.VelNavPub);
                catch
                    app.VelNavPub = [];
                end
                app.GoalPub = ros2publisher(app.ROSNode, '/goal_pose', ...
                    'geometry_msgs/PoseStamped');
                app.GoalMsg = ros2message(app.GoalPub);
                
                
                % Nav Mode subscriber (GPS/SLAM hibrit - ROSçunun sisteminden)
                try
                    app.NavModeSub = ros2subscriber(app.ROSNode, '/nav_mode', ...
                        'std_msgs/String');
                catch
                    app.NavModeSub = [];
                end
                
                % GPS Spoofer publisher
                try
                    app.GPSSpooferPub = ros2publisher(app.ROSNode, '/gps_spoofer/control', ...
                        'std_msgs/String');
                    app.GPSSpooferMsg = ros2message(app.GPSSpooferPub);
                catch
                    app.GPSSpooferPub = [];
                    app.GPSSpooferMsg = [];
                end
                
                % GERÇEK BAĞLANTIYI TEST ET - 3 saniye içinde /odom verisi gelmeli
                try
                    receive(app.OdomSub, 3);  % 3 saniye timeout
                    app.OfflineMode = false;
                    app.ROSLamp.Color = 'green';
                    app.ROSLabel.Text = 'ROS2: Connected ✓';
                catch
                    % Topic var ama veri gelmiyor (Gazebo açık değil)
                    app.OfflineMode = true;
                    app.ROSLamp.Color = [1 0.6 0];  % Turuncu
                    app.ROSLabel.Text = 'ROS2: No data (Is Gazebo running?)';
                end
            catch ME
                app.OfflineMode = true;
                app.ROSLamp.Color = 'red';
                app.ROSLabel.Text = 'ROS2: Offline ✗';
                uialert(app.UIFigure, ...
                    sprintf('ROS2 not available.\n\n%s', ME.message), ...
                    'Connection', 'Icon', 'warning');
            end
        end
        
        function applyPreset(app, preset)
            switch preset
                case 'Slow'
                    app.MaxVelField.Value = 0.3;
                    app.MaxAccField.Value = 0.2;
                    app.BrakingField.Value = 0.8;
                    app.SafeDistField.Value = 0.90;    % Güvenli uzak mesafe
                    app.GoalTolField.Value = 0.5;
                case 'Normal'
                    app.MaxVelField.Value = 0.5;
                    app.MaxAccField.Value = 0.3;
                    app.BrakingField.Value = 0.5;
                    app.SafeDistField.Value = 0.85;    % Engellere çarpmayı önler
                    app.GoalTolField.Value = 0.3;
                case 'Fast'
                    app.MaxVelField.Value = 1.0;
                    app.MaxAccField.Value = 0.6;
                    app.BrakingField.Value = 0.3;
                    app.SafeDistField.Value = 0.75;    % Hızlı ama yine de güvenli
                    app.GoalTolField.Value = 0.2;
            end
            app.syncSliders();
            app.updateYAML();
        end
        
        function syncSliders(app)
            app.MaxVelSlider.Value = app.MaxVelField.Value;
            app.MaxAccSlider.Value = app.MaxAccField.Value;
            app.BrakingSlider.Value = app.BrakingField.Value;
            app.SafeDistSlider.Value = app.SafeDistField.Value;
            app.GoalTolSlider.Value = app.GoalTolField.Value;
        end
        
        function updateYAML(app)
            wp = app.WaypointTable.Data;
            txt = sprintf('nav2_mission:\n');
            txt = [txt, sprintf('  ros__parameters:\n')];
            txt = [txt, sprintf('    max_velocity: %.2f\n', app.MaxVelField.Value)];
            txt = [txt, sprintf('    max_acceleration: %.2f\n', app.MaxAccField.Value)];
            txt = [txt, sprintf('    braking_reaction_time: %.2f\n', app.BrakingField.Value)];
            txt = [txt, sprintf('    min_safe_distance: %.2f\n', app.SafeDistField.Value)];
            txt = [txt, sprintf('    goal_tolerance: %.2f\n', app.GoalTolField.Value)];
            txt = [txt, sprintf('    waypoints:\n')];
            if ~isempty(wp)
                for i = 1:size(wp,1)
                    txt = [txt, sprintf('      - [%.2f, %.2f, %.2f]\n', wp(i,1), wp(i,2), wp(i,3))];
                end
            end
            app.YAMLArea.Value = txt;
        end
        
        function buildObstacleDB(app)
            % ═══ Engel veritabanı — her dünya için ═══
            % Obstacle = [cx, cy, half_w, half_h]; tunnel walls handled separately.
            % xlim/ylim: harita eksen sınırları (her dünyaya özel).
            db = struct();

            % ── obstacles world (leo_obstacles.sdf) — GERÇEK düzen ──
            % 14 kutu, her biri 2×2m (half=1.0). wall_north(y=17), wall_east(x=17).
            % Tünel x=20-30 y=±2 (duvarlar ±2), kapı x=35.
            db.obstacles.boxes = [ ...
                 6,   4, 1.0, 1.0; ...   box_01
                -7,   4, 1.0, 1.0; ...   box_02
                10,  -3, 1.0, 1.0; ...   box_03
               -10,  -5, 1.0, 1.0; ...   box_04
                 3,  -8, 1.0, 1.0; ...   box_05
                 9,  10, 1.0, 1.0; ...   box_06
                -5, -12, 1.0, 1.0; ...   box_07
                13,   7, 1.0, 1.0; ...   box_08
               -12,  10, 1.0, 1.0; ...   box_09
                 6,  4, 1.0, 1.0; ...   box_01  (SDF: 6,4)
               -7,  4, 1.0, 1.0; ...   box_02  (SDF: -7,4)
               10, -3, 1.0, 1.0; ...   box_03  (SDF: 10,-3)
              -10, -5, 1.0, 1.0; ...   box_04  (SDF: -10,-5)
                3, -8, 1.0, 1.0; ...   box_05  (SDF: 3,-8)
                9, 10, 1.0, 1.0; ...   box_06  (SDF: 9,10)
               -5,-12, 1.0, 1.0; ...   box_07  (SDF: -5,-12)
               13,  7, 1.0, 1.0; ...   box_08  (SDF: 13,7)
              -12, 10, 1.0, 1.0; ...   box_09  (SDF: -12,10)
                0, 10, 1.0, 1.0; ...   box_10  (SDF: 0,10)
               -3,  6, 1.0, 1.0; ...   box_11  (SDF: -3,6)
                6,-12, 1.0, 1.0; ...   box_12  (SDF: 6,-12)
              -14,  0, 1.0, 1.0; ...   box_13  (SDF: -14,0)
               14, -8, 1.0, 1.0; ...   box_14  (SDF: 14,-8)
                0, 17, 10.0, 0.2; ...  wall_north
               17,  0, 0.2, 8.0];      % wall_east
            db.obstacles.tunnel = [20, 30, 1.85];  % SDF: x=20-30, duvar y=±2.0 (et:0.15)
            db.obstacles.spawn  = [0, 0];
            db.obstacles.xlim   = [-20 35];
            db.obstacles.ylim   = [-20 20];

            % ── empty world (leo_empty.sdf) — engel yok ──
            db.empty.boxes  = zeros(0,4);
            db.empty.tunnel = [];
            db.empty.spawn  = [0, 0];
            db.empty.xlim   = [-25 25];
            db.empty.ylim   = [-25 25];

            % ── urban world (leo_urban.sdf) — binalar grid (yaklaşık) ──
            blds = [];
            for bx = [-12, 0, 12]
                for by = [-12, 0, 12]
                    if abs(bx)<3 && abs(by)<3, continue; end
                    blds = [blds; bx, by, 3.0, 3.0]; %#ok<AGROW>
                end
            end
            db.urban.boxes  = blds;
            db.urban.tunnel = [];
            db.urban.spawn  = [0, 0];
            db.urban.xlim   = [-25 25];
            db.urban.ylim   = [-25 25];

            % ── industrial world (leo_industrial.sdf) — raflar/koridorlar ──
            db.industrial.boxes = [ ...
                -15,  15, 0.5, 4.0;  -15,  5, 0.5, 4.0;  -15, -5, 0.5, 4.0;  -15, -15, 0.5, 4.0; ...
                 -5,  15, 0.5, 4.0;   -5,  5, 0.5, 4.0;   -5, -5, 0.5, 4.0;   -5, -15, 0.5, 4.0; ...
                  5,  15, 0.5, 4.0;    5,  5, 0.5, 4.0;    5, -5, 0.5, 4.0;    5, -15, 0.5, 4.0; ...
                 15,  15, 0.5, 4.0;   15,  5, 0.5, 4.0;   15, -5, 0.5, 4.0;   15, -15, 0.5, 4.0];
            db.industrial.tunnel = [];
            db.industrial.spawn  = [0, 0];
            db.industrial.xlim   = [-25 25];
            db.industrial.ylim   = [-25 25];

            % ── open_terrain / sloped_terrain — engelsiz (tepe/eğim, çarpışma yok) ──
            db.open_terrain.boxes  = zeros(0,4);
            db.open_terrain.tunnel = [];
            db.open_terrain.spawn  = [0, 0];
            db.open_terrain.xlim   = [-25 25];
            db.open_terrain.ylim   = [-25 25];
            db.sloped_terrain.boxes  = zeros(0,4);
            db.sloped_terrain.tunnel = [];
            db.sloped_terrain.spawn  = [0, 0];
            db.sloped_terrain.xlim   = [-25 25];
            db.sloped_terrain.ylim   = [-25 25];

            % ── earendil_env — mesh ortam, kraterler (yaklaşık no-go) ──
            db.earendil_env.boxes  = [ ...
                -9, -21, 2.0, 2.0; ...    % crater approach
                 9,  30, 1.5, 1.5];       % antenna
            db.earendil_env.tunnel = [];
            db.earendil_env.spawn  = [0, 0];
            db.earendil_env.xlim   = [-25 25];
            db.earendil_env.ylim   = [-30 40];

            % ══════════════════════════════════════════════════════════
            % ══  CLEARPATH DÜNYALARI (SDF dosyalarından çıkarıldı)  ══
            % ══════════════════════════════════════════════════════════

            % ── clearpath_warehouse (clearpath_warehouse_simple.sdf) ──
            % 40x30m bina — duvarlar, raflar, masalar, bariyerler, paletler, forklift
            db.clearpath_warehouse.boxes = [ ...
                  0,  15, 20.0, 0.15; ...   % wall_north (40m x 0.3m)
                  0, -15, 20.0, 0.15; ...   % wall_south
                 20,   0, 0.15, 15.0; ...   % wall_east (0.3m x 30m)
                -20,   0, 0.15, 15.0; ...   % wall_west
                -12,   8,  3.0,  0.5; ...   % shelf_1 (6x1m)
                 12,   8,  3.0,  0.5; ...   % shelf_2
                  0,   0,  3.0,  0.5; ...   % shelf_3
                -12,  -8,  3.0,  0.5; ...   % shelf_4
                 12,  -8,  3.0,  0.5; ...   % shelf_5
                 15,  12,  1.0,  0.5; ...   % table_1 (2x1m)
                 15, -12,  1.0,  0.5; ...   % table_2
                 16,  12, 0.25, 0.25; ...   % chair_1 (0.5x0.5m)
                 14,  12, 0.25, 0.25; ...   % chair_2
                 -5,   5, 0.25,  1.5; ...   % barrier_1 (90° rotated: 3x0.5→0.5x3)
                 -5,  -5, 0.25,  1.5; ...   % barrier_2
                -15,   0,  0.6,  0.6; ...   % pallet_1+box_1 (1.2x1.2m)
                -15,  -3,  0.6,  0.6; ...   % pallet_2+box_2
                  5,  10, 0.75,  1.0];      % forklift (1.5x2m)
            db.clearpath_warehouse.tunnel = [];
            db.clearpath_warehouse.spawn  = [0, 0];
            db.clearpath_warehouse.xlim   = [-25 25];
            db.clearpath_warehouse.ylim   = [-20 20];

            % ── clearpath_office (clearpath_office.sdf) ──
            % Mesh-based ofis binası + charge_dock
            % Mesh engelleri yaklaşık olarak kutu ile ifade edilemez,
            % charge_dock pozisyonunu engel olarak ekle.
            db.clearpath_office.boxes = [ ...
                 23,  -5, 0.5, 0.5];       % charge_dock
            db.clearpath_office.tunnel = [];
            db.clearpath_office.spawn  = [0, 0];
            db.clearpath_office.xlim   = [-15 30];
            db.clearpath_office.ylim   = [-15 15];

            % ── clearpath_orchard (clearpath_orchard.sdf) ──
            % Mesh-based bahçe + ağaç sıraları (offset -10,-10), base_station
            db.clearpath_orchard.boxes = [ ...
                 -8,  -8, 0.3, 0.3];       % base_station
            db.clearpath_orchard.tunnel = [];
            db.clearpath_orchard.spawn  = [0, 0];
            db.clearpath_orchard.xlim   = [-25 25];
            db.clearpath_orchard.ylim   = [-25 25];

            % ── clearpath_pipeline (clearpath_pipeline.sdf) ──
            % Pipeline mesh (0,20, scale 2x), base_station (0,8)
            db.clearpath_pipeline.boxes = [ ...
                  0,   8, 0.3, 0.3];       % base_station
            db.clearpath_pipeline.tunnel = [];
            db.clearpath_pipeline.spawn  = [0, 0];
            db.clearpath_pipeline.xlim   = [-25 30];
            db.clearpath_pipeline.ylim   = [-10 35];

            % ── clearpath_solar_farm (clearpath_solar_farm.sdf) ──
            % Mesh-based tarla, base_station (-30,-10), charge_dock (-4.5,-4)
            db.clearpath_solar_farm.boxes = [ ...
                -30, -10, 0.3, 0.3; ...    % base_station
                 -4.5, -4, 0.5, 0.5];     % charge_dock
            db.clearpath_solar_farm.tunnel = [];
            db.clearpath_solar_farm.spawn  = [0, 0];
            db.clearpath_solar_farm.xlim   = [-40 25];
            db.clearpath_solar_farm.ylim   = [-20 20];

            % ── clearpath_construction (clearpath_construction.sdf) ──
            % Mesh-based inşaat alanı + charge_dock (17,-2)
            db.clearpath_construction.boxes = [ ...
                 17,  -2, 0.5, 0.5];       % charge_dock
            db.clearpath_construction.tunnel = [];
            db.clearpath_construction.spawn  = [0, 0];
            db.clearpath_construction.xlim   = [-15 25];
            db.clearpath_construction.ylim   = [-15 15];

            app.ObstacleDB = db;
        end

        function drawObstacles(app, ax)
            % Seçili dünyanın engellerini verilen eksene çizer (arka plan).
            if isempty(app.ObstacleDB), app.buildObstacleDB(); end
            w = app.CurrentWorld;
            if ~isfield(app.ObstacleDB, w), return; end
            od = app.ObstacleDB.(w);

            % engel kutuları (gri dolgu + koyu kenar)
            if ~isempty(od.boxes)
                for i = 1:size(od.boxes,1)
                    cx = od.boxes(i,1); cy = od.boxes(i,2);
                    hw = od.boxes(i,3); hh = od.boxes(i,4);
                    % güvenlik halkası (açık kırmızı, şeffaf)
                    rectangle(ax, 'Position', [cx-hw-app.SafeClearance, cy-hh-app.SafeClearance, ...
                        2*(hw+app.SafeClearance), 2*(hh+app.SafeClearance)], ...
                        'Curvature', 0.3, 'EdgeColor', [0.95 0.6 0.6], ...
                        'LineStyle', ':', 'LineWidth', 1);
                    % engel gövdesi (gri)
                    rectangle(ax, 'Position', [cx-hw, cy-hh, 2*hw, 2*hh], ...
                        'FaceColor', [0.45 0.45 0.5], 'EdgeColor', [0.2 0.2 0.2], ...
                        'LineWidth', 1.5);
                end
            end

            % tünel duvarları (turuncu çizgiler) + GPS-denied bölge gölgesi
            if ~isempty(od.tunnel)
                xs = od.tunnel(1); xe = od.tunnel(2); hg = od.tunnel(3);
                % GPS-denied bölge (açık sarı dolgu)
                patch(ax, [xs xe xe xs], [-hg -hg hg hg], [1 0.95 0.7], ...
                    'EdgeColor', 'none', 'FaceAlpha', 0.4);
                % duvarlar
                line(ax, [xs xe], [ hg  hg], 'Color', [0.9 0.5 0.1], 'LineWidth', 3);
                line(ax, [xs xe], [-hg -hg], 'Color', [0.9 0.5 0.1], 'LineWidth', 3);
                text(ax, (xs+xe)/2, 0, 'TUNNEL', 'Color', [0.6 0.3 0], ...
                    'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            end

            % spawn noktası (mavi daire)
            if isfield(od, 'spawn') && ~isempty(od.spawn)
                plot(ax, od.spawn(1), od.spawn(2), 'o', 'MarkerSize', 10, ...
                    'MarkerEdgeColor', [0.1 0.3 0.8], 'LineWidth', 2, ...
                    'MarkerFaceColor', 'none');
            end
        end

        function [safe, mindist, who] = checkWaypointSafe(app, x, y)
            % Bir (x,y) noktası engellere göre güvenli mi? mindist = en yakın engele uzaklık.
            if isempty(app.ObstacleDB), app.buildObstacleDB(); end
            safe = true; mindist = inf; who = '';
            w = app.CurrentWorld;
            if ~isfield(app.ObstacleDB, w), return; end
            od = app.ObstacleDB.(w);

            % kutulara mesafe (kutu yüzeyine en yakın nokta)
            if ~isempty(od.boxes)
                for i = 1:size(od.boxes,1)
                    cx = od.boxes(i,1); cy = od.boxes(i,2);
                    hw = od.boxes(i,3); hh = od.boxes(i,4);
                    dx = max([cx-hw - x, x - (cx+hw), 0]);
                    dy = max([cy-hh - y, y - (cy+hh), 0]);
                    d = hypot(dx, dy);
                    if d < mindist, mindist = d; who = sprintf('box@(%.0f,%.0f)', cx, cy); end
                end
            end

            % tünel duvarlarına mesafe
            if ~isempty(od.tunnel)
                xs = od.tunnel(1); xe = od.tunnel(2); hg = od.tunnel(3);
                if x > xs-1 && x < xe+1
                    if abs(y) > hg
                        % Duvarın TAMAMEN DIŞINDA = gerçekten çarpışma
                        dwall = 0;
                    else
                        % Tünel geçidi İÇİNDE — duvara olan mesafeyi hesapla
                        % (hg - |y|) = duvara kalan mesafe
                        dwall = hg - abs(y);
                    end
                    if dwall < mindist, mindist = dwall; who = 'tunnel wall'; end
                end
            end

            if mindist < app.SafeClearance
                safe = false;
            end
        end

        function onWorldChanged(app, ~)
            % Dünya dropdown'u değişince haritayı yenile.
            map = containers.Map( ...
                {'Obstacles / Tunnel (hybrid)','Empty','Urban','Industrial', ...
                 'Open Terrain','Sloped Terrain','Earendil Env', ...
                 'Clearpath Warehouse','Clearpath Office','Clearpath Orchard', ...
                 'Clearpath Pipeline','Clearpath Solar Farm','Clearpath Construction'}, ...
                {'obstacles','empty','urban','industrial', ...
                 'open_terrain','sloped_terrain','earendil_env', ...
                 'clearpath_warehouse','clearpath_office','clearpath_orchard', ...
                 'clearpath_pipeline','clearpath_solar_farm','clearpath_construction'});
            app.CurrentWorld = map(app.WorldDropDown.Value);
            app.updateMap();
            app.loadMissionLibrary();
        end

        function onRecoverRobot(app, ~)
            % ═══ ROBOTU KURTAR — spawn'a (0,0) geri gönder ═══
            % Robot engele takıldıysa tek tıkla kurtarır.
            if app.OfflineMode
                uialert(app.UIFigure, 'ROS bağlı değil — kurtarma yapılamaz.', 'Offline');
                return;
            end
            % önce mevcut görevi durdur
            app.IsNavigating = false;
            app.EStopActive = false;
            if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                stop(app.EStopTimer); delete(app.EStopTimer); app.EStopTimer = [];
            end

            od = app.ObstacleDB.(app.CurrentWorld);
            sx = od.spawn(1); sy = od.spawn(2);

            % Nav2'ye spawn'ı hedef ver — robot geri sürülür
            try
                app.GoalMsg.header.frame_id = 'map';
                app.GoalMsg.pose.position.x = sx;
                app.GoalMsg.pose.position.y = sy;
                app.GoalMsg.pose.position.z = 0;
                app.GoalMsg.pose.orientation.w = 1;
                app.GoalMsg.pose.orientation.x = 0;
                app.GoalMsg.pose.orientation.y = 0;
                app.GoalMsg.pose.orientation.z = 0;
                send(app.GoalPub, app.GoalMsg);
                app.MissionTimeLbl.Text = '🚑 Recovering to spawn...';
                app.MissionTimeLbl.FontColor = [0.1 0.5 0.8];
                uialert(app.UIFigure, ...
                    ['🚑 Kurtarma başladı.' newline newline ...
                     sprintf('Robot spawn''a (%.0f, %.0f) yönlendiriliyor.', sx, sy) newline ...
                     'Vardığında yeni görev verebilirsin.'], ...
                    'Recovery', 'Icon', 'info');
            catch ME
                uialert(app.UIFigure, ['Kurtarma hatası: ' ME.message], 'Error');
            end
        end

        function applyWorldAxisLimits(app, ax)
            % Her dünya için ObstacleDB'deki xlim/ylim değerlerini uygula.
            if isempty(app.ObstacleDB), app.buildObstacleDB(); end
            w = app.CurrentWorld;
            if isfield(app.ObstacleDB, w) && isfield(app.ObstacleDB.(w), 'xlim')
                xlim(ax, app.ObstacleDB.(w).xlim);
                ylim(ax, app.ObstacleDB.(w).ylim);
            else
                xlim(ax, [-25 25]);
                ylim(ax, [-25 25]);
            end
        end

        function updateMap(app)
            cla(app.MapAxes);
            grid(app.MapAxes, 'on');
            title(app.MapAxes, sprintf('Mission Route  [%s]', app.CurrentWorld));
            xlabel(app.MapAxes, 'X (m)');
            ylabel(app.MapAxes, 'Y (m)');
            hold(app.MapAxes, 'on');

            % ═══ ENGELLERİ ÇİZ (her zaman, waypoint olmasa bile) ═══
            app.drawObstacles(app.MapAxes);

            if isempty(app.WaypointTable.Data)
                app.WaypointCountLbl.Text = 'Waypoints: 0';
                app.TotalDistLbl.Text = 'Total Distance: 0.00 m';
                axis(app.MapAxes, 'equal');
                app.applyWorldAxisLimits(app.MapAxes);
                hold(app.MapAxes, 'off');
                return;
            end
            
            data = app.WaypointTable.Data;
            x = data(:,1);
            y = data(:,2);
            
            plot(app.MapAxes, x, y, '-', 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);
            plot(app.MapAxes, x, y, 'o', 'MarkerSize', 10, ...
                'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerEdgeColor', 'w', 'LineWidth', 2);
            
            for i = 1:length(x)
                text(app.MapAxes, x(i)+0.15, y(i)+0.15, sprintf('WP%d', i), ...
                    'FontWeight', 'bold', 'FontSize', 10, 'Color', 'r');
                
                t = deg2rad(data(i,3));
                quiver(app.MapAxes, x(i), y(i), 0.3*cos(t), 0.3*sin(t), ...
                    'MaxHeadSize', 2, 'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);

                % ═══ Güvensiz waypoint'i kırmızı halka ile işaretle ═══
                [safe, ~, ~] = app.checkWaypointSafe(x(i), y(i));
                if ~safe
                    plot(app.MapAxes, x(i), y(i), 'o', 'MarkerSize', 18, ...
                        'MarkerEdgeColor', [0.9 0 0], 'LineWidth', 2.5, ...
                        'MarkerFaceColor', 'none');
                end
            end
            
            % Başlangıç ve bitiş özel işaretle
            plot(app.MapAxes, x(1), y(1), 's', 'MarkerSize', 14, ...
                'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
            if length(x) > 1
                plot(app.MapAxes, x(end), y(end), 'p', 'MarkerSize', 16, ...
                    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
            end
            
            hold(app.MapAxes, 'off');
            axis(app.MapAxes, 'equal');
            app.applyWorldAxisLimits(app.MapAxes);
            
            % İstatistikler
            n = size(data, 1);
            total = 0;
            for i = 2:n
                total = total + sqrt((data(i,1)-data(i-1,1))^2 + (data(i,2)-data(i-1,2))^2);
            end
            app.WaypointCountLbl.Text = sprintf('Waypoints: %d', n);
            app.TotalDistLbl.Text = sprintf('Total Distance: %.2f m', total);
        end
        
        function onOdomCallback(app, msg)
            app.CurrentRosTime = msg.header.stamp;
            if ~isempty(app.SonCizimZamani) && toc(app.SonCizimZamani) < 0.1
                return;
            end
            app.SonCizimZamani = tic;
            
            vel = msg.twist.twist.linear.x;
            
            % Extract robot position in global 'map' frame using TF
            x_map = []; y_map = [];
            if ~isempty(app.TFBuffer)
                try
                    t_tf = getTransform(app.TFBuffer, 'map', 'base_footprint', 'Timeout', 0.05);
                    x_map = t_tf.transform.translation.x;
                    y_map = t_tf.transform.translation.y;
                catch
                end
            end
            if isempty(x_map)
                x = msg.pose.pose.position.x;
                y = msg.pose.pose.position.y;
            else
                x = x_map;
                y = y_map;
            end
            app.RobotX = x;
            app.RobotY = y;
            
            % Save raw odom values for local navigation calculations
            app.OdomX = msg.pose.pose.position.x;
            app.OdomY = msg.pose.pose.position.y;
            qo = msg.pose.pose.orientation;
            app.OdomYaw = atan2(2*(qo.w*qo.z + qo.x*qo.y), 1 - 2*(qo.y*qo.y + qo.z*qo.z));
            
            % Gauge update
            app.VelGauge.Value = abs(vel);
            
            % Obstacle alert
            if abs(vel) < 0.01
                app.ObstacleLamp.Color = 'red';
                app.ObstacleLbl.Text = 'STOPPED';
            else
                app.ObstacleLamp.Color = 'green';
                app.ObstacleLbl.Text = 'MOVING';
            end
            
            % Navigation control
            if app.IsNavigating && ~isempty(app.WaypointTable.Data)
                if ~isempty(app.TargetFrame) && strcmp(app.TargetFrame, 'odom')
                    dist = sqrt((app.TargetX - app.OdomX)^2 + (app.TargetY - app.OdomY)^2);
                else
                    hx = app.WaypointTable.Data(app.CurrentWaypointIdx, 1);
                    hy = app.WaypointTable.Data(app.CurrentWaypointIdx, 2);
                    dist = sqrt((hx-x)^2 + (hy-y)^2);
                end
                app.DistGauge.Value = min(dist, 5);
                
                total_wp = size(app.WaypointTable.Data, 1);
                progress = (app.CurrentWaypointIdx - 1) / total_wp * 100;
                app.ProgressGauge.Value = progress;
                app.CurrentWPLbl.Text = sprintf('WP %d/%d', app.CurrentWaypointIdx, total_wp);
                
                elapsed = toc(app.MissionStartTime);
                app.MissionTimeLbl.Text = sprintf('Mission Time: %.1f s', elapsed);
                
                % --- Stuck Detection & Auto Goal Resend ---
                if isempty(app.LastPosition)
                    app.LastPosition = [x, y];
                    app.LastProgressTime = tic;
                end
                
                moved = sqrt((x - app.LastPosition(1))^2 + (y - app.LastPosition(2))^2);
                if moved > 0.15
                    app.LastPosition = [x, y];
                    app.LastProgressTime = tic;
                elseif toc(app.LastProgressTime) > 15.0
                    % Stuck timeout! Skip/Transition to next waypoint
                    fprintf('⚠️ WP %d Stuck Timeout! Skipping to next waypoint...\n', app.CurrentWaypointIdx);
                    app.CurrentWaypointIdx = app.CurrentWaypointIdx + 1;
                    if app.CurrentWaypointIdx <= total_wp
                        app.sendNextWaypoint();
                        app.LastPosition = [x, y];
                        app.LastProgressTime = tic;
                    else
                        app.IsNavigating = false;
                        app.ProgressGauge.Value = 100;
                        app.CurrentWPLbl.Text = sprintf('WP %d/%d ✓', total_wp, total_wp);
                        app.MissionTimeLbl.Text = sprintf('✓ Mission Complete: %.1f s', elapsed);
                        app.MissionTimeLbl.FontColor = 'green';
                        app.saveMissionResult(elapsed);
                        
                        % Intercept if Parameter Sweep is active
                        if ~isempty(app.SweepCurrentIdx) && app.SweepCurrentIdx > 0 && app.SweepCurrentIdx <= length(app.SweepValues)
                            val = app.SweepValues(app.SweepCurrentIdx);
                            runRes = struct('param_val', val, 'elapsed', elapsed);
                            if isempty(app.SweepResults)
                                app.SweepResults = runRes;
                            else
                                app.SweepResults(end+1) = runRes;
                            end
                            
                            app.SweepCurrentIdx = app.SweepCurrentIdx + 1;
                            if app.SweepCurrentIdx <= length(app.SweepValues)
                                nextVal = app.SweepValues(app.SweepCurrentIdx);
                                app.applySweptParameter(nextVal);
                                
                                msg = sprintf('Run %d/%d completed.\nParameter value tested: %.2f (Time: %.1f s)\n\nPlease reset/recover the robot in Gazebo, then click OK to start Run %d (Parameter = %.2f).', ...
                                    app.SweepCurrentIdx-1, length(app.SweepValues), val, elapsed, app.SweepCurrentIdx, nextVal);
                                uialert(app.UIFigure, msg, 'Sweep Progress', 'Icon', 'info', ...
                                    'CloseFcn', @(~,~) app.onStartMission());
                            else
                                app.SweepCurrentIdx = 0;
                                app.showSweepReport();
                            end
                        else
                            app.showMissionReport(elapsed, total_wp);
                        end
                    end
                    return;
                end
                
                % Auto-resend goal if stationary and > 2s elapsed since last send
                if abs(vel) < 0.02 && (isempty(app.LastGoalSentTime) || toc(app.LastGoalSentTime) > 2.0)
                    app.sendNextWaypoint();
                end
                % ------------------------------------------
                
                % Waypoint'e varış kontrolü:
                %  (a) mesafe tolerans içinde, VEYA
                %  (b) robot durmuş (Nav2 kendi toleransıyla "vardım" demiş) ve
                %      hedefe makul yakın (tolerans + 0.3 m payı içinde)
                reached = (dist <= app.GoalTolField.Value) || ...
                          (abs(vel) < 0.02 && dist <= app.GoalTolField.Value + 0.3 && elapsed > 3);
                if reached
                    app.CurrentWaypointIdx = app.CurrentWaypointIdx + 1;
                    if app.CurrentWaypointIdx <= total_wp
                        app.sendNextWaypoint();
                        app.LastPosition = [x, y];
                        app.LastProgressTime = tic;
                    else
                        app.IsNavigating = false;
                        app.ProgressGauge.Value = 100;
                        app.CurrentWPLbl.Text = sprintf('WP %d/%d ✓', total_wp, total_wp);
                        app.MissionTimeLbl.Text = sprintf('✓ Mission Complete: %.1f s', elapsed);
                        app.MissionTimeLbl.FontColor = 'green';
                        app.saveMissionResult(elapsed);
                        
                        % Intercept if Parameter Sweep is active
                        if ~isempty(app.SweepCurrentIdx) && app.SweepCurrentIdx > 0 && app.SweepCurrentIdx <= length(app.SweepValues)
                            val = app.SweepValues(app.SweepCurrentIdx);
                            runRes = struct('param_val', val, 'elapsed', elapsed);
                            if isempty(app.SweepResults)
                                app.SweepResults = runRes;
                            else
                                app.SweepResults(end+1) = runRes;
                            end
                            
                            app.SweepCurrentIdx = app.SweepCurrentIdx + 1;
                            if app.SweepCurrentIdx <= length(app.SweepValues)
                                nextVal = app.SweepValues(app.SweepCurrentIdx);
                                app.applySweptParameter(nextVal);
                                
                                msg = sprintf('Run %d/%d completed.\nParameter value tested: %.2f (Time: %.1f s)\n\nPlease reset/recover the robot in Gazebo, then click OK to start Run %d (Parameter = %.2f).', ...
                                    app.SweepCurrentIdx-1, length(app.SweepValues), val, elapsed, app.SweepCurrentIdx, nextVal);
                                uialert(app.UIFigure, msg, 'Sweep Progress', 'Icon', 'info', ...
                                    'CloseFcn', @(~,~) app.onStartMission());
                            else
                                app.SweepCurrentIdx = 0;
                                app.showSweepReport();
                            end
                        else
                            app.showMissionReport(elapsed, total_wp);
                        end
                    end
                end
            else
                app.LastPosition = [];
                app.LastProgressTime = [];
            end
            
            % Live position plot
            if isempty(app.RobotPlotLive) || ~isvalid(app.RobotPlotLive)
                hold(app.LiveMapAxes, 'on');
                app.RobotPlotLive = plot(app.LiveMapAxes, x, y, 'bo', ...
                    'MarkerFaceColor', 'b', 'MarkerSize', 12);
                hold(app.LiveMapAxes, 'off');
            else
                app.RobotPlotLive.XData = x;
                app.RobotPlotLive.YData = y;
            end
            
            % Logging
            if app.IsLogging
                ts = posixtime(datetime('now'));
                app.LogData = [app.LogData; ts, vel, x, y];
            end
        end
        
        function sendNextWaypoint(app)
            if app.OfflineMode || isempty(app.WaypointTable.Data)
                return;
            end
            idx = app.CurrentWaypointIdx;
            tx = app.WaypointTable.Data(idx, 1);
            ty = app.WaypointTable.Data(idx, 2);
            tt = app.WaypointTable.Data(idx, 3);
            
            % Use 'map' frame purely, as TF mode relay now provides a stable global map frame
            frame_id = 'map';
            target_x = tx;
            target_y = ty;
            target_yaw_rad = deg2rad(tt);
            
            % Update target variables for distance check in onOdomCallback
            app.TargetX = target_x;
            app.TargetY = target_y;
            app.TargetFrame = frame_id;
            
            app.GoalMsg.header.frame_id = frame_id;
            if ~isempty(app.CurrentRosTime)
                app.GoalMsg.header.stamp = app.CurrentRosTime;
            end
            app.GoalMsg.pose.position.x = target_x;
            app.GoalMsg.pose.position.y = target_y;
            app.GoalMsg.pose.position.z = 0;
            app.GoalMsg.pose.orientation.w = cos(target_yaw_rad/2);
            app.GoalMsg.pose.orientation.x = 0;
            app.GoalMsg.pose.orientation.y = 0;
            app.GoalMsg.pose.orientation.z = sin(target_yaw_rad/2);
            send(app.GoalPub, app.GoalMsg);
            app.LastGoalSentTime = tic;
        end
        
        function applyParamsToROS(app)
            if app.OfflineMode
                return;
            end
            
            % Get fields from GUI
            max_vel = app.MaxVelField.Value;
            max_acc = app.MaxAccField.Value;
            safe_dist = app.SafeDistField.Value;
            goal_tol = app.GoalTolField.Value;
            
            % Run docker commands dynamically in ROS 2 asynchronously (non-blocking)
            try
                cmd = sprintf(['docker exec ros2-dev bash -c "' ...
                    'source /opt/ros/humble/setup.bash && ' ...
                    'ros2 param set /controller_server FollowPath.max_vel_x %.2f && ' ...
                    'ros2 param set /controller_server FollowPath.max_speed_xy %.2f && ' ...
                    'ros2 param set /controller_server FollowPath.acc_lim_x %.2f && ' ...
                    'ros2 param set /controller_server general_goal_checker.xy_goal_tolerance %.2f && ' ...
                    'ros2 param set /controller_server FollowPath.xy_goal_tolerance %.2f && ' ...
                    'ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius %.2f && ' ...
                    'ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius %.2f' ...
                    '" &'], max_vel, max_vel, max_acc, goal_tol, goal_tol, safe_dist, safe_dist);
                system(cmd);
            catch
            end
        end
        
        function showMissionReport(app, elapsed, total_wp)
            % ╔════════════════════════════════════════════════╗
            % ║  Otomatik Görev Sonu Raporu                    ║
            % ║  Planlanan vs Gerçek rota karşılaştırması      ║
            % ╚════════════════════════════════════════════════╝
            
            if isempty(app.LogData) || isempty(app.WaypointTable.Data)
                uialert(app.UIFigure, ...
                    sprintf('🎉 Mission Complete!\n\nTime: %.1f s', elapsed), ...
                    'Success', 'Icon', 'success');
                return;
            end
            
            % Verileri çek
            wp = app.WaypointTable.Data;
            log = app.LogData;
            
            % Performans metrikleri hesapla
            mission_time = elapsed;
            
            % Toplam kat edilen yol
            traveled = 0;
            for i = 2:size(log,1)
                traveled = traveled + sqrt((log(i,3)-log(i-1,3))^2 + (log(i,4)-log(i-1,4))^2);
            end
            
            % Planlanan yol
            planned = 0;
            for i = 2:size(wp,1)
                planned = planned + sqrt((wp(i,1)-wp(i-1,1))^2 + (wp(i,2)-wp(i-1,2))^2);
            end
            
            % Path error (her gerçek noktanın en yakın planlanan segmente uzaklığı)
            errors = zeros(size(log,1),1);
            for i = 1:size(log,1)
                rx = log(i,3); ry = log(i,4);
                min_d = inf;
                for j = 1:size(wp,1)-1
                    % Segment uzaklığı
                    x1 = wp(j,1); y1 = wp(j,2);
                    x2 = wp(j+1,1); y2 = wp(j+1,2);
                    dx = x2-x1; dy = y2-y1;
                    if dx == 0 && dy == 0
                        d = sqrt((rx-x1)^2 + (ry-y1)^2);
                    else
                        t = max(0, min(1, ((rx-x1)*dx + (ry-y1)*dy) / (dx^2+dy^2)));
                        px = x1 + t*dx; py = y1 + t*dy;
                        d = sqrt((rx-px)^2 + (ry-py)^2);
                    end
                    min_d = min(min_d, d);
                end
                errors(i) = min_d;
            end
            
            mean_err = mean(errors);
            max_err = max(errors);
            rmse = sqrt(mean(errors.^2));
            
            % Hız metrikleri
            peak_vel = max(abs(log(:,2)));
            avg_vel = mean(abs(log(:,2)));
            
            % Smoothness (Jerk)
            if size(log,1) > 3
                dt = diff(log(:,1));
                v = log(:,2);
                a = diff(v) ./ dt;
                jerk = diff(a) ./ dt(2:end);
                smoothness = std(jerk);
            else
                smoothness = 0;
            end
            
            % FIGURE OLUŞTUR
            fig = figure('Name', sprintf('Mission Report - %s', ...
                datestr(now, 'HH:MM:SS')), ...
                'NumberTitle', 'off', ...
                'Position', [100 100 1400 800], ...
                'Color', 'white');
            
            % ── 1. Path Comparison (sol üst, büyük) ──
            subplot(2, 3, [1 4]);
            plot(log(:,3), log(:,4), 'b-', 'LineWidth', 2);
            hold on;
            plot(wp(:,1), wp(:,2), 'r--o', 'LineWidth', 2, ...
                'MarkerSize', 12, 'MarkerFaceColor', 'r');
            
            % Waypoint numaraları
            for i = 1:size(wp,1)
                text(wp(i,1)+0.1, wp(i,2)+0.15, sprintf('WP%d', i), ...
                    'FontWeight', 'bold', 'FontSize', 11);
            end
            
            % Başlangıç ve bitiş
            plot(log(1,3), log(1,4), 'gs', 'MarkerSize', 16, ...
                'MarkerFaceColor', 'g');
            plot(log(end,3), log(end,4), 'rp', 'MarkerSize', 18, ...
                'MarkerFaceColor', 'r');
            
            grid on; axis equal;
            xlabel('X (m)', 'FontSize', 12);
            ylabel('Y (m)', 'FontSize', 12);
            title('Planlanan vs Gerçek Rota', 'FontSize', 14, 'FontWeight', 'bold');
            legend({'Gerçek Yol', 'Planlanan Rota', 'Başlangıç', 'Bitiş'}, ...
                'Location', 'best');
            
            % ── 2. Hız Profili (sağ üst) ──
            subplot(2, 3, 2);
            t = log(:,1) - log(1,1);
            plot(t, abs(log(:,2)), 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Zaman (s)');
            ylabel('Hız (m/s)');
            title('Hız Profili', 'FontWeight', 'bold');
            
            % ── 3. Path Error (orta sağ) ──
            subplot(2, 3, 3);
            plot(t, errors, 'm-', 'LineWidth', 1.5);
            grid on;
            xlabel('Zaman (s)');
            ylabel('Path Error (m)');
            title('Rota Hatası (Zaman içinde)', 'FontWeight', 'bold');
            yline(mean_err, 'k--', sprintf('Ort: %.3f m', mean_err));
            
            % ── 4. Metrik Tablosu (alt sağ) ──
            subplot(2, 3, [5 6]);
            axis off;
            
            metrics_text = {
                sprintf('\\bf{╔══════════════ MİSYON METRİKLERİ ══════════════╗}');
                '';
                sprintf('\\bf{Görev Süresi:}       %.2f s', mission_time);
                sprintf('\\bf{Toplam Mesafe:}      %.2f m  (Planlanan: %.2f m)', traveled, planned);
                sprintf('\\bf{Verimlilik:}         %.1f%%', (planned/traveled)*100);
                '';
                sprintf('\\bf{Hız Metrikleri:}');
                sprintf('   Maksimum Hız:     %.2f m/s', peak_vel);
                sprintf('   Ortalama Hız:     %.2f m/s', avg_vel);
                '';
                sprintf('\\bf{Path Error (Rota Sapması):}');
                sprintf('   Ortalama Hata:    %.3f m', mean_err);
                sprintf('   Maksimum Hata:    %.3f m', max_err);
                sprintf('   RMSE:             %.3f m', rmse);
                '';
                sprintf('\\bf{Yol Düzgünlüğü:}     %.2f m/s³ (jerk std)', smoothness);
                sprintf('\\bf{Waypoint Sayısı:}    %d', total_wp);
                sprintf('\\bf{Veri Noktası:}       %d', size(log,1));
                '';
                sprintf('\\bf{╚════════════════════════════════════════════════╝}');
            };
            
            text(0.05, 0.95, metrics_text, 'Units', 'normalized', ...
                'VerticalAlignment', 'top', 'FontSize', 11, ...
                'FontName', 'FixedWidth');
            
            % Genel başlık
            sgtitle(sprintf('🎯 Mission Complete — Time: %.1f s | Waypoints: %d | Mean Error: %.3f m', ...
                mission_time, total_wp, mean_err), 'FontSize', 14, 'FontWeight', 'bold');
            
            % Otomatik kaydet
            ts = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
            report_path = fullfile(app.ResultsDir, sprintf('report_%s.png', ts));
            saveas(fig, report_path);
            fprintf('📊 Rapor kaydedildi: %s\n', report_path);
            
            % Başarı popup'ı
            uialert(app.UIFigure, ...
                sprintf(['🎉 Mission Complete!\n\n' ...
                'Time: %.1f s\n' ...
                'Path Error (mean): %.3f m\n' ...
                'Path Error (RMSE): %.3f m\n\n' ...
                'Report auto-saved!'], ...
                mission_time, mean_err, rmse), ...
                'Success', 'Icon', 'success');
        end
        
        function saveMissionResult(app, elapsed)
            ts = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
            filename = fullfile(app.ResultsDir, sprintf('test_%s.mat', ts));
            
            data = struct();
            data.timestamp = ts;
            data.waypoints = app.WaypointTable.Data;
            data.params = struct(...
                'max_vel', app.MaxVelField.Value, ...
                'max_acc', app.MaxAccField.Value, ...
                'braking', app.BrakingField.Value, ...
                'safe_dist', app.SafeDistField.Value, ...
                'goal_tol', app.GoalTolField.Value);
            data.log = app.LogData;
            data.mission_time = elapsed;
            data.completed = true;
            
            save(filename, 'data');
            app.refreshHistory();
            
            % Automatically load this result file on the Results Analyzer tab and switch to it
            app.loadResultFile(filename, sprintf('test_%s', ts));
            app.TabGroup.SelectedTab = app.AnalyzerTab;
        end
        
        function refreshHistory(app)
            files = dir(fullfile(app.ResultsDir, 'test_*.mat'));
            if isempty(files)
                app.HistoryTable.Data = {};
                return;
            end
            
            history = cell(length(files), 5);
            for i = 1:length(files)
                try
                    d = load(fullfile(app.ResultsDir, files(i).name));
                    [~, name, ~] = fileparts(files(i).name);
                    history{i, 1} = name;
                    history{i, 2} = size(d.data.waypoints, 1);
                    history{i, 3} = sprintf('%.2f', d.data.params.max_vel);
                    if isfield(d.data, 'mission_time')
                        history{i, 4} = sprintf('%.1f s', d.data.mission_time);
                    else
                        history{i, 4} = 'N/A';
                    end
                    history{i, 5} = files(i).date;
                catch
                end
            end
            app.HistoryTable.Data = history;
        end
        
        function updateClock(app, ~, ~)
            app.ClockLabel.Text = datestr(now, 'yyyy-mm-dd HH:MM:SS');
        end
        
    end
    
    %% ─── CALLBACKS ───
    methods (Access = private)
        
        function startupFcn(app)
            % Add path to repository matlab_interface
            repo_matlab_path = fullfile(getenv('HOME'), 'necati_repo', 'matlab_interface');
            if exist(repo_matlab_path, 'dir')
                addpath(repo_matlab_path);
            end
            app.SonCizimZamani = [];
            app.buildObstacleDB();
            app.setupDirs();
            app.connectROS();
            app.applyPreset('Normal');
            app.updateMap();
            app.refreshHistory();
            app.loadMissionLibrary();
            
            % Clock timer
            app.ClockTimer = timer('Period', 1, 'ExecutionMode', 'fixedRate', ...
                'TimerFcn', @(~,~) app.updateClock());
            start(app.ClockTimer);
            
            % Nav Mode timer (GPS/SLAM göstergesi - 0.5s'de bir güncelle)
            if ~app.OfflineMode && ~isempty(app.NavModeSub)
                app.NavModeTimer = timer('Period', 0.5, ...
                    'ExecutionMode', 'fixedRate', ...
                    'TimerFcn', @(~,~) app.updateNavMode());
                start(app.NavModeTimer);
            end
        end
        
        function updateNavMode(app)
            % GPS/SLAM mod göstergesini güncelle
            if isempty(app.NavModeSub)
                return;
            end
            try
                msg = app.NavModeSub.LatestMessage;
                if isempty(msg)
                    return;
                end
                mode = upper(strtrim(msg.data));
                
                % Mod değişti mi? Say
                if ~isempty(app.LastNavMode) && ~strcmp(app.LastNavMode, mode)
                    app.NavModeSwitchCount = app.NavModeSwitchCount + 1;
                    app.NavModeCountLbl.Text = sprintf('Mod geçiş sayısı: %d', ...
                        app.NavModeSwitchCount);
                    
                    % GPS -> SLAM geçişi — SADECE İLK KEZ kaydet
                    if strcmp(app.LastNavMode, 'GPS') && strcmp(mode, 'SLAM')
                        if isempty(app.SlamOdomStartX)
                            % İlk geçiş: odom başlangıç noktasını kaydet
                            app.SlamOdomStartX   = app.OdomX;
                            app.SlamOdomStartY   = app.OdomY;
                            app.SlamOdomStartYaw = app.OdomYaw;
                            % Aktif waypoint'i odom frame'inde yeniden gönder
                            if app.IsNavigating
                                app.sendNextWaypoint();
                            end
                        end
                        % Sonraki GPS->SLAM geçişlerinde SlamOdomStart'ı BOZMA
                        % (GPS spoofer artık latch kullandığı için bu nadiren olur)
                    elseif strcmp(app.LastNavMode, 'SLAM') && strcmp(mode, 'GPS')
                        % SLAM->GPS: tünelden çıkınca odom referansını sıfırla
                        app.SlamOdomStartX   = [];
                        app.SlamOdomStartY   = [];
                        app.SlamOdomStartYaw = [];
                        % GPS moduna döndüğünde map frame'inde devam et
                        if app.IsNavigating
                            app.sendNextWaypoint();
                        end
                    end
                end
                app.LastNavMode = mode;
                
                % Renk ve metin
                if contains(mode, 'GPS')
                    app.NavModeLamp.Color = [0.2 0.8 0.2];  % Yeşil
                    app.NavModeValueLbl.Text = '🛰️ GPS';
                    app.NavModeValueLbl.FontColor = [0.1 0.6 0.1];
                elseif contains(mode, 'SLAM')
                    app.NavModeLamp.Color = [0.2 0.4 0.9];  % Mavi
                    app.NavModeValueLbl.Text = '🗺️ SLAM';
                    app.NavModeValueLbl.FontColor = [0.1 0.3 0.8];
                else
                    app.NavModeLamp.Color = [0.6 0.6 0.6];
                    app.NavModeValueLbl.Text = mode;
                    app.NavModeValueLbl.FontColor = [0.3 0.3 0.3];
                end
            catch
            end
        end
        
        function onGPSModeChanged(app, event)
            % XY <-> GPS modu değişimi
            if strcmp(event.Value, 'GPS')
                app.GPSMode = true;
                app.XLabel.Text = 'Lat:';
                app.YLabel.Text = 'Lon:';
                app.XField.Value = app.GPS_LAT0;
                app.YField.Value = app.GPS_LON0;
            else
                app.GPSMode = false;
                app.XLabel.Text = 'X (m):';
                app.YLabel.Text = 'Y (m):';
                app.XField.Value = 0;
                app.YField.Value = 0;
            end
        end
        
        function [x, y] = gpsToXY(app, lat, lon)
            % GPS lat/lon → map XY (ROSçunun /fromLL ile kalibre, <2cm hata)
            R = 6378137.0;  % WGS84 ekvator yarıçapı
            yaw = deg2rad(app.GPS_YAW_DEG);
            dlat = deg2rad(lat - app.GPS_LAT0);
            dlon = deg2rad(lon - app.GPS_LON0);
            east  = dlon * R * cos(deg2rad(app.GPS_LAT0));
            north = dlat * R;
            x = east*cos(yaw) - north*sin(yaw);
            y = east*sin(yaw) + north*cos(yaw);
        end

        function loadMissionLibrary(app)
            % Clean existing library
            app.MissionLibrary = [];
            
            missionsDir = fullfile(getenv('HOME'), 'zeynep_missions_deneme');
            if ~exist(missionsDir, 'dir')
                mkdir(missionsDir);
            end
            
            % If the folder is empty of matching files, try running the generator
            d = dir(fullfile(missionsDir, '*.mat'));
            if isempty(d)
                try
                    generate_mission_library();
                    d = dir(fullfile(missionsDir, '*.mat'));
                catch
                end
            end
            
            prefix = [app.CurrentWorld, '_'];
            dropdownItems = {};
            dropdownData = {};
            
            for i = 1:length(d)
                filename = d(i).name;
                [~, name, ~] = fileparts(filename);
                if startsWith(name, prefix)
                    try
                        content = load(fullfile(missionsDir, filename));
                        if isfield(content, 'mission')
                            m = content.mission;
                            if isempty(app.MissionLibrary)
                                app.MissionLibrary = m;
                            else
                                app.MissionLibrary(end+1) = m;
                            end
                            dropdownItems{end+1} = sprintf('%s (%s)', m.scenario, m.name);
                            dropdownData{end+1} = m.scenario;
                        end
                    catch
                    end
                end
            end
            
            if isempty(dropdownItems)
                app.ScenarioDropDown.Items = {'No scenarios found'};
                app.ScenarioDropDown.ItemsData = {''};
            else
                app.ScenarioDropDown.Items = dropdownItems;
                app.ScenarioDropDown.ItemsData = dropdownData;
            end
            app.updateScenarioInfo();
        end

        function updateScenarioInfo(app)
            val = app.ScenarioDropDown.Value;
            if isempty(val) || strcmp(val, '') || isempty(app.MissionLibrary)
                app.QuickMissionInfoLbl.Text = 'No scenario active.';
                return;
            end
            
            found = false;
            for i = 1:length(app.MissionLibrary)
                if strcmp(app.MissionLibrary(i).scenario, val)
                    m = app.MissionLibrary(i);
                    wps_count = size(m.waypoints, 1);
                    desc = m.description;
                    if length(desc) > 80
                        desc = [desc(1:77), '...'];
                    end
                    app.QuickMissionInfoLbl.Text = sprintf('📝 %d WPs: %s', wps_count, desc);
                    found = true;
                    break;
                end
            end
            if ~found
                app.QuickMissionInfoLbl.Text = 'No scenario details found.';
            end
        end

        function onScenarioChanged(app, ~)
            app.updateScenarioInfo();
        end

        function onQuickLoad(app, ~)
            val = app.ScenarioDropDown.Value;
            if isempty(val) || strcmp(val, '') || isempty(app.MissionLibrary)
                uialert(app.UIFigure, 'No scenario selected or found!', 'Error');
                return;
            end
            
            found = false;
            for i = 1:length(app.MissionLibrary)
                if strcmp(app.MissionLibrary(i).scenario, val)
                    m = app.MissionLibrary(i);
                    app.WaypointTable.Data = m.waypoints;
                    app.updateMap();
                    found = true;
                    uialert(app.UIFigure, sprintf('⚡ Waypoints loaded for scenario "%s" (%d waypoints)!', m.scenario, size(m.waypoints,1)), 'Success', 'Icon', 'success');
                    break;
                end
            end
            if ~found
                uialert(app.UIFigure, 'Failed to load scenario waypoints.', 'Error');
            end
        end

        function onAutoParams(app, ~)
            val = app.ScenarioDropDown.Value;
            if isempty(val) || strcmp(val, '') || isempty(app.MissionLibrary)
                uialert(app.UIFigure, 'No scenario selected or found!', 'Error');
                return;
            end
            
            found = false;
            for i = 1:length(app.MissionLibrary)
                if strcmp(app.MissionLibrary(i).scenario, val)
                    m = app.MissionLibrary(i);
                    app.MaxVelField.Value = m.params.max_vel;
                    app.MaxVelSlider.Value = m.params.max_vel;
                    
                    app.MaxAccField.Value = m.params.max_acc;
                    app.MaxAccSlider.Value = m.params.max_acc;
                    
                    app.BrakingField.Value = m.params.braking;
                    app.BrakingSlider.Value = m.params.braking;
                    
                    app.SafeDistField.Value = m.params.safe_dist;
                    app.SafeDistSlider.Value = m.params.safe_dist;
                    
                    app.GoalTolField.Value = m.params.goal_tol;
                    app.GoalTolSlider.Value = m.params.goal_tol;
                    
                    app.CustomBtn.Value = true;
                    app.onGenerateYAML();
                    
                    found = true;
                    uialert(app.UIFigure, sprintf('⚙️ Scenario parameters auto-tuned successfully!\nMax Vel: %.2f m/s\nMax Acc: %.2f m/s²\nBraking Time: %.2f s\nSafe Dist: %.2f m\nGoal Tol: %.2f m', ...
                        m.params.max_vel, m.params.max_acc, m.params.braking, m.params.safe_dist, m.params.goal_tol), 'Auto-Tune', 'Icon', 'success');
                    break;
                end
            end
            if ~found
                uialert(app.UIFigure, 'Failed to load scenario parameters.', 'Error');
            end
        end

        function onSweepPlan(app, ~)
            paramName = app.SweepParamDropDown.Value;
            mn = app.SweepMinField.Value;
            mx = app.SweepMaxField.Value;
            step = app.SweepStepField.Value;
            
            if mn > mx
                uialert(app.UIFigure, 'Min value cannot be greater than Max value!', 'Sweep Error');
                return;
            end
            if step <= 0
                uialert(app.UIFigure, 'Step must be positive!', 'Sweep Error');
                return;
            end
            
            values = mn:step:mx;
            if isempty(values)
                uialert(app.UIFigure, 'No parameter sweep steps generated. Check range and step.', 'Sweep Error');
                return;
            end
            
            switch paramName
                case 'Max Velocity'
                    app.SweepParamName = 'max_vel';
                case 'Max Acceleration'
                    app.SweepParamName = 'max_acc';
                case 'Braking Time'
                    app.SweepParamName = 'braking';
                case 'Safe Distance'
                    app.SweepParamName = 'safe_dist';
                case 'Goal Tolerance'
                    app.SweepParamName = 'goal_tol';
            end
            
            app.SweepValues = values;
            app.SweepCurrentIdx = 1;
            app.SweepResults = [];
            
            app.SweepInfoLbl.Text = sprintf('Sweep Ready: %d runs for %s [%.2f:%.2f:%.2f]', ...
                length(values), paramName, mn, step, mx);
            app.SweepInfoLbl.FontColor = [0 0.5 0];
            
            app.applySweptParameter(values(1));
            
            uialert(app.UIFigure, sprintf('⚡ Sweep Plan Ready!\n%d runs generated for %s.\n\nFirst test value is %.2f.\nClick "START MISSION" on the Monitor tab to begin the sweep sequence.', ...
                length(values), paramName, values(1)), 'Sweep Plan', 'Icon', 'success');
        end

        function applySweptParameter(app, val)
            switch app.SweepParamName
                case 'max_vel'
                    app.MaxVelField.Value = val;
                    app.MaxVelSlider.Value = val;
                case 'max_acc'
                    app.MaxAccField.Value = val;
                    app.MaxAccSlider.Value = val;
                case 'braking'
                    app.BrakingField.Value = val;
                    app.BrakingSlider.Value = val;
                case 'safe_dist'
                    app.SafeDistField.Value = val;
                    app.SafeDistSlider.Value = val;
                case 'goal_tol'
                    app.GoalTolField.Value = val;
                    app.GoalTolSlider.Value = val;
            end
            app.CustomBtn.Value = true;
            app.onGenerateYAML();
        end

        function showSweepReport(app)
            msg = sprintf('📊 PARAMETER SWEEP SUMMARY\nParameter Swept: %s\n\n', app.SweepParamName);
            for i = 1:length(app.SweepResults)
                res = app.SweepResults(i);
                msg = [msg, sprintf('Run %d: Val = %.2f, Completion Time = %.1f s\n', ...
                    i, res.param_val, res.elapsed)];
            end
            
            try
                vals = [app.SweepResults.param_val];
                times = [app.SweepResults.elapsed];
                fig = figure('Name', 'Sweep Analysis', 'NumberTitle', 'off');
                plot(vals, times, '-o', 'LineWidth', 2, 'MarkerSize', 8);
                xlabel(app.SweepParamName);
                ylabel('Completion Time (s)');
                title(['Sweep Analysis: ' app.SweepParamName ' vs Completion Time']);
                grid on;
            catch
            end
            
            uialert(app.UIFigure, msg, 'Sweep Completed!', 'Icon', 'success');
        end
        
        function onAddBtn(app, ~)
            if app.GPSMode
                % GPS modu: Lat/Lon → XY dönüşümü
                lat = app.XField.Value;
                lon = app.YField.Value;
                [x, y] = app.gpsToXY(lat, lon);
                new_wp = [x, y, app.ThetaField.Value];
                % GPS girişini sıfırla (datum'a dön)
                app.XField.Value = app.GPS_LAT0;
                app.YField.Value = app.GPS_LON0;
                app.ThetaField.Value = 0;
            else
                new_wp = [app.XField.Value, app.YField.Value, app.ThetaField.Value];
                app.XField.Value = 0;
                app.YField.Value = 0;
                app.ThetaField.Value = 0;
            end

            % ═══ GÜVENLİK KONTROLÜ ═══
            if ~app.confirmIfUnsafe(new_wp(1), new_wp(2)), return; end

            if isempty(app.WaypointTable.Data)
                app.WaypointTable.Data = new_wp;
            else
                app.WaypointTable.Data = [app.WaypointTable.Data; new_wp];
            end
            app.updateMap();
            app.updateYAML();
        end

        function ok = confirmIfUnsafe(app, x, y)
            % Engele yakınsa kullanıcıyı uyarır; ok=false ise ekleme iptal.
            ok = true;
            [safe, mindist, who] = app.checkWaypointSafe(x, y);
            if ~safe
                if mindist <= 0.01
                    msg = sprintf(['⛔ (%.1f, %.1f) bir engelin İÇİNDE (%s).' newline newline ...
                        'Bu nokta neredeyse kesin çarpışmaya yol açar.' newline ...
                        'Yine de eklemek istiyor musun?'], x, y, who);
                else
                    msg = sprintf(['⚠️ (%.1f, %.1f) bir engele çok yakın (%.2f m, %s).' newline newline ...
                        'Güvenli sınır %.1f m. Robot takılabilir.' newline newline ...
                        'Yine de eklemek istiyor musun?'], x, y, mindist, who, app.SafeClearance);
                end
                sel = uiconfirm(app.UIFigure, msg, 'Güvenlik Uyarısı', ...
                    'Options', {'Yine de Ekle', 'İptal'}, ...
                    'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
                if strcmp(sel, 'İptal')
                    ok = false;
                    app.SafetyLbl.Text = '⚠️ Güvensiz waypoint iptal edildi';
                    app.SafetyLbl.FontColor = [0.9 0.5 0];
                end
            else
                app.SafetyLbl.Text = sprintf('✓ Güvenli (en yakın engel %.1f m)', mindist);
                app.SafetyLbl.FontColor = [0.2 0.6 0.2];
            end
        end
        
        function onDeleteWp(app, ~)
            sel = app.WaypointTable.Selection;
            if isempty(sel) || isempty(app.WaypointTable.Data)
                uialert(app.UIFigure, 'Select a waypoint first.', 'No Selection');
                return;
            end
            row = sel(1);
            data = app.WaypointTable.Data;
            data(row, :) = [];
            app.WaypointTable.Data = data;
            app.updateMap();
            app.updateYAML();
        end
        
        function onMoveUp(app, ~)
            sel = app.WaypointTable.Selection;
            if isempty(sel) || sel(1) == 1
                return;
            end
            row = sel(1);
            data = app.WaypointTable.Data;
            tmp = data(row, :);
            data(row, :) = data(row-1, :);
            data(row-1, :) = tmp;
            app.WaypointTable.Data = data;
            app.updateMap();
            app.updateYAML();
        end
        
        function onMoveDown(app, ~)
            sel = app.WaypointTable.Selection;
            if isempty(sel) || sel(1) >= size(app.WaypointTable.Data, 1)
                return;
            end
            row = sel(1);
            data = app.WaypointTable.Data;
            tmp = data(row, :);
            data(row, :) = data(row+1, :);
            data(row+1, :) = tmp;
            app.WaypointTable.Data = data;
            app.updateMap();
            app.updateYAML();
        end
        
        function onClearAll(app, ~)
            choice = uiconfirm(app.UIFigure, ...
                'Delete all waypoints?', 'Confirm Clear', ...
                'Options', {'Yes, Delete All', 'Cancel'}, ...
                'Icon', 'warning');
            if strcmp(choice, 'Yes, Delete All')
                app.WaypointTable.Data = [];
                app.updateMap();
                app.updateYAML();
            end
        end
        
        function onImportCSV(app, ~)
            [file, path] = uigetfile('*.csv', 'Select Waypoint File');
            if isequal(file, 0)
                return;
            end
            try
                data = readmatrix(fullfile(path, file));
                if size(data, 2) < 2
                    error('CSV must have at least 2 columns (X, Y)');
                end
                if size(data, 2) >= 3
                    new_data = data(:, 1:3);
                else
                    new_data = [data(:, 1:2), zeros(size(data,1), 1)];
                end
                if isempty(app.WaypointTable.Data)
                    app.WaypointTable.Data = new_data;
                else
                    app.WaypointTable.Data = [app.WaypointTable.Data; new_data];
                end
                app.updateMap();
                app.updateYAML();
            catch ME
                uialert(app.UIFigure, ME.message, 'Import Error', 'Icon', 'error');
            end
        end
        
        function onExportCSV(app, ~)
            if isempty(app.WaypointTable.Data)
                uialert(app.UIFigure, 'No waypoints to export!', 'Empty');
                return;
            end
            [file, path] = uiputfile('*.csv', 'Save Waypoints');
            if isequal(file, 0)
                return;
            end
            writematrix(app.WaypointTable.Data, fullfile(path, file));
            uialert(app.UIFigure, 'Waypoints exported!', 'Success', 'Icon', 'success');
        end
        
        function onSaveMission(app, ~)
            if isempty(app.WaypointTable.Data)
                uialert(app.UIFigure, 'No mission to save!', 'Empty');
                return;
            end
            prompt = inputdlg('Mission Name:', 'Save Mission', [1 40], {'mission_1'});
            if isempty(prompt)
                return;
            end
            mission = struct();
            mission.name = prompt{1};
            mission.waypoints = app.WaypointTable.Data;
            mission.params = struct(...
                'max_vel', app.MaxVelField.Value, ...
                'max_acc', app.MaxAccField.Value, ...
                'braking', app.BrakingField.Value, ...
                'safe_dist', app.SafeDistField.Value, ...
                'goal_tol', app.GoalTolField.Value);
            mission.created = datestr(now);
            filename = fullfile(app.MissionsDir, sprintf('%s.mat', prompt{1}));
            save(filename, 'mission');
            uialert(app.UIFigure, sprintf('Mission saved: %s', prompt{1}), 'Success', 'Icon', 'success');
        end
        
        function onLoadMission(app, ~)
            [file, path] = uigetfile(fullfile(app.MissionsDir, '*.mat'), 'Load Mission');
            if isequal(file, 0)
                return;
            end
            try
                d = load(fullfile(path, file));
                app.WaypointTable.Data = d.mission.waypoints;
                if isfield(d.mission, 'params')
                    p = d.mission.params;
                    app.MaxVelField.Value = p.max_vel;
                    app.MaxAccField.Value = p.max_acc;
                    app.BrakingField.Value = p.braking;
                    app.SafeDistField.Value = p.safe_dist;
                    app.GoalTolField.Value = p.goal_tol;
                    app.syncSliders();
                end
                app.updateMap();
                app.updateYAML();
                uialert(app.UIFigure, 'Mission loaded!', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, ME.message, 'Error', 'Icon', 'error');
            end
        end
        
        function onMapClick(app, event)
            x = double(event.IntersectionPoint(1));
            y = double(event.IntersectionPoint(2));
            app.XField.Value = x;
            app.YField.Value = y;
            % ═══ GÜVENLİK KONTROLÜ ═══
            if ~app.confirmIfUnsafe(x, y), return; end
            new_wp = [x, y, 0];
            if isempty(app.WaypointTable.Data)
                app.WaypointTable.Data = new_wp;
            else
                app.WaypointTable.Data = [app.WaypointTable.Data; new_wp];
            end
            app.updateMap();
            app.updateYAML();
        end
        
        function onTableEdit(app, ~)
            app.updateMap();
            app.updateYAML();
        end
        
        function onPresetChanged(app, event)
            preset = event.NewValue.Text;
            if ~strcmp(preset, 'Custom')
                app.applyPreset(preset);
            end
        end
        
        function onParamChanged(app, ~)
            app.CustomBtn.Value = true;
            app.syncSliders();
            app.updateYAML();
        end
        
        function onSliderChanged(app, ~)
            app.MaxVelField.Value = app.MaxVelSlider.Value;
            app.MaxAccField.Value = app.MaxAccSlider.Value;
            app.BrakingField.Value = app.BrakingSlider.Value;
            app.SafeDistField.Value = app.SafeDistSlider.Value;
            app.GoalTolField.Value = app.GoalTolSlider.Value;
            app.CustomBtn.Value = true;
            app.updateYAML();
        end
        
        function onGenerateYAML(app, ~)
            app.updateYAML();
            uialert(app.UIFigure, 'YAML updated in preview!', 'Generated', 'Icon', 'success');
        end
        
        function onSaveYAML(app, ~)
            [file, path] = uiputfile('*.yaml', 'Save Nav2 YAML');
            if isequal(file, 0)
                return;
            end
            fid = fopen(fullfile(path, file), 'w');
            yamlContent = app.YAMLArea.Value;
            if iscell(yamlContent)
                yamlContent = strjoin(yamlContent, newline);
            end
            fprintf(fid, '%s', yamlContent);
            fclose(fid);
            uialert(app.UIFigure, 'YAML saved!', 'Success', 'Icon', 'success');
        end
        
        function onStartMission(app, ~)
            if isempty(app.WaypointTable.Data)
                uialert(app.UIFigure, 'No waypoints defined!', 'Error', 'Icon', 'error');
                return;
            end
            if app.OfflineMode
                uialert(app.UIFigure, 'ROS2 not connected!', 'Offline', 'Icon', 'error');
                return;
            end
            if app.GoalTolField.Value < 0.1
                uialert(app.UIFigure, 'Goal Tolerance must be >= 0.1m!', 'Parameter Error');
                return;
            end
            
            % Dynamically apply parameters to ROS 2 nodes
            app.applyParamsToROS();
            
            app.CurrentWaypointIdx = 1;
            app.IsNavigating = true;
            app.EStopActive = false;
            
            % Reset transition and target properties
            app.SlamOdomStartX = [];
            app.SlamOdomStartY = [];
            app.SlamOdomStartYaw = [];
            app.TargetX = 0;
            app.TargetY = 0;
            app.TargetFrame = 'map';
            
            % Initialize tracking states
            app.LastPosition = [];
            app.LastProgressTime = [];
            app.LastGoalSentTime = [];
            
            % Eski emergency stop timer'ını tamamen temizle
            if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                stop(app.EStopTimer);
                delete(app.EStopTimer);
                app.EStopTimer = [];
            end
            app.MissionStartTime = tic;
            app.MissionTimeLbl.Text = 'Mission Time: Running...';
            app.MissionTimeLbl.FontColor = 'blue';
            app.LogData = [];
            app.IsLogging = true;
            app.LoggingLamp.Color = 'red';
            app.LoggingSwitch.Value = 'On';
            app.ProgressGauge.Value = 0;
            app.sendNextWaypoint();
        end
        
        function onEStop(app, ~)
            % ═══ GERÇEK ACİL DURDURMA (v2 - goal iptal + agresif sıfır hız) ═══
            app.IsNavigating = false;
            app.EStopActive = true;
            app.MissionTimeLbl.Text = '🛑 EMERGENCY STOP!';
            app.MissionTimeLbl.FontColor = 'red';
            
            if ~app.OfflineMode
                % STRATEJİ: navigate_to_pose bir ACTION. Robotu durdurmak için
                % (a) mevcut konumu yeni hedef veririz → Nav2 "vardım" deyip durur,
                % (b) HEM /cmd_vel HEM /cmd_vel_nav'e yoğun sıfır hız basarız
                %     (zincir: Nav2→cmd_vel_nav→smoother→cmd_vel; ikisini de susturmalı).
                
                % 1. Robotun kendi yerel frame'ini (base_footprint) hedef ver
                try
                    app.GoalMsg.header.frame_id = 'base_footprint';
                    if ~isempty(app.CurrentRosTime)
                        app.GoalMsg.header.stamp = app.CurrentRosTime;
                    end
                    app.GoalMsg.pose.position.x = 0;
                    app.GoalMsg.pose.position.y = 0;
                    app.GoalMsg.pose.position.z = 0;
                    app.GoalMsg.pose.orientation.w = 1;
                    app.GoalMsg.pose.orientation.x = 0;
                    app.GoalMsg.pose.orientation.y = 0;
                    app.GoalMsg.pose.orientation.z = 0;
                    send(app.GoalPub, app.GoalMsg);
                catch
                end
                
                % 2. Agresif sıfır hız — HEM /cmd_vel HEM /cmd_vel_nav'e bas.
                app.VelMsg.linear.x = 0; app.VelMsg.linear.y = 0; app.VelMsg.linear.z = 0;
                app.VelMsg.angular.x = 0; app.VelMsg.angular.y = 0; app.VelMsg.angular.z = 0;
                if ~isempty(app.VelNavPub)
                    app.VelNavMsg.linear.x = 0; app.VelNavMsg.linear.y = 0;
                    app.VelNavMsg.angular.z = 0;
                end
                for k = 1:20
                    send(app.VelPub, app.VelMsg);
                    if ~isempty(app.VelNavPub), send(app.VelNavPub, app.VelNavMsg); end
                end
                
                % 3. Sürekli sıfır-hız timer'ı (4 saniye, 50 Hz, iki topic'e birden).
                if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                    stop(app.EStopTimer); delete(app.EStopTimer);
                end
                app.EStopTimer = timer('Period', 0.02, ...
                    'ExecutionMode', 'fixedRate', ...
                    'TasksToExecute', 200, ...
                    'TimerFcn', @(~,~) app.sendZeroVel());
                start(app.EStopTimer);
            end
            
            uialert(app.UIFigure, ...
                ['🛑 EMERGENCY STOP AKTİF!' newline newline ...
                 'Nav2 hedefi iptal edildi, robot durduruldu.' newline ...
                 'Devam için RESET''e bas.'], ...
                'STOP', 'Icon', 'error');
        end
        
        function sendZeroVel(app)
            % Emergency stop timer'ı - sürekli sıfır hız (iki topic'e birden)
            if ~app.EStopActive
                if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                    stop(app.EStopTimer);
                end
                return;
            end
            if ~app.OfflineMode
                try
                    app.VelMsg.linear.x = 0;
                    app.VelMsg.linear.y = 0;
                    app.VelMsg.angular.z = 0;
                    send(app.VelPub, app.VelMsg);
                    if ~isempty(app.VelNavPub)
                        app.VelNavMsg.linear.x = 0;
                        app.VelNavMsg.linear.y = 0;
                        app.VelNavMsg.angular.z = 0;
                        send(app.VelNavPub, app.VelNavMsg);
                    end
                catch
                end
            end
        end
        
        function onReset(app, ~)
            % ═══ RESET — her şeyi temiz başlangıç durumuna getir ═══
            % Emergency stop timer'ını durdur
            app.EStopActive = false;
            if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                stop(app.EStopTimer);
                delete(app.EStopTimer);
                app.EStopTimer = [];
            end
            
            % Robotu durdur: mevcut konumu hedef ver + iki cmd_vel topic'ine sıfır.
            if ~app.OfflineMode
                try
                    odom = receive(app.OdomSub, 1);
                    app.GoalMsg.header.frame_id = 'odom';
                    app.GoalMsg.header.stamp = odom.header.stamp;
                    app.GoalMsg.pose.position.x = odom.pose.pose.position.x;
                    app.GoalMsg.pose.position.y = odom.pose.pose.position.y;
                    app.GoalMsg.pose.position.z = 0;
                    app.GoalMsg.pose.orientation = odom.pose.pose.orientation;
                    send(app.GoalPub, app.GoalMsg);
                catch
                end
                try
                    app.VelMsg.linear.x = 0; app.VelMsg.linear.y = 0;
                    app.VelMsg.angular.z = 0;
                    for k = 1:10
                        send(app.VelPub, app.VelMsg);
                        if ~isempty(app.VelNavPub)
                            app.VelNavMsg.linear.x = 0; app.VelNavMsg.linear.y = 0;
                            app.VelNavMsg.angular.z = 0;
                            send(app.VelNavPub, app.VelNavMsg);
                        end
                    end
                catch
                end
            end
            
            app.IsNavigating = false;
            app.CurrentWaypointIdx = 1;
            
            % Reset transition and target properties
            app.SlamOdomStartX = [];
            app.SlamOdomStartY = [];
            app.SlamOdomStartYaw = [];
            app.TargetX = 0;
            app.TargetY = 0;
            app.TargetFrame = 'map';
            app.MissionTimeLbl.Text = 'Mission Time: 0.0 s';
            app.MissionTimeLbl.FontColor = 'black';
            app.ProgressGauge.Value = 0;
            app.CurrentWPLbl.Text = 'WP -/-';
            cla(app.LiveMapAxes);
            grid(app.LiveMapAxes, 'on');
            title(app.LiveMapAxes, 'Live Position');
            app.RobotPlotLive = [];
        end
        
        function onLoggingChanged(app, event)
            if strcmp(event.Value, 'On')
                app.IsLogging = true;
                app.LoggingLamp.Color = 'red';
                app.LogData = [];
            else
                app.IsLogging = false;
                app.LoggingLamp.Color = 'green';
                if ~isempty(app.LogData)
                    LogData = app.LogData;
                    uisave({'LogData'}, 'mission_log.mat');
                end
            end
        end
        
        function onRefreshHistory(app, ~)
            app.refreshHistory();
        end
        
        function onLoadResult(app, ~)
            sel = app.HistoryTable.Selection;
            if isempty(sel)
                uialert(app.UIFigure, 'Select a test result first!', 'No Selection');
                return;
            end
            row = sel(1);
            name = app.HistoryTable.Data{row, 1};
            filename = fullfile(app.ResultsDir, sprintf('%s.mat', name));
            app.loadResultFile(filename, name);
        end
        
        function loadResultFile(app, filename, name)
            try
                d = load(filename);
                
                % Grafiği çiz
                cla(app.AnalysisAxes);
                if ~isempty(d.data.log)
                    plot(app.AnalysisAxes, d.data.log(:,3), d.data.log(:,4), ...
                        'b-', 'LineWidth', 1.5);
                    hold(app.AnalysisAxes, 'on');
                    
                    % Waypoints
                    wp = d.data.waypoints;
                    plot(app.AnalysisAxes, wp(:,1), wp(:,2), 'ro--', ...
                        'MarkerSize', 10, 'LineWidth', 2);
                    
                    legend(app.AnalysisAxes, {'Actual Path', 'Planned'}, 'Location', 'best');
                    title(app.AnalysisAxes, sprintf('Test: %s', name), 'Interpreter', 'none');
                    xlabel(app.AnalysisAxes, 'X (m)');
                    ylabel(app.AnalysisAxes, 'Y (m)');
                    grid(app.AnalysisAxes, 'on');
                    axis(app.AnalysisAxes, 'equal');
                    hold(app.AnalysisAxes, 'off');
                end
                
                % Metrikler
                metrics = {};
                metrics{end+1, 1} = 'Waypoints';
                metrics{end, 2} = size(d.data.waypoints, 1);
                metrics{end+1, 1} = 'Max Velocity';
                metrics{end, 2} = sprintf('%.2f m/s', d.data.params.max_vel);
                metrics{end+1, 1} = 'Goal Tolerance';
                metrics{end, 2} = sprintf('%.2f m', d.data.params.goal_tol);
                if isfield(d.data, 'mission_time')
                    metrics{end+1, 1} = 'Mission Time';
                    metrics{end, 2} = sprintf('%.1f s', d.data.mission_time);
                end
                if ~isempty(d.data.log)
                    path_err = mean(abs(d.data.log(:,4)));
                    metrics{end+1, 1} = 'Mean Y Deviation';
                    metrics{end, 2} = sprintf('%.3f m', path_err);
                    
                    max_vel = max(abs(d.data.log(:,2)));
                    metrics{end+1, 1} = 'Peak Velocity';
                    metrics{end, 2} = sprintf('%.2f m/s', max_vel);
                end
                app.MetricsTable.Data = metrics;
                
            catch ME
                uialert(app.UIFigure, ME.message, 'Error', 'Icon', 'error');
            end
        end
        
        function onDeleteResult(app, ~)
            sel = app.HistoryTable.Selection;
            if isempty(sel)
                return;
            end
            choice = uiconfirm(app.UIFigure, 'Delete this result?', 'Confirm', ...
                'Options', {'Delete', 'Cancel'});
            if strcmp(choice, 'Delete')
                row = sel(1);
                name = app.HistoryTable.Data{row, 1};
                delete(fullfile(app.ResultsDir, sprintf('%s.mat', name)));
                app.refreshHistory();
            end
        end
        
        function onGeneratePDF(app, ~)
            uialert(app.UIFigure, ...
                'PDF report generation will be implemented in next phase.', ...
                'Coming Soon', 'Icon', 'info');
        end
        
        function onExportExcel(app, ~)
            sel = app.HistoryTable.Selection;
            if isempty(sel)
                uialert(app.UIFigure, 'Select a result!', 'No Selection');
                return;
            end
            row = sel(1);
            name = app.HistoryTable.Data{row, 1};
            try
                d = load(fullfile(app.ResultsDir, sprintf('%s.mat', name)));
                [file, path] = uiputfile('*.xlsx', 'Export to Excel', [name '.xlsx']);
                if isequal(file, 0), return; end
                T = array2table(d.data.log, 'VariableNames', ...
                    {'Timestamp', 'Velocity', 'X', 'Y'});
                writetable(T, fullfile(path, file));
                uialert(app.UIFigure, 'Excel exported!', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, ME.message, 'Error', 'Icon', 'error');
            end
        end
        
        function onCloseRequest(app, ~)
            try
                app.EStopActive = false;
                if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                    stop(app.EStopTimer);
                    delete(app.EStopTimer);
                end
                if ~isempty(app.ClockTimer) && isvalid(app.ClockTimer)
                    stop(app.ClockTimer);
                    delete(app.ClockTimer);
                end
                if ~isempty(app.NavModeTimer) && isvalid(app.NavModeTimer)
                    stop(app.NavModeTimer);
                    delete(app.NavModeTimer);
                end
                clear app.OdomSub app.VelPub app.VelNavPub app.GoalPub app.ROSNode;
            catch
            end
            delete(app);
        end
        
    end
    
    %% ─── UI CONSTRUCTION (Component Initialization) ───
    methods (Access = private)
        
        function createComponents(app)
            % Main figure
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [50 50 1300 850];
            app.UIFigure.Name = 'Zeynep Mission Control Suite v2.0';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @onCloseRequest, true);
            
            % Status bar
            app.StatusPanel = uipanel(app.UIFigure);
            app.StatusPanel.Position = [10 810 1280 32];
            app.StatusPanel.BackgroundColor = [0.15 0.18 0.25];
            app.StatusPanel.BorderType = 'none';
            
            app.ROSLamp = uilamp(app.StatusPanel);
            app.ROSLamp.Position = [10 6 20 20];
            app.ROSLamp.Color = 'red';
            
            app.ROSLabel = uilabel(app.StatusPanel);
            app.ROSLabel.Position = [35 5 200 22];
            app.ROSLabel.Text = 'ROS2: Checking...';
            app.ROSLabel.FontColor = 'white';
            app.ROSLabel.FontWeight = 'bold';
            
            app.ClockLabel = uilabel(app.StatusPanel);
            app.ClockLabel.Position = [550 5 200 22];
            app.ClockLabel.Text = datestr(now);
            app.ClockLabel.FontColor = 'white';
            app.ClockLabel.HorizontalAlignment = 'center';
            
            app.EStopBtn = uibutton(app.StatusPanel, 'push');
            app.EStopBtn.Position = [1140 4 130 24];
            app.EStopBtn.Text = '🛑 EMERGENCY STOP';
            app.EStopBtn.BackgroundColor = [0.85 0.1 0.1];
            app.EStopBtn.FontColor = 'white';
            app.EStopBtn.FontWeight = 'bold';
            app.EStopBtn.ButtonPushedFcn = createCallbackFcn(app, @onEStop, true);
            
            % Tab group
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [10 10 1280 795];
            
            app.createDesignerTab();
            app.createTunerTab();
            app.createMonitorTab();
            app.createAnalyzerTab();
            
            app.UIFigure.Visible = 'on';
        end
        
        function createDesignerTab(app)
            app.DesignerTab = uitab(app.TabGroup);
            app.DesignerTab.Title = '  🎯 Mission Designer  ';
            
            % Input panel
            app.InputPanel = uipanel(app.DesignerTab);
            app.InputPanel.Title = 'Add Waypoint';
            app.InputPanel.Position = [20 645 420 125];
            app.InputPanel.FontWeight = 'bold';
            
            % GPS Modu anahtarı
            app.GPSModeLbl = uilabel(app.InputPanel);
            app.GPSModeLbl.Text = 'Koordinat:';
            app.GPSModeLbl.Position = [15 88 70 22];
            app.GPSModeLbl.FontWeight = 'bold';
            
            app.GPSModeSwitch = uiswitch(app.InputPanel, 'slider');
            app.GPSModeSwitch.Items = {'XY', 'GPS'};
            app.GPSModeSwitch.Value = 'XY';
            app.GPSModeSwitch.Position = [95 90 45 20];
            app.GPSModeSwitch.FontSize = 11;
            app.GPSModeSwitch.ValueChangedFcn = createCallbackFcn(app, @onGPSModeChanged, true);
            
            uilabel(app.InputPanel, 'Text', '🛰️ Lat/Lon veya XY seç', ...
                'Position', [160 88 250 22], 'FontColor', [0.4 0.4 0.5], 'FontSize', 11);
            
            app.XLabel = uilabel(app.InputPanel, 'Text', 'X (m):', 'Position', [15 55 50 22]);
            app.XField = uieditfield(app.InputPanel, 'numeric', 'Position', [70 55 80 22]);
            
            app.YLabel = uilabel(app.InputPanel, 'Text', 'Y (m):', 'Position', [160 55 50 22]);
            app.YField = uieditfield(app.InputPanel, 'numeric', 'Position', [215 55 80 22]);
            
            uilabel(app.InputPanel, 'Text', 'θ (deg):', 'Position', [15 25 55 22]);
            app.ThetaField = uieditfield(app.InputPanel, 'numeric', 'Position', [70 25 80 22]);
            
            app.AddBtn = uibutton(app.InputPanel, 'push');
            app.AddBtn.Position = [215 22 190 30];
            app.AddBtn.Text = '➕ Add Waypoint';
            app.AddBtn.BackgroundColor = [0.3 0.7 0.3];
            app.AddBtn.FontColor = 'white';
            app.AddBtn.FontWeight = 'bold';
            app.AddBtn.ButtonPushedFcn = createCallbackFcn(app, @onAddBtn, true);
            
            % Quick Mission Panel (Designer Tab)
            app.QuickMissionPanel = uipanel(app.DesignerTab);
            app.QuickMissionPanel.Title = 'Quick Mission Load';
            app.QuickMissionPanel.Position = [20 510 420 125];
            app.QuickMissionPanel.FontWeight = 'bold';

            app.ScenarioDropDown = uidropdown(app.QuickMissionPanel);
            app.ScenarioDropDown.Position = [15 65 240 24];
            app.ScenarioDropDown.Items = {'No scenarios found'};
            app.ScenarioDropDown.ItemsData = {''};
            app.ScenarioDropDown.ValueChangedFcn = createCallbackFcn(app, @onScenarioChanged, true);

            app.QuickLoadBtn = uibutton(app.QuickMissionPanel, 'push');
            app.QuickLoadBtn.Position = [265 62 140 28];
            app.QuickLoadBtn.Text = '⚡ Load Waypoints';
            app.QuickLoadBtn.BackgroundColor = [0.2 0.5 0.8];
            app.QuickLoadBtn.FontColor = 'white';
            app.QuickLoadBtn.FontWeight = 'bold';
            app.QuickLoadBtn.ButtonPushedFcn = createCallbackFcn(app, @onQuickLoad, true);

            app.AutoParamsBtn = uibutton(app.QuickMissionPanel, 'push');
            app.AutoParamsBtn.Position = [265 28 140 28];
            app.AutoParamsBtn.Text = '⚙️ Auto Params';
            app.AutoParamsBtn.BackgroundColor = [0.7 0.5 0.1];
            app.AutoParamsBtn.FontColor = 'white';
            app.AutoParamsBtn.FontWeight = 'bold';
            app.AutoParamsBtn.ButtonPushedFcn = createCallbackFcn(app, @onAutoParams, true);

            app.QuickMissionInfoLbl = uilabel(app.QuickMissionPanel);
            app.QuickMissionInfoLbl.Position = [15 5 240 50];
            app.QuickMissionInfoLbl.Text = 'No scenario active.';
            app.QuickMissionInfoLbl.WordWrap = true;
            app.QuickMissionInfoLbl.FontAngle = 'italic';
            app.QuickMissionInfoLbl.FontColor = [0.4 0.4 0.4];

            % Action panel
            app.ActionPanel = uipanel(app.DesignerTab);
            app.ActionPanel.Title = 'Import / Export';
            app.ActionPanel.Position = [20 390 420 110];
            app.ActionPanel.FontWeight = 'bold';
            
            app.ImportCSVBtn = uibutton(app.ActionPanel, 'push');
            app.ImportCSVBtn.Position = [15 50 95 30];
            app.ImportCSVBtn.Text = '📁 Import CSV';
            app.ImportCSVBtn.ButtonPushedFcn = createCallbackFcn(app, @onImportCSV, true);
            
            app.ExportCSVBtn = uibutton(app.ActionPanel, 'push');
            app.ExportCSVBtn.Position = [120 50 95 30];
            app.ExportCSVBtn.Text = '💾 Export CSV';
            app.ExportCSVBtn.ButtonPushedFcn = createCallbackFcn(app, @onExportCSV, true);
            
            app.SaveMissionBtn = uibutton(app.ActionPanel, 'push');
            app.SaveMissionBtn.Position = [225 50 95 30];
            app.SaveMissionBtn.Text = '⭐ Save Mission';
            app.SaveMissionBtn.ButtonPushedFcn = createCallbackFcn(app, @onSaveMission, true);
            
            app.LoadMissionBtn = uibutton(app.ActionPanel, 'push');
            app.LoadMissionBtn.Position = [325 50 95 30];
            app.LoadMissionBtn.Text = '📂 Load Mission';
            app.LoadMissionBtn.ButtonPushedFcn = createCallbackFcn(app, @onLoadMission, true);
            
            app.WaypointCountLbl = uilabel(app.ActionPanel);
            app.WaypointCountLbl.Position = [15 15 200 22];
            app.WaypointCountLbl.Text = 'Waypoints: 0';
            app.WaypointCountLbl.FontWeight = 'bold';
            
            app.TotalDistLbl = uilabel(app.ActionPanel);
            app.TotalDistLbl.Position = [225 15 195 22];
            app.TotalDistLbl.Text = 'Total Distance: 0.00 m';
            app.TotalDistLbl.FontWeight = 'bold';
            
            % Waypoint table
            app.WaypointTable = uitable(app.DesignerTab);
            app.WaypointTable.Position = [20 100 420 280];
            app.WaypointTable.ColumnName = {'X (m)', 'Y (m)', 'θ (deg)'};
            app.WaypointTable.ColumnEditable = [true true true];
            app.WaypointTable.SelectionType = 'row';
            app.WaypointTable.CellEditCallback = createCallbackFcn(app, @onTableEdit, true);
            
            % Table control buttons
            app.DeleteWpBtn = uibutton(app.DesignerTab, 'push');
            app.DeleteWpBtn.Position = [20 60 95 30];
            app.DeleteWpBtn.Text = '🗑 Delete';
            app.DeleteWpBtn.BackgroundColor = [0.9 0.5 0.5];
            app.DeleteWpBtn.ButtonPushedFcn = createCallbackFcn(app, @onDeleteWp, true);
            
            app.MoveUpBtn = uibutton(app.DesignerTab, 'push');
            app.MoveUpBtn.Position = [120 60 95 30];
            app.MoveUpBtn.Text = '⬆ Move Up';
            app.MoveUpBtn.ButtonPushedFcn = createCallbackFcn(app, @onMoveUp, true);
            
            app.MoveDownBtn = uibutton(app.DesignerTab, 'push');
            app.MoveDownBtn.Position = [220 60 95 30];
            app.MoveDownBtn.Text = '⬇ Move Down';
            app.MoveDownBtn.ButtonPushedFcn = createCallbackFcn(app, @onMoveDown, true);
            
            app.ClearAllBtn = uibutton(app.DesignerTab, 'push');
            app.ClearAllBtn.Position = [320 60 120 30];
            app.ClearAllBtn.Text = '🗑 Clear All';
            app.ClearAllBtn.BackgroundColor = [0.9 0.3 0.3];
            app.ClearAllBtn.FontColor = 'white';
            app.ClearAllBtn.ButtonPushedFcn = createCallbackFcn(app, @onClearAll, true);
            
            % Map visualization
            app.MapAxes = uiaxes(app.DesignerTab);
            app.MapAxes.Position = [460 100 810 640];
            title(app.MapAxes, 'Mission Route');
            xlabel(app.MapAxes, 'X (m)');
            ylabel(app.MapAxes, 'Y (m)');
            grid(app.MapAxes, 'on');
            app.MapAxes.ButtonDownFcn = createCallbackFcn(app, @onMapClick, true);

            % ═══ Dünya seçimi (haritanın üstünde) ═══
            app.WorldLbl = uilabel(app.DesignerTab);
            app.WorldLbl.Position = [460 748 60 24];
            app.WorldLbl.Text = '🌍 World:';
            app.WorldLbl.FontWeight = 'bold';

            app.WorldDropDown = uidropdown(app.DesignerTab);
            app.WorldDropDown.Position = [522 748 280 24];
            app.WorldDropDown.Items = {'Obstacles / Tunnel (hybrid)','Empty','Urban', ...
                'Industrial','Open Terrain','Sloped Terrain','Earendil Env', ...
                'Clearpath Warehouse','Clearpath Office','Clearpath Orchard', ...
                'Clearpath Pipeline','Clearpath Solar Farm','Clearpath Construction'};
            app.WorldDropDown.Value = 'Obstacles / Tunnel (hybrid)';
            app.WorldDropDown.ValueChangedFcn = createCallbackFcn(app, @onWorldChanged, true);

            % Güvenlik durum etiketi
            app.SafetyLbl = uilabel(app.DesignerTab);
            app.SafetyLbl.Position = [770 748 360 24];
            app.SafetyLbl.Text = 'ℹ️ Engeller haritada gri; kırmızı kesik çizgi = güvenli sınır';
            app.SafetyLbl.FontColor = [0.4 0.4 0.4];

            % Robotu kurtar butonu
            app.RecoverBtn = uibutton(app.DesignerTab, 'push');
            app.RecoverBtn.Position = [1130 748 140 24];
            app.RecoverBtn.Text = '🚑 Robotu Kurtar';
            app.RecoverBtn.BackgroundColor = [0.95 0.7 0.2];
            app.RecoverBtn.FontWeight = 'bold';
            app.RecoverBtn.ButtonPushedFcn = createCallbackFcn(app, @onRecoverRobot, true);
        end
        
        function createTunerTab(app)
            app.TunerTab = uitab(app.TabGroup);
            app.TunerTab.Title = '  ⚙️ Parameter Tuner  ';
            
            % Preset panel
            app.PresetPanel = uibuttongroup(app.TunerTab);
            app.PresetPanel.Title = 'Quick Presets';
            app.PresetPanel.Position = [20 670 420 100];
            app.PresetPanel.FontWeight = 'bold';
            app.PresetPanel.SelectionChangedFcn = createCallbackFcn(app, @onPresetChanged, true);
            
            app.SlowBtn = uiradiobutton(app.PresetPanel);
            app.SlowBtn.Position = [20 50 80 22];
            app.SlowBtn.Text = '🐢 Slow';
            
            app.NormalBtn = uiradiobutton(app.PresetPanel);
            app.NormalBtn.Position = [120 50 80 22];
            app.NormalBtn.Text = '🚶 Normal';
            
            app.FastBtn = uiradiobutton(app.PresetPanel);
            app.FastBtn.Position = [220 50 80 22];
            app.FastBtn.Text = '🏃 Fast';
            
            app.CustomBtn = uiradiobutton(app.PresetPanel);
            app.CustomBtn.Position = [320 50 80 22];
            app.CustomBtn.Text = '⚙️ Custom';
            
            uilabel(app.PresetPanel, 'Position', [20 15 380 22], ...
                'Text', 'Slow: precise indoor • Normal: balanced • Fast: open area', ...
                'FontAngle', 'italic');
            
            % Parameter sliders
            y_start = 620;
            spacing = 95;
            
            app.createParamSlider(1, y_start, 'Max Velocity', 'm/s', 0, 2, 0.5, ...
                'Higher = faster, but harder to stop at goals');
            app.createParamSlider(2, y_start - spacing, 'Max Acceleration', 'm/s²', 0, 1, 0.3, ...
                'Higher = quicker speedup, may cause oscillation');
            app.createParamSlider(3, y_start - 2*spacing, 'Braking Time', 's', 0.1, 2, 0.5, ...
                'Higher = smoother stop, larger overshoot');
            app.createParamSlider(4, y_start - 3*spacing, 'Safe Distance', 'm', 0.1, 1, 0.3, ...
                'Higher = safer from obstacles, may detour');
            app.createParamSlider(5, y_start - 4*spacing, 'Goal Tolerance', 'm', 0.1, 1, 0.3, ...
                'Higher = stops earlier; recommend 0.3-0.5');
            
            % YAML preview
            uilabel(app.TunerTab, 'Position', [470 740 200 30], ...
                'Text', '📄 Live YAML Preview', 'FontSize', 14, 'FontWeight', 'bold');
            
            app.YAMLArea = uitextarea(app.TunerTab);
            app.YAMLArea.Position = [470 130 800 610];
            app.YAMLArea.FontName = 'Monospaced';
            app.YAMLArea.Editable = 'off';
            
            app.GenerateYAMLBtn = uibutton(app.TunerTab, 'push');
            app.GenerateYAMLBtn.Position = [470 90 200 30];
            app.GenerateYAMLBtn.Text = '🔄 Refresh YAML';
            app.GenerateYAMLBtn.ButtonPushedFcn = createCallbackFcn(app, @onGenerateYAML, true);
            
            app.SaveYAMLBtn = uibutton(app.TunerTab, 'push');
            app.SaveYAMLBtn.Position = [680 90 200 30];
            app.SaveYAMLBtn.Text = '💾 Save YAML to File';
            app.SaveYAMLBtn.BackgroundColor = [0.3 0.5 0.8];
            app.SaveYAMLBtn.FontColor = 'white';
            app.SaveYAMLBtn.FontWeight = 'bold';
            app.SaveYAMLBtn.ButtonPushedFcn = createCallbackFcn(app, @onSaveYAML, true);

            % ─── Parameter Sweep Panel ───
            app.SweepPanel = uipanel(app.TunerTab);
            app.SweepPanel.Title = 'Parameter Sweep';
            app.SweepPanel.Position = [20 20 420 190];
            app.SweepPanel.FontWeight = 'bold';

            uilabel(app.SweepPanel, 'Position', [15 130 100 22], 'Text', 'Swept Param:', 'FontWeight', 'bold');
            app.SweepParamDropDown = uidropdown(app.SweepPanel);
            app.SweepParamDropDown.Position = [130 130 260 22];
            app.SweepParamDropDown.Items = {'Max Velocity', 'Max Acceleration', 'Braking Time', 'Safe Distance', 'Goal Tolerance'};
            app.SweepParamDropDown.Value = 'Max Velocity';

            uilabel(app.SweepPanel, 'Position', [15 90 30 22], 'Text', 'Min:');
            app.SweepMinField = uieditfield(app.SweepPanel, 'numeric', 'Position', [50 90 60 22]);
            app.SweepMinField.Value = 0.2;

            uilabel(app.SweepPanel, 'Position', [130 90 30 22], 'Text', 'Max:');
            app.SweepMaxField = uieditfield(app.SweepPanel, 'numeric', 'Position', [165 90 60 22]);
            app.SweepMaxField.Value = 1.0;

            uilabel(app.SweepPanel, 'Position', [245 90 35 22], 'Text', 'Step:');
            app.SweepStepField = uieditfield(app.SweepPanel, 'numeric', 'Position', [285 90 60 22]);
            app.SweepStepField.Value = 0.2;

            app.SweepPlanBtn = uibutton(app.SweepPanel, 'push');
            app.SweepPlanBtn.Position = [15 50 380 30];
            app.SweepPlanBtn.Text = '⚡ Generate Sweep Plan';
            app.SweepPlanBtn.BackgroundColor = [0.2 0.6 0.4];
            app.SweepPlanBtn.FontColor = 'white';
            app.SweepPlanBtn.FontWeight = 'bold';
            app.SweepPlanBtn.ButtonPushedFcn = createCallbackFcn(app, @onSweepPlan, true);

            app.SweepInfoLbl = uilabel(app.SweepPanel);
            app.SweepInfoLbl.Position = [15 15 380 25];
            app.SweepInfoLbl.Text = 'No active sweep plan.';
            app.SweepInfoLbl.FontAngle = 'italic';
            app.SweepInfoLbl.FontColor = [0.4 0.4 0.4];
        end
        
        function createParamSlider(app, idx, y, label, unit, mn, mx, def, info)
            % Helper function to create parameter row
            lbl = uilabel(app.TunerTab);
            lbl.Position = [20 y+30 200 22];
            lbl.Text = sprintf('%s (%s):', label, unit);
            lbl.FontWeight = 'bold';
            lbl.FontSize = 13;
            
            slider = uislider(app.TunerTab);
            slider.Position = [20 y+15 350 3];
            slider.Limits = [mn mx];
            slider.Value = def;
            slider.ValueChangedFcn = createCallbackFcn(app, @onSliderChanged, true);
            
            field = uieditfield(app.TunerTab, 'numeric');
            field.Position = [380 y+5 60 22];
            field.Value = def;
            field.Limits = [mn mx];
            field.ValueChangedFcn = createCallbackFcn(app, @onParamChanged, true);
            
            info_lbl = uilabel(app.TunerTab);
            info_lbl.Position = [20 y-10 420 22];
            info_lbl.Text = ['💡 ' info];
            info_lbl.FontAngle = 'italic';
            info_lbl.FontColor = [0.4 0.4 0.5];
            info_lbl.FontSize = 11;
            
            switch idx
                case 1
                    app.MaxVelSlider = slider; app.MaxVelField = field; app.MaxVelInfo = info_lbl;
                case 2
                    app.MaxAccSlider = slider; app.MaxAccField = field; app.MaxAccInfo = info_lbl;
                case 3
                    app.BrakingSlider = slider; app.BrakingField = field; app.BrakingInfo = info_lbl;
                case 4
                    app.SafeDistSlider = slider; app.SafeDistField = field; app.SafeDistInfo = info_lbl;
                case 5
                    app.GoalTolSlider = slider; app.GoalTolField = field; app.GoalTolInfo = info_lbl;
            end
        end
        
        function createMonitorTab(app)
            app.MonitorTab = uitab(app.TabGroup);
            app.MonitorTab.Title = '  📡 Live Monitor  ';
            
            % Live position map
            app.LiveMapAxes = uiaxes(app.MonitorTab);
            app.LiveMapAxes.Position = [20 100 700 670];
            title(app.LiveMapAxes, 'Live Robot Position');
            xlabel(app.LiveMapAxes, 'X (m)');
            ylabel(app.LiveMapAxes, 'Y (m)');
            grid(app.LiveMapAxes, 'on');
            
            % Right panel
            % Velocity gauge
            app.VelGaugeLbl = uilabel(app.MonitorTab);
            app.VelGaugeLbl.Position = [780 720 200 22];
            app.VelGaugeLbl.Text = 'Current Velocity';
            app.VelGaugeLbl.FontWeight = 'bold';
            app.VelGaugeLbl.FontSize = 14;
            app.VelGaugeLbl.HorizontalAlignment = 'center';
            
            app.VelGauge = uigauge(app.MonitorTab, 'circular');
            app.VelGauge.Position = [780 540 200 180];
            app.VelGauge.Limits = [0 2];
            app.VelGauge.MajorTicks = [0 0.5 1 1.5 2];
            
            % Distance gauge
            app.DistGaugeLbl = uilabel(app.MonitorTab);
            app.DistGaugeLbl.Position = [1010 720 200 22];
            app.DistGaugeLbl.Text = 'Distance to Goal';
            app.DistGaugeLbl.FontWeight = 'bold';
            app.DistGaugeLbl.FontSize = 14;
            app.DistGaugeLbl.HorizontalAlignment = 'center';
            
            app.DistGauge = uigauge(app.MonitorTab, 'linear');
            app.DistGauge.Position = [1010 670 250 50];
            app.DistGauge.Limits = [0 5];
            
            % Progress
            uilabel(app.MonitorTab, 'Position', [1010 620 200 22], ...
                'Text', 'Mission Progress (%)', 'FontWeight', 'bold', 'FontSize', 14);
            
            app.ProgressGauge = uigauge(app.MonitorTab, 'linear');
            app.ProgressGauge.Position = [1010 570 250 50];
            app.ProgressGauge.Limits = [0 100];
            
            % Status labels
            app.CurrentWPLbl = uilabel(app.MonitorTab);
            app.CurrentWPLbl.Position = [780 500 250 22];
            app.CurrentWPLbl.Text = 'WP -/-';
            app.CurrentWPLbl.FontWeight = 'bold';
            app.CurrentWPLbl.FontSize = 13;
            app.CurrentWPLbl.HorizontalAlignment = 'center';
            
            app.MissionTimeLbl = uilabel(app.MonitorTab);
            app.MissionTimeLbl.Position = [780 470 250 22];
            app.MissionTimeLbl.Text = 'Mission Time: 0.0 s';
            app.MissionTimeLbl.FontWeight = 'bold';
            app.MissionTimeLbl.FontSize = 13;
            app.MissionTimeLbl.HorizontalAlignment = 'center';
            
            % Obstacle alert
            app.ObstacleLbl = uilabel(app.MonitorTab);
            app.ObstacleLbl.Position = [1060 500 150 22];
            app.ObstacleLbl.Text = 'Obstacle Alert';
            app.ObstacleLbl.FontWeight = 'bold';
            
            app.ObstacleLamp = uilamp(app.MonitorTab);
            app.ObstacleLamp.Position = [1230 498 25 25];
            
            % ═══ NAV MODE GÖSTERGESİ (GPS/SLAM Hibrit) ═══
            app.NavModeLbl = uilabel(app.MonitorTab);
            app.NavModeLbl.Position = [780 360 200 22];
            app.NavModeLbl.Text = '🛰️ Navigation Mode';
            app.NavModeLbl.FontWeight = 'bold';
            app.NavModeLbl.FontSize = 14;
            
            app.NavModeLamp = uilamp(app.MonitorTab);
            app.NavModeLamp.Position = [780 325 30 30];
            app.NavModeLamp.Color = [0.5 0.5 0.5];
            
            app.NavModeValueLbl = uilabel(app.MonitorTab);
            app.NavModeValueLbl.Position = [820 325 200 30];
            app.NavModeValueLbl.Text = 'Bağlanıyor...';
            app.NavModeValueLbl.FontWeight = 'bold';
            app.NavModeValueLbl.FontSize = 18;
            
            app.NavModeCountLbl = uilabel(app.MonitorTab);
            app.NavModeCountLbl.Position = [780 300 300 22];
            app.NavModeCountLbl.Text = 'Mod geçiş sayısı: 0';
            app.NavModeCountLbl.FontSize = 12;
            app.NavModeCountLbl.FontColor = [0.4 0.4 0.5];
            
            % Logging
            app.LoggingSwitchLbl = uilabel(app.MonitorTab);
            app.LoggingSwitchLbl.Position = [780 420 100 22];
            app.LoggingSwitchLbl.Text = 'Data Logging';
            app.LoggingSwitchLbl.FontWeight = 'bold';
            
            app.LoggingSwitch = uiswitch(app.MonitorTab, 'slider');
            app.LoggingSwitch.Position = [890 420 45 20];
            app.LoggingSwitch.ValueChangedFcn = createCallbackFcn(app, @onLoggingChanged, true);
            
            app.LoggingLampLbl = uilabel(app.MonitorTab);
            app.LoggingLampLbl.Position = [970 420 100 22];
            app.LoggingLampLbl.Text = 'Recording:';
            
            app.LoggingLamp = uilamp(app.MonitorTab);
            app.LoggingLamp.Position = [1060 418 25 25];
            app.LoggingLamp.Color = 'green';
            
            % Mission control buttons
            app.StartMissionBtn = uibutton(app.MonitorTab, 'push');
            app.StartMissionBtn.Position = [780 200 230 60];
            app.StartMissionBtn.Text = '🚀 START MISSION';
            app.StartMissionBtn.FontSize = 18;
            app.StartMissionBtn.FontWeight = 'bold';
            app.StartMissionBtn.BackgroundColor = [0.2 0.7 0.2];
            app.StartMissionBtn.FontColor = 'white';
            app.StartMissionBtn.ButtonPushedFcn = createCallbackFcn(app, @onStartMission, true);
            
            app.ResetBtn = uibutton(app.MonitorTab, 'push');
            app.ResetBtn.Position = [1030 200 230 60];
            app.ResetBtn.Text = '🔄 RESET';
            app.ResetBtn.FontSize = 18;
            app.ResetBtn.FontWeight = 'bold';
            app.ResetBtn.BackgroundColor = [0.5 0.5 0.7];
            app.ResetBtn.FontColor = 'white';
            app.ResetBtn.ButtonPushedFcn = createCallbackFcn(app, @onReset, true);
        end
        
        function createAnalyzerTab(app)
            app.AnalyzerTab = uitab(app.TabGroup);
            app.AnalyzerTab.Title = '  📊 Results Analyzer  ';
            
            % Test history table
            uilabel(app.AnalyzerTab, 'Position', [20 740 200 22], ...
                'Text', '📋 Test History', 'FontSize', 14, 'FontWeight', 'bold');
            
            app.HistoryTable = uitable(app.AnalyzerTab);
            app.HistoryTable.Position = [20 380 540 360];
            app.HistoryTable.ColumnName = {'Test Name', 'Waypoints', 'Max Vel', 'Time', 'Date'};
            app.HistoryTable.ColumnWidth = {180, 70, 60, 70, 140};
            app.HistoryTable.SelectionType = 'row';
            
            app.RefreshHistoryBtn = uibutton(app.AnalyzerTab, 'push');
            app.RefreshHistoryBtn.Position = [20 340 130 30];
            app.RefreshHistoryBtn.Text = '🔄 Refresh';
            app.RefreshHistoryBtn.ButtonPushedFcn = createCallbackFcn(app, @onRefreshHistory, true);
            
            app.LoadResultBtn = uibutton(app.AnalyzerTab, 'push');
            app.LoadResultBtn.Position = [160 340 130 30];
            app.LoadResultBtn.Text = '📂 Load';
            app.LoadResultBtn.BackgroundColor = [0.3 0.5 0.8];
            app.LoadResultBtn.FontColor = 'white';
            app.LoadResultBtn.ButtonPushedFcn = createCallbackFcn(app, @onLoadResult, true);
            
            app.DeleteResultBtn = uibutton(app.AnalyzerTab, 'push');
            app.DeleteResultBtn.Position = [300 340 130 30];
            app.DeleteResultBtn.Text = '🗑 Delete';
            app.DeleteResultBtn.BackgroundColor = [0.8 0.3 0.3];
            app.DeleteResultBtn.FontColor = 'white';
            app.DeleteResultBtn.ButtonPushedFcn = createCallbackFcn(app, @onDeleteResult, true);
            
            % Metrics table
            uilabel(app.AnalyzerTab, 'Position', [20 290 200 22], ...
                'Text', '📈 Metrics', 'FontSize', 14, 'FontWeight', 'bold');
            
            app.MetricsTable = uitable(app.AnalyzerTab);
            app.MetricsTable.Position = [20 50 540 240];
            app.MetricsTable.ColumnName = {'Metric', 'Value'};
            app.MetricsTable.ColumnWidth = {200, 280};
            
            % Analysis plot
            uilabel(app.AnalyzerTab, 'Position', [590 740 200 22], ...
                'Text', '📍 Trajectory Analysis', 'FontSize', 14, 'FontWeight', 'bold');
            
            app.AnalysisAxes = uiaxes(app.AnalyzerTab);
            app.AnalysisAxes.Position = [590 100 680 640];
            title(app.AnalysisAxes, 'Select a test from history');
            xlabel(app.AnalysisAxes, 'X (m)');
            ylabel(app.AnalysisAxes, 'Y (m)');
            grid(app.AnalysisAxes, 'on');
            
            % Export buttons
            app.GeneratePDFBtn = uibutton(app.AnalyzerTab, 'push');
            app.GeneratePDFBtn.Position = [590 50 200 40];
            app.GeneratePDFBtn.Text = '📄 Generate PDF Report';
            app.GeneratePDFBtn.BackgroundColor = [0.8 0.3 0.3];
            app.GeneratePDFBtn.FontColor = 'white';
            app.GeneratePDFBtn.FontWeight = 'bold';
            app.GeneratePDFBtn.FontSize = 13;
            app.GeneratePDFBtn.ButtonPushedFcn = createCallbackFcn(app, @onGeneratePDF, true);
            
            app.ExportExcelBtn = uibutton(app.AnalyzerTab, 'push');
            app.ExportExcelBtn.Position = [800 50 200 40];
            app.ExportExcelBtn.Text = '📊 Export to Excel';
            app.ExportExcelBtn.BackgroundColor = [0.2 0.6 0.4];
            app.ExportExcelBtn.FontColor = 'white';
            app.ExportExcelBtn.FontWeight = 'bold';
            app.ExportExcelBtn.FontSize = 13;
            app.ExportExcelBtn.ButtonPushedFcn = createCallbackFcn(app, @onExportExcel, true);
        end
        
    end
    
    %% ─── PUBLIC INTERFACE ───
    methods (Access = public)
        
        function app = mission_control_v2_DENEME
            createComponents(app);
            registerApp(app, app.UIFigure);
            runStartupFcn(app, @startupFcn);
            if nargout == 0
                clear app
            end
        end
        
        function delete(app)
            try
                if ~isempty(app.ClockTimer) && isvalid(app.ClockTimer)
                    stop(app.ClockTimer);
                    delete(app.ClockTimer);
                end
                if ~isempty(app.NavModeTimer) && isvalid(app.NavModeTimer)
                    stop(app.NavModeTimer);
                    delete(app.NavModeTimer);
                end
                if ~isempty(app.EStopTimer) && isvalid(app.EStopTimer)
                    stop(app.EStopTimer);
                    delete(app.EStopTimer);
                end
            catch
            end
            delete(app.UIFigure);
        end
        
    end
    
end
