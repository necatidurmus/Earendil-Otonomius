function generate_mission_library_DENEME()
% GENERATE_MISSION_LIBRARY_DENEME  13 dünya × 4 senaryo = 52 mission dosyası oluşturur.
%   ~/zeynep_missions_deneme/ dizinine kaydeder.
%   Her mission: waypoints (Nx3), params, world, scenario, description içerir.
%
%   Kullanım:
%     generate_mission_library_DENEME()

    missionsDir = fullfile(getenv('HOME'), 'zeynep_missions_deneme');
    if ~exist(missionsDir, 'dir'), mkdir(missionsDir); end

    fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║   ZEYNEP MISSION LIBRARY GENERATOR                      ║\n');
    fprintf('║   13 Dünya × 4 Senaryo = 52 Mission                     ║\n');
    fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

    count = 0;
    worlds = getAllWorlds();

    for w = 1:length(worlds)
        wd = worlds(w);
        fprintf('── %s ──\n', upper(wd.key));
        for s = 1:4
            [wp, par, sc_key, sc_desc] = getScenario(wd, s);
            mission = struct();
            mission.name = sprintf('%s_%s', wd.key, sc_key);
            mission.waypoints = wp;
            mission.params = par;
            mission.world = wd.key;
            mission.scenario = sc_key;
            mission.description = sc_desc;
            mission.created = datestr(now);
            filename = fullfile(missionsDir, sprintf('%s.mat', mission.name));
            save(filename, 'mission');
            count = count + 1;
            fprintf('  [%02d] %-40s  (%d WP) ✓\n', count, mission.name, size(wp,1));
        end
    end

    fprintf('\n✅ %d mission dosyası oluşturuldu: %s\n\n', count, missionsDir);
end

%% ─── DÜNYA TANIMLARI ────────────────────────────────────────────────────
function worlds = getAllWorlds()
    w = struct('key','','type','','wp_safe',[],'wp_challenge',[],'wp_speed',[],'wp_precision',[]);
    worlds = repmat(w, 13, 1);
    idx = 0;

    % 1. obstacles
    idx=idx+1; worlds(idx).key='obstacles'; worlds(idx).type='hybrid';
    worlds(idx).wp_safe      = [0 0 0; -8 -10 0; 8 -10 0; 12 12 0; -10 12 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 6 -5 -30; 10 -10.5 0; 18 -10.5 0; 19 0 90; 25 0 0; 31 0 0; 25 0 180; 19 0 180; 18 -10.5 180; 10 -10.5 180; 6 -5 180; 0 0 180];
    worlds(idx).wp_speed     = [0 0 0; 15 -15 0; -15 -15 180; -15 15 90; 15 15 0; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];

    % 2. empty
    idx=idx+1; worlds(idx).key='empty'; worlds(idx).type='outdoor_open';
    worlds(idx).wp_safe      = [0 0 0; 10 0 0; 10 10 90; 0 10 180; -10 10 180; -10 0 270; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 8 4 30; 0 8 120; -8 4 210; -8 -4 240; 0 -8 300; 8 -4 330; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 20 0 0; 20 20 90; -20 20 180; -20 -20 270; 20 -20 0; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 0.5 0 0; 0.5 0.5 90; 0 0.5 180; 0 0 270];

    % 3. urban (Dört yol ağzı: x=0 ve y=0)
    idx=idx+1; worlds(idx).key='urban'; worlds(idx).type='urban';
    worlds(idx).wp_safe      = [0 0 0; 0 20 90; 0 0 270; 20 0 0; 0 0 180; 0 -20 270; 0 0 90];
    worlds(idx).wp_challenge = [0 0 0; -20 0 180; -20 15 90; 0 15 0; 0 -15 270; 15 -15 0; 15 0 90; 0 0 180];
    worlds(idx).wp_speed     = [0 0 0; 0 22 90; 0 -22 270; 0 0 90];
    worlds(idx).wp_precision = [0 0 0; 2 0 0; 2 2 90; 0 2 180; 0 0 270];

    % 4. industrial (Raf araları ve cross koridorlara uygun devriye)
    idx=idx+1; worlds(idx).key='industrial'; worlds(idx).type='indoor';
    % Safe Patrol: Ana koridorlarda geniş turlar (x=0 ve x=10)
    worlds(idx).wp_safe      = [0 0 0; 0 20 90; 10 20 0; 10 -20 270; 0 -20 180; 0 0 90];
    % Challenge Route: Raflar arası dar yatay geçişler (y=10, y=0) kullanarak zig-zag
    worlds(idx).wp_challenge = [0 0 0; 20 0 0; 20 10 90; -10 10 180; -10 20 90; 10 20 0; 10 0 270; 0 0 180];
    % Speed Test: Uzun dikey raf koridoru (x=0)
    worlds(idx).wp_speed     = [0 0 0; 0 22 90; 0 -22 270; 0 0 90];
    worlds(idx).wp_precision = [0 0 0; 0 2 90; 2 2 0; 2 0 270; 0 0 180];

    % 5. open_terrain
    idx=idx+1; worlds(idx).key='open_terrain'; worlds(idx).type='outdoor_open';
    worlds(idx).wp_safe      = [0 0 0; 10 5 0; 15 -5 0; 5 -15 0; -10 -10 0; -15 5 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 5 0 0; 10 5 45; 20 10 0; 15 -5 315; 0 -10 180; -15 -10 180; -10 5 90; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 20 0 0; 20 20 90; -20 0 180; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 2 0 0; 2 2 90; 0 2 180; 0 0 270];

    % 6. sloped_terrain
    idx=idx+1; worlds(idx).key='sloped_terrain'; worlds(idx).type='terrain';
    worlds(idx).wp_safe      = [0 0 0; 5 5 0; 10 0 0; 5 -5 0; -5 -5 0; -10 0 0; -5 5 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 0 15 0; 15 0 0; 0 -15 180; -15 0 270; -20 -20 225; 20 20 45; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 15 0 0; -15 0 180; 0 15 90; 0 -15 270; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];

    % 7. earendil_env
    idx=idx+1; worlds(idx).key='earendil_env'; worlds(idx).type='hybrid';
    worlds(idx).wp_safe      = [0 0 0; 5 5 0; 5 15 0; -5 15 0; -5 5 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 5 20 0; 7 28 0; -5 25 180; -7 -15 270; -8 -19 225; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 0 20 0; 0 -15 180; 15 0 90; -15 0 270; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 1 45; -1 1 135; 0 0 0];

    % 8. clearpath_warehouse
    idx=idx+1; worlds(idx).key='clearpath_warehouse'; worlds(idx).type='indoor';
    worlds(idx).wp_safe      = [0 0 0; 8 4 0; 8 -4 0; -8 -4 180; -8 4 90; 0 12 0; 0 -12 180; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; -3 5 90; -3 -5 270; 3 5 90; 3 -5 270; 10 0 0; -10 0 180; 5 10 45; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 15 0 0; -15 0 180; 0 12 90; 0 -12 270; 0 0 0];
    worlds(idx).wp_precision = [0 3 0; 0 -3 180; 3 0 90; -3 0 270; 0 3 0];

    % 9. clearpath_office
    idx=idx+1; worlds(idx).key='clearpath_office'; worlds(idx).type='indoor';
    worlds(idx).wp_safe      = [0 0 0; 10 0 0; 10 10 90; 0 10 180; -10 0 270; 0 -10 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 15 0 0; 20 -3 0; 15 -8 180; 5 -5 180; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 20 0 0; -10 0 180; 0 10 90; 0 -10 270; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];

    % 10. clearpath_orchard
    idx=idx+1; worlds(idx).key='clearpath_orchard'; worlds(idx).type='nature';
    worlds(idx).wp_safe      = [0 0 0; 10 0 0; 10 10 90; 0 10 180; -10 10 180; -10 -10 270; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; -5 -5 225; -10 -10 225; -5 -15 315; 5 -10 0; 10 0 45; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 15 15 45; -15 -15 225; 15 -15 315; -15 15 135; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];

    % 11. clearpath_pipeline
    idx=idx+1; worlds(idx).key='clearpath_pipeline'; worlds(idx).type='nature';
    worlds(idx).wp_safe      = [0 0 0; 10 0 0; 10 10 90; -10 10 180; -10 0 270; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 0 5 0; 5 10 45; 0 18 90; -5 15 180; -5 5 270; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 20 0 0; 20 25 90; -20 0 180; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 2 0 0; 2 2 90; 0 2 180; 0 0 270];

    % 12. clearpath_solar_farm
    idx=idx+1; worlds(idx).key='clearpath_solar_farm'; worlds(idx).type='outdoor_open';
    worlds(idx).wp_safe      = [0 0 0; 10 5 0; 15 -5 0; 5 -15 0; -10 -5 180; -15 5 90; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; -5 -4 225; -20 -8 180; -28 -10 180; -20 -5 0; -5 0 0; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 20 0 0; 20 15 90; -20 15 180; -20 -15 270; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];

    % 13. clearpath_construction
    idx=idx+1; worlds(idx).key='clearpath_construction'; worlds(idx).type='urban';
    worlds(idx).wp_safe      = [0 0 0; 8 0 0; 8 8 90; -8 8 180; -8 -8 270; 8 -8 0; 0 0 0];
    worlds(idx).wp_challenge = [0 0 0; 10 0 0; 15 -2 0; 18 -3 0; 12 -5 180; 5 0 90; 0 0 0];
    worlds(idx).wp_speed     = [0 0 0; 15 0 0; -10 10 135; -10 -10 225; 15 -10 0; 0 0 0];
    worlds(idx).wp_precision = [0 0 0; 1 0 0; 1 1 90; 0 1 180; 0 0 270];
end

%% ─── SENARYO DETAYLARI ──────────────────────────────────────────────────
function [wp, par, key, desc] = getScenario(wd, idx)
    switch idx
        case 1  % Safe Patrol
            key = 'safe_patrol';
            desc = sprintf('%s — Güvenli devriye: engellerden uzak dairesel rota', wd.key);
            wp = wd.wp_safe;
            par = getParams(wd.type, 'safe');
        case 2  % Challenge Route
            key = 'challenge_route';
            desc = sprintf('%s — Zorlayıcı rota: dar geçitler, engel yakını', wd.key);
            wp = wd.wp_challenge;
            par = getParams(wd.type, 'challenge');
        case 3  % Speed Test
            key = 'speed_test';
            desc = sprintf('%s — Hız testi: uzun düz segmentler, maksimum hız', wd.key);
            wp = wd.wp_speed;
            par = getParams(wd.type, 'speed');
        case 4  % Precision Test
            key = 'precision_test';
            desc = sprintf('%s — Hassasiyet testi: sıkı tolerans, kısa mesafe', wd.key);
            wp = wd.wp_precision;
            par = getParams(wd.type, 'precision');
    end
end

%% ─── PARAMETRE PROFİLLERİ ───────────────────────────────────────────────
function p = getParams(worldType, scenario)
    % Dünya tipi × senaryo matrisinden parametre seç
    % Dünya tipi bazında baz değerler
    switch worldType
        case 'indoor'
            base = struct('max_vel',0.25,'max_acc',0.2,'braking',0.8,'safe_dist',0.85,'goal_tol',0.3);
        case 'urban'
            base = struct('max_vel',0.35,'max_acc',0.3,'braking',0.5,'safe_dist',0.85,'goal_tol',0.3);
        case 'outdoor_open'
            base = struct('max_vel',0.70,'max_acc',0.5,'braking',0.3,'safe_dist',0.85,'goal_tol',0.3);
        case 'terrain'
            base = struct('max_vel',0.30,'max_acc',0.2,'braking',0.6,'safe_dist',0.85,'goal_tol',0.5);
        case 'hybrid'
            base = struct('max_vel',0.40,'max_acc',0.3,'braking',0.5,'safe_dist',0.85,'goal_tol',0.3);
        case 'nature'
            base = struct('max_vel',0.45,'max_acc',0.3,'braking',0.4,'safe_dist',0.85,'goal_tol',0.3);
        otherwise
            base = struct('max_vel',0.40,'max_acc',0.3,'braking',0.5,'safe_dist',0.85,'goal_tol',0.3);
    end

    % Senaryo bazında modifikasyon
    switch scenario
        case 'safe'
            p = base;  % baz değerler zaten güvenli
            p.safe_dist = min(p.safe_dist * 1.5, 1.0);
        case 'challenge'
            p = base;
            p.max_vel = p.max_vel * 0.7;
            p.safe_dist = max(p.safe_dist * 0.9, 0.50);
            p.goal_tol = max(p.goal_tol * 0.8, 0.1);
        case 'speed'
            p = base;
            p.max_vel = min(p.max_vel * 1.8, 1.5);
            p.max_acc = min(p.max_acc * 1.5, 0.8);
            p.braking = max(p.braking * 0.6, 0.1);
        case 'precision'
            p = base;
            p.max_vel = max(p.max_vel * 0.4, 0.15);
            p.max_acc = max(p.max_acc * 0.5, 0.1);
            p.goal_tol = 0.1;
            p.safe_dist = min(p.safe_dist * 1.2, 0.8);
    end
end
