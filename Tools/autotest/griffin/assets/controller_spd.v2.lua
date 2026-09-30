------------------------------------------------------------------
---                      Parameters                            ---
------------------------------------------------------------------
local update_rate_srv=100 -- currently runnign at about 15ms
local up0=false
local tr0=false

--controller
local sim_time_total=30--31 --34
local sim_time_steps=121--187 --206
local frame_type="tilt";
    --type == tilt:quadtillt
    --type == pusher:quad+pusher/puller

local gain_td=sim_time_total/sim_time_total 
local sim_td=sim_time_total/sim_time_steps
local log_td=10

-- channel setup
local m1_chl=7-1
local m2_chl=5-1
local m3_chl=6-1
local m4_chl=8-1
local tilt_chl=13-1
local tilt_chl_2=12-1
local elv_chl=2-1


local quad_rpm_min=0
local quad_rpm_max=10000
local counter_3=0

local motor_overide_flag=false
local u_cmd_pwm ={nil,nil,nil,nil}
local u_cmd ={nil,nil,nil,nil}
local k_roll=-0.2
local expo=20*180/math.pi
local elevator_overide_flag=false
local elevator_abs_max=math.pi/4
local tilt_overide_flag=false
local tilt_max=math.pi/2

-- attitude loops around the LQR (start values for SITL, tune from LYAW/LROL)
-- effector authority vs tilt xi (0 = forward, pi/2 = vertical):
--   differential tilt        -> yaw ~ sin(xi), roll ~ cos(xi)
--   front differential thrust-> roll ~ sin(xi), yaw ~ cos(xi)
--   rear differential thrust -> roll always
-- (one table: ArduPilot Lua allows only 100 locals per function)
local update_rate=1
local a=0.2*update_rate+1
local AG = {
    Kpy_t = 0.3/a,  Kdy_t = 0.05/a,  -- yaw via differential tilt   [rad/rad], [rad/(rad/s)]
    Kpy_m = 300/a,  Kdy_m = 50/a, --50    -- yaw via front diff thrust   [us/rad],  [us/(rad/s)]
    Kpr_m = 600/a,  Kdr_m = 100/a, --100   -- roll via diff thrust        [us/rad],  [us/(rad/s)]
    Kpr_t = 0.2/a,  Kdr_t = 0.03/a,  -- roll via differential tilt  [rad/rad], [rad/(rad/s)]
    dtilt_max = 0.26,            -- rad (15 deg) per side
    dthr_max  = 200,             -- us per motor
    Kpl_m = 60/a,   Kdl_m = 120/a,   -- cross-track via roll diff thrust [us/m], [us/(m/s)] (SITL hover tuned, + xte -> roll left)
}

local u_elevator_prev
local u_tilt_prev
local t_0=0

local now=0
local prev=0
local update_interval_motors=30 --30 ish hz (not anymore)
local update_interval_elevator=30 --10hz (not anymore)
local update_interval_tilt=30 --10hz (not anymore)
local update_timestamp_motors=0
local update_timestamp_elevator=0
local update_timestamp_tilt=0

local cal_flag=0
local endflag=0
local max_motor_rpm=01
local scaler=1
local scaleflag=false

---debug arible1
local first_flag=false




------------------------------------------------------------------
---                      Setup                                 ---
------------------------------------------------------------------



--- CSV LOADER SETUP
local CSV_DIR = "scripts/"

local FILES = {
    { file = "K", global = "K" },
    { file = "U", global = "U_d" },
    { file = "Y", global = "Y_d" },
}

-- Pre-declare all globals as nil so other scripts see them immediately.
for _, entry in ipairs(FILES) do
    _ENV[entry.global] = nil
end

--- Trigger
local VTOL_LAND_CMD     = 85    -- MAV_CMD_NAV_VTOL_LAND
local TRANSITION_DIST_M = 400   -- metres (edit as needed)
local UPDATE_RATE_MS    = 100   -- 10 Hz is plenty; 1 ms floods the scheduler
local AUTO_MODE         = 10    -- ArduPlane / QuadPlane AUTO mode

start_transition = false        -- global flag, readable by other scripts
local _triggered     = false
local _last_debug_ms = 0        -- rate-limit GCS debug spam
local dist_m=1000

--- State observer
local FN_MOTOR1    = 33   -- SRV_Channel::k_motor1
local FN_MOTOR2    = 34
local FN_MOTOR3    = 35
local FN_MOTOR4    = 36
local FN_TILT      = 41   -- k_tiltMotor
local FN_ELEVATOR  = 19   -- k_elevator
local MAX_TILT_RAD = 1.5708   -- ~90 deg
-- local MAX_ELEV_RAD = 0.4363   -- ~25 deg
local MAX_ELEV_RAD = 0.475

local _obs_last_ms  = 0
local _obs_vtol_loc = nil
local _obs_appr_n   = nil   -- fixed approach axis, unit vector in NE, cached at trigger
local _obs_appr_e   = nil
local _obs_appr_hdg = nil   -- heading of the approach axis (rad), yaw reference for the approach
local _takeoff_alt  = nil   -- MSL altitude (m) at takeoff, cached from home


------------------------------------------------------------------
---                      Functions                             ---
------------------------------------------------------------------

-- state observer helpers
local function pwm_to_throttle_pct(pwm)
    if not pwm or pwm < 900 then return 0 end
    return math.max(0, math.min(100, (pwm - 1000) / 10))
end

local function pwm_to_rad(pwm, max_rad)
    if not pwm or pwm < 900 then return 0 end
    return (pwm - 1500) / 500 * max_rad
end

-- Returns y_now (6x1) and u_now (6x1) of the current aircraft state.
-- y_now: { lat_dist, height, pitch, lat_vel, height_rate, pitch_rate }
-- u_now: { m1, m2, m3, m4, tilt_rad, elev_rad }
local function get_current_state()
    local now_ms = tonumber(tostring(millis()))
    --gcs:send_text(1, string.format("now_ms = %f", now_ms))
    --gcs:send_text(1, string.format("last_ms = %f", _obs_last_ms))

    local dt = math.max((now_ms - _obs_last_ms) / 1000, 1e-6)
    _obs_last_ms = now_ms

    -- _obs_vtol_loc is cached by controller_trigger() at the moment it arms
    -- the transition, so it lines up with t_0 rather than lagging behind
    -- until the mission sequencer advances onto the VTOL_LAND item.

    -- Ground velocity. An EKF state, not a differenced measurement, so it is
    -- clean at the full loop rate. Used for both lat_vel and height_rate.
    local vel_ned = ahrs:get_velocity_NED()

    -- Height rate
    local height_rate = 0
    if vel_ned then height_rate = -vel_ned:z() end

    -- Along-track distance and velocity, projected onto the fixed approach axis.
    -- Do NOT difference get_distance(): EKF position updates at ~5 Hz while this
    -- loop runs at up to 1 kHz, so the quotient is either 0 or a 60 m/s spike
    -- (log 143: 16.6 m/s mean per-sample jump, range 0..67.8). Projecting the EKF
    -- velocity instead gives ~0.07 m/s. lat_dist is also signed this way, so it
    -- passes through zero rather than folding if the WP is overflown.
    local lat_dist, lat_vel = 0, 0
    local current_loc = ahrs:get_location()
    if current_loc and _obs_vtol_loc and _obs_appr_n then
        local to_wp = current_loc:get_distance_NE(_obs_vtol_loc)
        lat_dist = to_wp:x() * _obs_appr_n + to_wp:y() * _obs_appr_e
        if vel_ned then
            lat_vel = -(vel_ned:x() * _obs_appr_n + vel_ned:y() * _obs_appr_e)
        end
    end

    -- Height above takeoff point (home altitude cached once)
    local height = 0
    if current_loc then
        if not _takeoff_alt then
            local home = ahrs:get_home()
            if home then _takeoff_alt = home:alt() / 100 end
        end
        height = current_loc:alt() / 100 - (_takeoff_alt or 0)
    end

    -- Attitude
    local pitch      = ahrs:get_pitch_rad() or 0
    local gyro       = ahrs:get_gyro()
    local pitch_rate = gyro and gyro:y() or 0

    -- Motor throttle (%)
    local m1 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR1))
    local m2 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR2))
    local m3 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR3))
    local m4 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR4))

    -- Servo angles (rad)
    local tilt_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_TILT),     MAX_TILT_RAD)
    local elev_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_ELEVATOR), MAX_ELEV_RAD)

    local y_now = {
        lat_dist,    -- [1]  lateral distance to VTOL_LAND WP (m)
        height,      -- [2]  height above ground (m)
        pitch,       -- [3]  pitch angle (rad)
        lat_vel,     -- [4]  lateral velocity (m/s)
        height_rate, -- [5]  height rate (m/s)
        pitch_rate,  -- [6]  pitch rate (rad/s)
    }

    local u_now = {
        m1,       -- [1]  motor 1 throttle (%)
        m2,       -- [2]  motor 2 throttle (%)
        m3,       -- [3]  motor 3 throttle (%)
        m4,       -- [4]  motor 4 throttle (%)
        tilt_rad, -- [5]  tilt servo (rad)
        elev_rad, -- [6]  elevator (rad)
    }

    return y_now, u_now
end

--csv
local function parse_csv(filepath)
    local file = io.open(filepath, "r")
    if not file then
        gcs:send_text(3, "csv_loader: cannot open " .. filepath)
        return nil
    end

    local data = {}
    for line in file:lines() do
        if line ~= "" then
            local row = {}
            for field in line:gmatch("[^,]+") do
                local num = tonumber(field)
                if num then
                    row[#row + 1] = num
                end
            end
            if #row > 0 then
                data[#data + 1] = row
            end
        end
    end

    file:close()
    return data
end

local function csv_loader()
    gcs:send_text(6, "csv_loader: loading CSV files...")

    for _, entry in ipairs(FILES) do
        local path   = CSV_DIR .. entry.file .. ".csv"
        local result = parse_csv(path)
        if result then
            _ENV[entry.global] = result
            gcs:send_text(6, string.format(
                "csv_loader: %s -> %s (%d rows x %d cols)",
                entry.file, entry.global, #result, #result[1]
            ))
        else
            gcs:send_text(3, "csv_loader: " .. entry.file .. " failed to load")
        end
    end

    gcs:send_text(6, "csv_loader: all files processed")
end

-- trigger
local function controller_trigger()
    -- Only act in AUTO mode; reset state when leaving AUTO.
    if vehicle:get_mode() ~= AUTO_MODE then
        if _triggered then
            _triggered       = false
            start_transition = false
        end
        return update, UPDATE_RATE_MS
    end

    -- Index of the waypoint currently being navigated to.
    local current_idx = mission:get_current_nav_index()
    if not current_idx then
        return update, UPDATE_RATE_MS
    end

    -- Look one waypoint ahead of the one currently being navigated to.
    local next_idx = current_idx + 1

    -- Fetch that mission item and check its command ID.
    local nav_item = mission:get_item(next_idx)
    if not nav_item then
        return update, UPDATE_RATE_MS
    end

    if nav_item:command() ~= VTOL_LAND_CMD then
        -- Not approaching a VTOL_LAND; clear trigger so it can fire again.
        if _triggered then
            _triggered       = false
            start_transition = false
        end
        return update, UPDATE_RATE_MS
    end

    -- Flag already set for this approach.
    if _triggered then
        return update, UPDATE_RATE_MS
    end

    -- Current vehicle position.
    local current_loc = ahrs:get_location()
    if not current_loc then
        return update, UPDATE_RATE_MS
    end

    -- Build the VTOL_LAND location. Re-use ahrs:get_location() as a base
    -- object (avoids Location() constructor and :copy() compatibility issues)
    -- then overwrite lat/lng with the mission item coordinates.
    local vtol_loc = ahrs:get_location()
    if not vtol_loc then
        return update, UPDATE_RATE_MS
    end
    vtol_loc:lat(nav_item:x())   -- int32, degrees * 1e7
    vtol_loc:lng(nav_item:y())   -- int32, degrees * 1e7

    -- 2-D distance in metres.
    dist_m = current_loc:get_distance(vtol_loc)

    -- Rate-limited debug: print at most once per second.
    local now_ms = millis()
    if now_ms - _last_debug_ms >= 1000 then
        _last_debug_ms = now_ms
        gcs:send_text(7, string.format(
            "VTOL_LAND WP %d: dist=%.1f m  threshold=%d m",
            next_idx, dist_m, TRANSITION_DIST_M
        ))
    end

    -- Trigger once when the distance crosses the threshold.
    if dist_m <= TRANSITION_DIST_M then
        start_transition = true
        _triggered       = true
        _obs_vtol_loc      = vtol_loc
        -- Cache the approach axis: unit vector from here to the VTOL_LAND WP.
        -- The MATLAB model is planar longitudinal, so x must be measured along a
        -- FIXED ground track, not a line-of-sight that rotates as the WP closes.
        local to_wp = current_loc:get_distance_NE(vtol_loc)
        local appr_len = to_wp:length()
        if appr_len > 1e-3 then
            _obs_appr_n = to_wp:x() / appr_len
            _obs_appr_e = to_wp:y() / appr_len
            -- Yaw reference for the whole approach. vehicle:get_wp_bearing_deg()
            -- returns 0 (not nil) once VTOL_LAND enters position control
            -- (Plane::get_wp_bearing_deg, !using_wp_nav), so it cannot be used.
            _obs_appr_hdg = math.atan(_obs_appr_e, _obs_appr_n)
        end
        gcs:send_text(6, string.format(
            "start_transition SET at %.1f m from VTOL_LAND WP %d",
            dist_m, next_idx
        ))
        local now_tr=tonumber(tostring(millis()))
        gcs:send_text(1,string.format("transition time =  %f",now_tr))
    end
end

-- controller

local function controller_init ()
    t_0=tonumber(tostring(millis()))
    --gcs:send_text(1,string.format("t_0 = %f",t_0))
    tr0=true

end

-- matrix/vector subtraction: C = A - B  (must be same dimensions)
local function mat_sub(A, B)
    if #A ~= #B then
        gcs:send_text(3, string.format("mat_sub mismatch: #A=%d x %d #B=%d x %d", #A,#A, #B,#B))
        return nil
    end
    local C = {}
    for i = 1, #A do
        if type(A[i]) == "table" then
            C[i] = {}
            for j = 1, #A[i] do
                C[i][j] = A[i][j] - B[i][j]
            end
        else
            C[i] = A[i] - B[i]
        end
    end
    return C
end


-- matrix addition: C = A + B  (must be same dimensions)
local function mat_add(A, B)
    local C = {}
    for i = 1, #A do
        C[i] = {}
        for j = 1, #A[i] do
            C[i][j] = A[i][j] + B[i][j]
        end
    end
    return C
end

-- matrix/vector multiplication, handles all combinations of 1D vectors and 2D matrices
local function mat_mul(A, B)
    local a2d = type(A[1]) == "table"
    local b2d = type(B[1]) == "table"

    if a2d and b2d then
        -- (m×k) * (k×n) = (m×n)
        local m, k, n = #A, #B, #B[1]
        local C = {}
        for i = 1, m do
            C[i] = {}
            for j = 1, n do
                local s = 0
                for p = 1, k do s = s + A[i][p] * B[p][j] end
                C[i][j] = s
            end
        end
        return C
    elseif a2d and not b2d then
        -- (m×k) * (k) = (m)
        local m, k = #A, #B
        local C = {}
        for i = 1, m do
            local s = 0
            for p = 1, k do s = s + A[i][p] * B[p] end
            C[i] = s
        end
        return C
    elseif not a2d and b2d then
        -- (k) * (k×n) = (n)
        local k, n = #A, #B[1]
        local C = {}
        for j = 1, n do
            local s = 0
            for p = 1, k do s = s + A[p] * B[p][j] end
            C[j] = s
        end
        return C
    else
        -- (k) · (k) = scalar dot product
        local s = 0
        for p = 1, #A do s = s + A[p] * B[p] end
        return s
    end
end

-- matrix element wise multiplication C=A.*B
local function element_mul(A,B)
    local l=#B
    C={}
    for i =1 ,l do 
        C[i]=A*B[i]
    end
    return C
end

-- interpolates matlab arrays to current time
local function interpolate(func, now_p, dataset)
    local x1, x3
    local length
    local y
    if func == Y_d then
        length =6
        y={0,0,0,0,0,0}
    elseif func == U_d then
        length=4
        y={0,0,0,0}
    else
        gcs:send_text(1,"ERROR, no data set recognised")
    end
    
    if dataset == "sim" then
        x1 = math.floor((now_p / sim_td))+1
        x3 = math.ceil(now_p / sim_td)+1
    else
        gcs:send_text(3, "interpolate: unknown dataset '" .. tostring(dataset) .. "'")
        return nil
    end
    local r=-x1+((now_p)/(sim_td)+1)
    
    local i=1
    while i < length+1 do
        local v1=func[i][x1]
        local v3=func[i][x3]
        -- gcs:send_text(1,string.format("r = %f",r))
        -- gcs:send_text(1,string.format("v1 = %f",v1))
        -- gcs:send_text(1,string.format("v3 = %f",v3))
        y[i] = (1-r)*v1+(r)*v3
        i=i+1
    end
    return y
end

-- interpolates matrix array to current time (K stored as 4 rows per timestep)
local function interpolate_matrix(func, now_p, dataset, dimension)

    -- olds
    -- local x1, x3
    -- if dataset == "gain" then
    --     x1 = math.floor((now_p / gain_td)+1)
    --     x3 = math.ceil((now_p / gain_td)+1)
    -- else
    --     gcs:send_text(3, "interpolate_matrix: unknown dataset '" .. tostring(dataset) .. "'")
    --     return nil
    -- end

    -- x1=(x1-1)*6+1
    -- x3=(x3-1)*6+1

    -- local k1={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
    -- local k3={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
    -- local k2={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
    -- local i=1
   
    -- while i<=4 do
    --      local j=1
    --     while j<=6 do
    --         -- gcs:send_text(1,string.format("i=%f,j=%f",i,j))
    --         -- gcs:send_text(1,string.format("x1=%f,x3=%f",x1,x3))
    --         -- -- gcs:send_text(1,string.format("func[x3+i][j]=%f",func[x3+i][j]))
    --         -- gcs:send_text(1,string.format("func[x3+i][j]=%f",func[4][6]))
    --         -- gcs:send_text(1,string.format("k3[x1+i][j]=%f",k3[4][6]))
    --         k1[i][j]=func[i][x1+j-1]
    --         k3[i][j]=func[i][x3+j-1]
    --         j=j+1
    --     end
    --     j=1
    --     i=i+1
    -- end


    -- -- local r = x1-((now_p/gain_td)+1)
    -- local r = (now_p/gain_td + 1)-x1
    -- local i=1


    -- while i<=4 do
    --     local j=1
    --     while j<=6 do
    --         k2[i][j]=(1-r)*k1[i][j]+(r)*k3[i][j]
    --         j=j+1
    --     end
    --     i=i+1
    -- end

    -- return k2
    
    -- olds



    -- local num_steps = math.floor(#func / 4)
    -- x1 = math.max(1, math.min(x1, num_steps))
    -- local x2 = math.min(x1 + 1, num_steps)
    -- local w1 = x3 - math.floor(x3)
    -- local w2 = 1 - w1
    -- local b1 = 4 * (x1 - 1) + 1
    -- local b2 = 4 * (x2 - 1) + 1
    -- local matrix_1 = {func[b1], func[b1+1], func[b1+2], func[b1+3]}
    -- local matrix_2 = {func[b2], func[b2+1], func[b2+2], func[b2+3]}
    -- local y = {}
    -- for i = 4, 1 do
    --     local col = {}
    --     for j = 1, dimension do
    --         col[j] = w2 * matrix_1[i][j] + w1 * matrix_2[i][j]
    --     end
    --     y[i] = col
    -- end
    -- return y

    -- Node index is 1-based: node n holds columns (n-1)*6+1 .. n*6, and node 1 is t=0.
    -- w must come from the fractional part of the SCALED time, not from a difference
    -- between a time and an index - and computing it this way means there is no
    -- division at all, so a sample landing exactly on a node cannot produce 0/0.
    local tn = now_p / gain_td
    local n1 = math.floor(tn) + 1
    local n2 = n1 + 1                      -- never equal to n1
    local w  = tn - math.floor(tn)         -- 0..1

    local n_nodes = math.floor(#func[1] / 6)
    if n1 > n_nodes then n1 = n_nodes end
    if n2 > n_nodes then n2 = n_nodes end

    local y = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}

    local a = 1
    while a<5 do
        local b = 1
        while b<7 do
            local v1 = func[a][(n1-1)*6 + b]
            local v2 = func[a][(n2-1)*6 + b]
            y[a][b] = v1 + w*(v2 - v1)
            b=b+1
        end
        a=a+1
    end

    return y

end

--rearranges matlab arrays to correct dimensions for script
--decided to do this in matlab pre-export

--throttle expo (scalar)
local function Expo(thr_pct)
    gcs:send_text(1,string.format("thr_pct = %f",thr_pct))
    local thr_norm=thr_pct/100
    gcs:send_text(1,string.format("thr_norm = %f",thr_norm))
    local thr_expo=(thr_norm^3)*expo+thr_norm*(1-expo)
    gcs:send_text(1,string.format("thr_exp_norm = %f",thr_expo))
    local thr_expo_pwm=1000+thr_expo*1000
    gcs:send_text(1,string.format("thr_expo_pwm = %f",thr_expo_pwm))
    return thr_expo_pwm
end

local function pct_to_pwm (pct)
    local pwm=1000+math.floor(pct*10)
    --gcs:send_text(1,string.format("pct = %f, pwm = %f",pct,pwm))
    return pwm
end


--converts throttle values from rpm to %
local function rpm_to_pct (rpm,min,max)
    local pct=((rpm)/(max))*100
    if (rpm>max and rpm>max_motor_rpm) then
        max_motor_rpm=rpm
        scaleflag=true
        
    else 
        scaleflag=false
    end
    pct_clamp=math.min(100,(math.max(pct,0)))
    --gcs:send_text(1,string.format("rpm = %f,min = %f,max = %f, pct = %f",rpm,min,max,pct_clamp))
    return pct_clamp
end

-- calculates scaler for all throttles to ensure attitude stability
local function motor_clip_scale(max_motor_rpm)
    scaler = quad_rpm_max/max_motor_rpm
    return scaler
end

--converts throttle values from %(expo) to rpm 
-- VALUES MUST BE CHANGED FOR NEW EXPO VALUE REFER TO EXCEL SHEET
local function pct_to_rpm (thr_pct,min_rpm,max_rpm)
    local norm_e=thr_pct/100
    local norm=0.6826*(norm_e^3)-1.7606*(norm_e^2)+2.0796*norm_e+0.0011
    local rpm=norm*(max_rpm-min_rpm)
    return rpm
end

-- Signed cross-track error from the fixed approach line (trigger point -> VTOL_LAND WP).
-- Returns xte (m, + = aircraft RIGHT of track looking along the approach) and
-- xte_rate (m/s, + = drifting right). Both 0 until the approach axis is latched.
-- Right-hand normal of the axis (n, e) in NE is (-e, n).
local function get_cross_track()
    local loc = ahrs:get_location()
    if not (loc and _obs_vtol_loc and _obs_appr_n) then return 0, 0 end
    -- vehicle -> WP; vehicle offset from WP is its negative
    local to_wp = loc:get_distance_NE(_obs_vtol_loc)
    local xte = to_wp:x() * _obs_appr_e - to_wp:y() * _obs_appr_n
    local xte_rate = 0
    local vel = ahrs:get_velocity_NED()
    if vel then
        xte_rate = vel:y() * _obs_appr_n - vel:x() * _obs_appr_e
    end
    return xte, xte_rate
end


local function controller(u_cmd,y)

    -- gcs:send_text(1,"input command calculated")

        -- attitude errors shared by the tilt and motor blocks
        local xte, xter = get_cross_track()
        logger:write('LXTE','xte,xter','ff', xte, xter)

        local roll  = ahrs:get_roll_rad()
        local yaw   = ahrs:get_yaw_rad()
        local rates = ahrs:get_gyro()
        local rollr, yawr = rates:x(), rates:z()
        local yaw_desr = _obs_appr_hdg or yaw                       -- no latched heading -> zero error
        local yaw_err  = (yaw - yaw_desr + math.pi) % (2*math.pi) - math.pi -- wrap to [-pi,pi)
        local xi = math.min(math.pi/2, math.max(0, u_cmd[3]))      -- LQR tilt clamped to [0,pi/2]
        local sx, cx = math.sin(xi), math.cos(xi)                  -- sx ~1 vertical, cx ~1 forward
        local function clamp(v, lim) return math.max(-lim, math.min(lim, v)) end

        -- each effector weighted by its authority at this tilt: no divisions,
        -- cross-coupling vanishes at both ends of the tilt range
        local dtilt_y = clamp((AG.Kpy_t*yaw_err + AG.Kdy_t*yawr) * sx, AG.dtilt_max) -- rad
        local dtilt_r = clamp((AG.Kpr_t*roll    + AG.Kdr_t*rollr) * cx, AG.dtilt_max) -- rad
        local dthr_y  = clamp((AG.Kpy_m*yaw_err + AG.Kdy_m*yawr) * cx, AG.dthr_max)   -- us, front
        local dthr_rf = clamp((AG.Kpr_m*roll    + AG.Kdr_m*rollr + AG.Kpl_m*xte + AG.Kdl_m*xter) * sx, AG.dthr_max)  -- us, front
        local dthr_rr = clamp((AG.Kpr_m*roll    + AG.Kdr_m*rollr + AG.Kpl_m*xte + AG.Kdl_m*xter),      AG.dthr_max)  -- us, rear

        --tilt
        if (frame_type=="tilt") then
            
            --stop overide if tilt command not updated
            if (now-update_timestamp_tilt>update_interval_tilt) then
                tilt_overide_flag=false
            end

            --calculate new tilt command
            local tilt_pwm
            if (now-update_timestamp_tilt>update_interval_tilt or tilt_overide_flag==false) then
                --update current value
                
                
                --update pwm value
                tilt_overide_flag=true
                
                -- local u_cmd_tilt_norm=(u_cmd[3])/tilt_max
                -- tilt_pwm=1500+u_cmd_tilt_norm*500

                -- differential tilt: {left, right}. +yaw_err = nose right of track ->
                -- left rotor LESS forward (larger xi), right MORE forward, matching
                -- ArduPilot's vectored yaw sign (tiltrotor.cpp). +roll = right wing
                -- down -> right rotor more vertical (larger xi) lifts the right side.
                t_cmd={xi + dtilt_y - dtilt_r, xi - dtilt_y + dtilt_r}

                -- TODO change the scaling factor to one that uses the params
                t_cmd_pwm={2000-((1716/math.pi)*(t_cmd[1])),2000-((1716/math.pi)*(t_cmd[2]))}
                t_cmd_pwm[1]=math.min(2000,math.max(1000,t_cmd_pwm[1]))
                t_cmd_pwm[2]=math.min(2000,math.max(1000,t_cmd_pwm[2]))

                tilt_pwm=2000-((1716/math.pi)*xi)
                tilt_pwm=math.max(1000,tilt_pwm)
                tilt_pwm=math.min(2000,tilt_pwm)
                logger:write('LYAW','yerr,ydes,yawr,dtilt,dthr','fffff',
                yaw_err,yaw_desr,yawr,dtilt_y,dthr_y)
            
            end

            --overide tilt command
            if (tilt_overide_flag) then
                -- SRV_Channels:set_output_pwm_chan_timeout(tilt_chl, math.floor(tilt_pwm),update_rate_srv)
                -- SRV_Channels:set_output_pwm_chan_timeout(tilt_chl_2, math.floor(tilt_pwm),update_rate_srv)

                SRV_Channels:set_output_pwm_chan_timeout(tilt_chl, math.floor(t_cmd_pwm[1]),update_rate_srv)
                SRV_Channels:set_output_pwm_chan_timeout(tilt_chl_2, math.floor(t_cmd_pwm[2]),update_rate_srv)
            end
        end

        --motor
        --stop motor overide if command is not updated
        if (now-update_timestamp_motors>update_interval_motors) then
            motor_overide_flag=false
        end

        --motor command calculate
        if (now-update_timestamp_motors>update_interval_motors or motor_overide_flag==false) then
            --cmd rpm --> pct --> norm--> expo --> pwm 
            motor_overide_flag=true
            max_motor_rpm=2200;
            --update current motor values
            -- u1_prev=pct_to_rpm(y[7],quad_rpm_min,quad_rpm_max)
            -- u2_prev=pct_to_rpm(y[8],quad_rpm_min,quad_rpm_max)
            -- u3_prev=pct_to_rpm(y[9],quad_rpm_min,quad_rpm_max)
            -- u4_prev=pct_to_rpm(y[10],quad_rpm_min,quad_rpm_max)
            -- if (frame_type=="pusher") then
            --     u5_prev=pct_to_rpm(y[11],quad_rpm_min,quad_rpm_max)
            -- end

  
            local roll =ahrs:get_roll_rad() --rads
            local roll_compensation_rpm=k_roll*roll

            --update % values
            local K_m=1
            local u_cmd_pct={nil,nil,nil,nil}
            -- u_cmd_pct[1] = rpm_to_pct((K_m*u_cmd[1]*(update_interval_motors/1000))+u1_prev-roll_compensation_rpm*math.cos(u_tilt_prev),quad_rpm_min,quad_rpm_max)
            -- u_cmd_pct[2] = rpm_to_pct((K_m*u_cmd[1]*(update_interval_motors/1000))+u2_prev+roll_compensation_rpm*math.cos(u_tilt_prev),quad_rpm_min,quad_rpm_max)
            -- u_cmd_pct[3] = rpm_to_pct((K_m*u_cmd[2]*(update_interval_motors/1000))+u3_prev-roll_compensation_rpm,quad_rpm_min,quad_rpm_max)
            -- u_cmd_pct[4] = rpm_to_pct((K_m*u_cmd[2]*(update_interval_motors/1000))+u4_prev+roll_compensation_rpm,quad_rpm_min,quad_rpm_max)
            -- gcs:send_text(1,string.format("u_cmd_pct 1 = %f , u1_prev = %f, dt = %f",u_cmd_pct[1],u1_prev,update_interval_motors/1000))
            
            
            -- local prv_rpm_f,prv_rpm_r=rpmf,rpmr
            -- local rpmf = 0.5*(rpm:get_rpm(0)+rpm:get_rpm(1))
            -- local rpmr = 0.5*(rpm:get_rpm(3)+rpm:get_rpm(2))
            -- local t_step=now-prev
            -- local rpmf_dot=(rpmf-prv_rpm_f)/(t_step)




            -- rpm = rpm:get_rpm(0)
            
            u_cmd_pct[1] = rpm_to_pct(u_cmd[1],quad_rpm_min,quad_rpm_max)
            u_cmd_pct[2] = rpm_to_pct(u_cmd[1],quad_rpm_min,quad_rpm_max)
            u_cmd_pct[3] = rpm_to_pct(u_cmd[2],quad_rpm_min,quad_rpm_max)
            u_cmd_pct[4] = rpm_to_pct(u_cmd[2],quad_rpm_min,quad_rpm_max)

            logger:write('CMDL', 'R1,R2,R3,R4,M1pt,M2pt,M3pt,M4pt', 'ffffffff',
            u_cmd[1], u_cmd[2], u_cmd[3], u_cmd[4],u_cmd_pct[1], u_cmd_pct[2], u_cmd_pct[3], u_cmd_pct[4])

           
        

            --clipping scaler
            if  (scaleflag==true) then
                motor_clip_scale(max_motor_rpm)
            else 
                scaler=1
            end

            --update pwm values
            -- gcs:send_text(1,string.format("u_cmd_pct = %f, %f, %f, %f",u_cmd_pct[1],u_cmd_pct[2],u_cmd_pct[3],u_cmd_pct[4]))
            -- gcs:send_text(1,string.format("scaler = %f",scaler))
            u_cmd_pwm[1]=pct_to_pwm(scaler*u_cmd_pct[1])
            u_cmd_pwm[2]=pct_to_pwm(scaler*u_cmd_pct[2])
            u_cmd_pwm[3]=pct_to_pwm(scaler*u_cmd_pct[3])
            u_cmd_pwm[4]=pct_to_pwm(scaler*u_cmd_pct[4])

        end

        --motor overide
        if (motor_overide_flag) then
            -- overide output 
            --gcs:send_text(1,string.format("m1 pwm = %f",u_cmd_pwm[1]))
            -- SRV_Channels:set_output_pwm_chan_timeout(m1_chl, math.floor(u_cmd_pwm[1]),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m2_chl, math.floor(u_cmd_pwm[2]),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m3_chl, math.floor(u_cmd_pwm[3]),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m4_chl, math.floor(u_cmd_pwm[4]),update_rate_srv)
            
            -- roll / yaw via differential thrust: {front-left, front-right, rear-left, rear-right}
            -- (m1_chl=SERVO7=motor3 FL, m2_chl=SERVO5=motor1 FR, m3_chl=SERVO6=motor2 RL,
            --  m4_chl=SERVO8=motor4 RR for Q_FRAME_TYPE=1). +roll -> more thrust on the
            -- right; +yaw_err -> less forward thrust on the left.
            local m_cmd_pwm={0,0,0,0}
            local base_f = math.floor(1000+1000*u_cmd[1]/10000)
            local base_r = math.floor(1000+1000*u_cmd[2]/10000)

            m_cmd_pwm[1] = base_f - dthr_y - dthr_rf
            m_cmd_pwm[2] = base_f + dthr_y + dthr_rf
            m_cmd_pwm[3] = base_r - dthr_rr
            m_cmd_pwm[4] = base_r + dthr_rr

            logger:write('LROL','roll,rollr,dtiltr,dthrf,dthrr','fffff',
            roll,rollr,dtilt_r,dthr_rf,dthr_rr)


            SRV_Channels:set_output_pwm_chan_timeout(m1_chl, math.min(2000,math.max(1000,math.floor(m_cmd_pwm[1]))),update_rate_srv)
            SRV_Channels:set_output_pwm_chan_timeout(m2_chl, math.min(2000,math.max(1000,math.floor(m_cmd_pwm[2]))),update_rate_srv)
            SRV_Channels:set_output_pwm_chan_timeout(m3_chl, math.min(2000,math.max(1000,math.floor(m_cmd_pwm[3]))),update_rate_srv)
            SRV_Channels:set_output_pwm_chan_timeout(m4_chl, math.min(2000,math.max(1000,math.floor(m_cmd_pwm[4]))),update_rate_srv)

            -- previous working implementation
            -- SRV_Channels:set_output_pwm_chan_timeout(m1_chl, math.min(2000,math.max(0,math.floor(1000+1000*(u_cmd[1]/10000)))),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m2_chl, math.min(2000,math.max(0,math.floor(1000+1000*(u_cmd[1]/10000)))),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m3_chl, math.min(2000,math.max(0,math.floor(1000+1000*(u_cmd[2]/10000)))),update_rate_srv)
            -- SRV_Channels:set_output_pwm_chan_timeout(m4_chl, math.min(2000,math.max(0,math.floor(1000+1000*(u_cmd[2]/10000)))),update_rate_srv)


            logger:write('L2', 'f,r', 'ff',
            math.floor(1000+1000*(u_cmd[1]/10000)), math.floor(1000+1000*(u_cmd[2]/10000)))

            -- min(2000,max(0,math.floor(1000+1000*(u_cmd[1]/10000))))
        end

        -- SRV_Channels:set_output_pwm_chan_timeout(m1_chl,1000,update_rate_srv)
        -- SRV_Channels:set_output_pwm_chan_timeout(m2_chl,1000,update_rate_srv)
        -- SRV_Channels:set_output_pwm_chan_timeout(m3_chl,1000,update_rate_srv)
        -- SRV_Channels:set_output_pwm_chan_timeout(m4_chl,1000 ,update_rate_srv)
       

        --elevator
        local elevator_pwm
        
        -- stop overiding elevator if too long without update
        if (now-update_timestamp_elevator>update_interval_elevator) then
            elevator_overide_flag=false
        end

        -- calculate new elevator value
        if (now-update_timestamp_elevator>update_interval_elevator or elevator_overide_flag==false) then
            -- -- update current elevator value
            -- local elev_pct=((u_cmd[4])/(math.pi/2))*(100)

            -- --calculate pwm value
            elevator_overide_flag=true
            -- local elevator_rate_clamp = math.min(50,math.max(-1*(50),elev_pct))
            -- elevator_pwm=math.floor(((elevator_rate_clamp)*10)+1500)

            elevator_pwm = math.floor(1500-((2000/(math.pi))*u_cmd[4]))
            -- elevator_pwm = math.floor(1760-((2000/(math.pi))*u_cmd[4]))
            elevator_pwm = math.max(elevator_pwm,1000)
            elevator_pwm = math.min(elevator_pwm,2000)

        end

        -- overide elevator
        if (elevator_overide_flag) then
            SRV_Channels:set_output_pwm_chan_timeout(elv_chl, elevator_pwm,update_rate_srv)--2000 max pitch dowm 1000 max pitch up
            -- gcs:send_text(1,string.format("elev servo cmd = %f",elevator_pwm))
            -- SRV_Channels:set_output_pwm_chan_timeout(elv_chl, 1000,update_rate_srv)--2000 max pitch dowm 1000 max pitch up
        end


        

end

local function calculate_cmd(y,u)
    prev=now
    now=tonumber(tostring(millis()))
    
    --gcs:send_text(1,string.format("now = %f, t_0 = %f",now,t_0))
    if (now-t_0>10) then
        cal_flag=1
        --gcs:send_text(1,"contrller function started")

            --convert imported data dimensions
            
            y_dim={-y[1],-y[2],y[3],-y[4],-y[5],y[6]}
            --gcs:send_text(1,string.format("y_dim 4 = %f",y_dim[4]))
        
        --gcs:send_text(1,"y dimension changed")


        --interpolation of inputs
        --gcs:send_text(1,string.format("y index = %f, t-t_0 = %f",((now-t_0)/1000)/sim_td,now-t_0))
        if ((((now-t_0)/1000))>=sim_time_total-0.3) then
            endflag=1
            gcs:send_text(1,"Landing Approach Completed, Handing off to Ardupilot")
        end
        local y_d=interpolate(Y_d,((now-t_0)/1000),"sim")
        local u_d=interpolate(U_d,((now-t_0)/1000),"sim")
        local k=interpolate_matrix(K,((now-t_0)/1000),"gain",10)
        -- local k_i = math.floor(((now-t_0)/1000)/gain_td)
        -- local k ={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}} 

        -- local a=1
        -- while a<5 do
        --     local b=1
        --     while b<7 do
        --         k[a][b]=K[a][(6*k_i)+b]
        --         b=b+1
        --     end
        --     a=a+1
        -- end
        logger:write('GAN1','k11,k12,k13,k14,k15,k16,k21,k22,k23,k24,k25,k26','ffffffffffff',
        k[1][1],k[1][2],k[1][3],k[1][4],k[1][5],k[1][6],k[2][1],k[2][2],k[2][3],k[2][4],k[2][5],k[2][6])

        logger:write('GAN2','k31,k32,k33,k34,k35,k36,k41,k42,k43,k44,k45,k46','ffffffffffff',
        k[3][1],k[3][2],k[3][3],k[3][4],k[3][5],k[3][6],k[4][1],k[4][2],k[4][3],k[4][4],k[4][5],k[4][6])


        local y_d_fixxed={y_d[1],y_d[2],y_d[3],y_d[4],y_d[5],y_d[6]}
        --gcs:send_text(1,string.format("y_ned' 4 = %f",y_d_not_NED[4]))
        --gcs:send_text(1,string.format("y_d_not_NED= %f",y_d_not_NED))


        --gcs:send_text(1,"interpolation done")
        

        --pct/s to rpm/s
        local u1_rpm=pct_to_rpm(u[1],0,10000)
        local u2_rpm=pct_to_rpm(u[2],0,10000)
        local u3_rpm=pct_to_rpm(u[3],0,10000)
        local u4_rpm=pct_to_rpm(u[4],0,10000)

        local u_rpm={(u1_rpm+u2_rpm)/2,(u3_rpm+u4_rpm)/2,u[5],u[6]}

        local y_bar={0,0,0,0,0,0}
        local u_bar={0,0,0,0}
        local z=1
        while z<7 do
            y_bar[z]=y_dim[z]-y_d_fixxed[z]
            -- if (z<5) then
            --     u_bar[z]=u_rpm[z]-u_d[z]
            -- end
            z=z+1
        end

        --u_cmd=u_d-K*Y
        local Ky_bar={0,0,0,0,0,0}
        local line=0
        local i=1
        while i<5 do
            local j=1
            while j<7 do
                -- gcs:send_text(1,string.format("y_bar[1] = %f", y_bar[1]))
                -- gcs:send_text(1,string.format("k[1][1] = %f", k[1][1]))
                line=line+k[i][j]*y_bar[j]
                j=j+1
            end
            Ky_bar[i]=line
            line=0
            i=i+1
        end
        
        local i=1
        while i<5 do
            u_cmd[i]=u_d[i]-Ky_bar[i];
            i=i+1
        end
        -- u_cmd[1]=u_d[1]
        -- u_cmd[2]=u_d[2]
        -- u_cmd[3]=u_d[3]
        -- u_cmd[4]=u_d[4]



        logger:write('DESR','y1,y2,y3,y4,y5,y6','ffffff',
        y_d_fixxed[1],y_d_fixxed[2],y_d_fixxed[3],y_d_fixxed[4],y_d_fixxed[5],y_d_fixxed[6])

        logger:write('FBL','y1,y2,y3,y4,y5,y6','ffffff',
        y_dim[1],y_dim[2],y_dim[3],y_dim[4],y_dim[5],y_dim[6])

        -- drag visulisation
        -- local l={{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
        -- local d={0,0,0,0}
        -- local d2={{0,0},{0,0},{0,0},{0,0}}
        -- local a=1
        -- local vr={0,0,0,0}
        -- local mRbr={{math.cos(-y_d_fixxed[3]),-math.sin(-y_d_fixxed[3])},{math.sin(-y_d_fixxed[3]),math.cos(-y_d_fixxed[3])}}
        -- local mRbf={{math.cos(-1*(y_d_fixxed[3]+u_d[3]+1.5707)),-math.sin(-1*(y_d_fixxed[3]+u_d[3]+1.5707))},{math.sin(-1*(y_d_fixxed[3]+u_d[3]+1.5707)),math.cos(-1*(y_d_fixxed[3]+u_d[3]+1.5707))}}
        -- while a<5 do
        --     if a<3 then
        --         l[a][3]=0.5*2*1.2682*0.025*0.175*0.175*0.175*0.00668*u_d[1]*u_d[1]
        --         vr[a]=y_d_fixxed[4]*math.cos(-1*(y_d_fixxed[3]+u_d[3]+1.5707))+y_d_fixxed[5]*math.sin(-1*(y_d_fixxed[3]+u_d[3]+1.5707))
        --     else 
        --         l[a][3]=0.5*2*1.2682*0.025*0.175*0.175*0.175*0.00668*u_d[2]*u_d[2]
        --         vr[a]=y_d_fixxed[4]*math.cos(-y_d_fixxed[3])+y_d_fixxed[5]*math.sin(-y_d_fixxed[3])
        --     end
            
        --     if a<3 then
        --         d[a]=l[a][3]*vr[a]/(u_d[1]*0.175)
        --     else 
        --         d[a]=l[a][3]*vr[a]/(u_d[2]*0.175)
        --     end
        --     local b=1
        --     while b<3 do
        --     if a<3 then
        --         if b==1 then
        --             d2[a][b]=d[a]*math.cos(-1*(y_d_fixxed[3]+u_d[3]+1.5707))
        --         else
        --             d2[a][b]=d[a]*math.sin(-1*(y_d_fixxed[3]+u_d[3]+1.5707))
        --         end
        --     else
        --         if b==1 then
        --             d2[a][b]=d[a]*math.cos(-1*(y_d_fixxed[3]))
        --         else
        --             d2[a][b]=d[a]*math.sin(-1*(y_d_fixxed[3]))
        --         end
        --     end

        --     b=b+1
        --     end
            
        --     a=a+1
        -- end
        -- logger:write('MTML','m1dx,m1dz,m2dx,m2dz,m3dx,m3dz,m4dx,m4dz','ffffffff',
        -- d2[1][1],d2[1][2],d2[2][1],d2[2][2],d2[3][1],d2[3][2],d2[4][1],d2[4][2])

        logger:write('ARRS', 'u1_d,u2_d,u3_d,u4_d,u1_cmd,u2_cmd,u3_cmd,u4_cmd', 'ffffffff',
        u_d[1], u_d[2], u_d[3], u_d[4],u_cmd[1], u_cmd[2], u_cmd[3], u_cmd[4])

       
        return u_cmd
    end
end

------------------------------------------------------------------
---                      MAIN                                  ---
------------------------------------------------------------------



local function init ()
    csv_loader()
    up0=true
end


local function update()
    --debugging
    -- SRV_Channels:set_output_pwm_chan_timeout(m1_chl,1010,update_rate_srv)
    -- SRV_Channels:set_output_pwm_chan_timeout(m2_chl,1010,update_rate_srv)
    -- SRV_Channels:set_output_pwm_chan_timeout(m3_chl,1010,update_rate_srv)
    -- SRV_Channels:set_output_pwm_chan_timeout(m4_chl,1010 ,update_rate_srv)


    if (up0==false) then
        init()
    end
    -- if (up0==true) then
    --     gcs:send_text(1, "init ran")
    -- end

    if (_triggered==false) then
        controller_trigger()
    end
    if (_triggered==false and dist_m<320 and dist_m > 295) then
        local y, u = get_current_state()
    end
    
    --SRV_Channels:set_output_pwm_chan_timeout(elv_chl, 2000,update_rate_srv)

    

    -- 2. Register the custom log format in the initialization phase

    if (_triggered==true and endflag==0) then
        if (tr0==false) then
            controller_init()
        end
        local y, u = get_current_state()
        
        local inputs=calculate_cmd(y,u)

        if (cal_flag==1) then
            controller(inputs,y)
        end

        local count=0
        if (now-t_0>=(count+1)*2000) then
            --gcs:send_text(1,"controller override enagaged")
            count=count+1
        end
    end
   

    

    return update,update_rate
end

return update()

--93932 t_tr 397