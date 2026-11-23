-- SPDX-License-Identifier: GPL-3.0-or-later
-- Golden reference for the native exterior surface driver (native/aileron_datarefs).
--
-- Runs the REAL FlyWithLua drivers and modules, unmodified:
--   Scripts/A380X_XP12_Ailerons.lua + Modules/a380x_aileron_dance.lua
--   Scripts/A380X_XP12_Hydraulics.lua + Modules/a380x_hydraulic_surfaces.lua
-- under a FlyWithLua host model (one shared Lua environment, scripts loaded alphabetically,
-- do_every_frame callbacks executed in registration order, XPLMFindDataRef/XPLMGetDataRefTypes/
-- dataref_table as FlyWithLua NG exposes them) over scripted simulator inputs, and writes every
-- frame's inputs and the 22 published animation dataref values to a text file. The C test
-- (native/aileron_datarefs/test_surface_golden.c) feeds the same inputs to the real plugin through a
-- mock XPLM and compares its 22 published values frame by frame.
--
-- Dataref storage is modelled exactly: simulator floats are float32 (string.pack "f"), the 22 custom
-- refs store float32 after the plugin's clamp-and-keep-finite write.
--
--   lua tests/surface_golden_lua.lua <FlyWithLua dir> <output file>
local root = assert(arg[1], "pass the FlyWithLua directory (Scripts/, Modules/)")
local output = assert(arg[2], "pass the output file")
assert(string.pack, "Lua 5.3+ is required (string.pack)")

local function f32(x)
    return (string.unpack("<f", string.pack("<f", x)))
end
local function finite(x) return x == x and x > -math.huge and x < math.huge end

-- The 22 aircraft-plugin refs, in the plugin's order, with their clamp ranges.
local CUSTOM = {}
for _, side in ipairs({"l", "r"}) do
    for _, panel in ipairs({"inboard", "midboard", "outboard"}) do
        CUSTOM[#CUSTOM + 1] = {"a380x/xp12/visual/aileron_" .. side .. "_" .. panel .. "_ratio", -1, 1}
    end
end
for _, side in ipairs({"l", "r"}) do
    for _, panel in ipairs({"inboard", "midboard", "outboard"}) do
        CUSTOM[#CUSTOM + 1] = {"a380x/xp12/hydraulics/aileron_" .. side .. "_" .. panel .. "_ratio", -1, 1}
    end
end
CUSTOM[#CUSTOM + 1] = {"a380x/xp12/hydraulics/elevator_ratio", -1, 1}
CUSTOM[#CUSTOM + 1] = {"a380x/xp12/hydraulics/rudder_ratio", -1, 1}
for _, panel in ipairs({"inboard", "inner", "middle", "outer"}) do
    for _, side in ipairs({"l", "r"}) do
        CUSTOM[#CUSTOM + 1] = {"a380x/xp12/hydraulics/spoiler_" .. side .. "_" .. panel .. "_deg", 0, 65}
    end
end
assert(#CUSTOM == 22)

-- Simulator inputs (order = the golden file's input columns).
local SIM = {
    {"roll", "sim/flightmodel2/controls/roll_ratio", 2},
    {"pitch", "sim/flightmodel2/controls/pitch_ratio", 2},
    {"yaw", "sim/flightmodel2/controls/heading_ratio", 2},
    {"grounded", "sim/flightmodel/failures/onground_any", 1},
    {"paused", "sim/time/paused", 1},
    {"dt", "sim/operation/misc/frame_rate_period", 2},
    {"heading", "sim/flightmodel/position/psi", 2},
    {"wind_dir", "sim/weather/aircraft/wind_now_direction_degt", 2},
    {"wind", "sim/weather/aircraft/wind_now_speed_msc", 2},
    {"airspeed", "sim/flightmodel/position/true_airspeed", 2},
    {"p1", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1", 2},
    {"p2", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2", 2},
    {"p3", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3", 2},
}
local ARRAYS = {
    {"sim/flightmodel2/wing/speedbrake1_deg", {"sb0", "sb1"}, 0},
    {"sim/flightmodel2/wing/spoiler1_deg", {"sp0", "sp1", "sp2", "sp3"}, 0},
    {"sim/flightmodel2/wing/spoiler2_deg", {"sq4", "sq5"}, 4},
}
local COLUMNS = {}
for _, row in ipairs(SIM) do COLUMNS[#COLUMNS + 1] = row[1] end
for _, row in ipairs(ARRAYS) do for _, key in ipairs(row[2]) do COLUMNS[#COLUMNS + 1] = key end end
assert(#COLUMNS == 21)

-- Shared X-Plane dataref space: custom refs survive script reloads (the plugin keeps them).
local custom_value, custom_range = {}, {}
for _, row in ipairs(CUSTOM) do custom_value[row[1]], custom_range[row[1]] = 0, {row[2], row[3]} end
local sim_scalar, sim_array, sim_type = {}, {}, {}
for _, row in ipairs(SIM) do sim_type[row[2]] = row[3] end
for _, row in ipairs(ARRAYS) do sim_type[row[1]] = 8; sim_array[row[1]] = {} end

local function set_inputs(frame)
    for _, row in ipairs(SIM) do
        local v = frame[row[1]]
        sim_scalar[row[2]] = row[3] == 1 and v or f32(v)
    end
    for _, row in ipairs(ARRAYS) do
        for k, key in ipairs(row[2]) do sim_array[row[1]][row[3] + k - 1] = f32(frame[key]) end
    end
end

local messages = {}
local function new_host()
    local env = setmetatable({}, {__index = _G})
    env._G = env
    env.PLANE_ICAO, env.AIRCRAFT_PATH = "A388", "/X-Plane 12/Aircraft/A380X-XP12/"
    env.logMsg = function(text) messages[#messages + 1] = text end
    local handles, names = {}, {}
    local function handle(name)
        if not handles[name] then
            local h = setmetatable({}, {__name = "XPLMDataRef"})
            handles[name], names[h] = h, name
        end
        return handles[name]
    end
    env.XPLMFindDataRef = function(name)
        if custom_value[name] ~= nil or sim_type[name] ~= nil then return handle(name) end
        return nil
    end
    env.XPLMGetDataRefTypes = function(h)
        local name = assert(names[h], "unknown handle")
        return custom_value[name] ~= nil and 2 or sim_type[name]
    end
    env.dataref_table = function(name)
        assert(custom_value[name] ~= nil or sim_type[name] ~= nil, "unknown dataref " .. tostring(name))
        return setmetatable({}, {
            __index = function(_, index)
                if custom_value[name] ~= nil then assert(index == 0); return custom_value[name] end
                if sim_type[name] == 8 then return sim_array[name][index] end
                assert(index == 0); return sim_scalar[name]
            end,
            __newindex = function(_, index, value)
                assert(custom_value[name] ~= nil and index == 0, "write to simulator dataref " .. name)
                -- XPLMSetDataf(float) then the plugin's write accessor: finite -> clamp, else keep.
                local v = f32(value)
                if finite(v) then
                    local range = custom_range[name]
                    custom_value[name] = v < range[1] and range[1] or v > range[2] and range[2] or v
                end
            end,
        })
    end
    local frames = {}
    env.do_every_frame = function(code) frames[#frames + 1] = assert(load(code, "frame", "t", env)) end
    env.do_on_exit = function() end
    env.require = function(module)
        return assert(loadfile(root .. "/Modules/" .. module .. ".lua", "t", env))()
    end
    for _, script in ipairs({"A380X_XP12_Ailerons.lua", "A380X_XP12_Hydraulics.lua"}) do
        assert(loadfile(root .. "/Scripts/" .. script, "t", env))()
    end
    return function() for _, chunk in ipairs(frames) do chunk() end end
end

-- ---------------------------------------------------------------- scenarios
local function base(t)
    return {roll = 0, pitch = 0, yaw = 0, grounded = 1, paused = 0, dt = 1 / 60, heading = 90,
            wind_dir = 270, wind = 0, airspeed = 0, p1 = 3000, p2 = 3000, p3 = 0,
            sb0 = 0, sb1 = 0, sp0 = 0, sp1 = 0, sp2 = 0, sp3 = 0, sq4 = 0, sq5 = 0, t = t}
end
local function seconds(duration, dt) return math.floor(duration / dt + 0.5) end
local function sweep(t, period) return math.sin(2 * math.pi * t / period) end
local function lcg(seed)
    local state = seed
    return function()
        state = (state * 1103515245 + 12345) % 2147483648
        return state / 2147483648
    end
end

local scenarios = {}
local function scenario(name, build) scenarios[#scenarios + 1] = {name = name, frames = build()} end

scenario("ground powered full deflections", function()
    local out, t, dt = {}, 0, 1 / 60
    for _ = 1, seconds(12, dt) do
        local f = base(t)
        f.roll, f.pitch, f.yaw = sweep(t, 3) * 1.2, sweep(t, 4), sweep(t, 5) * -1.1
        if t > 6 then f.roll, f.pitch, f.yaw = (t % 2 < 1) and 1 or -1, (t % 3 < 1.5) and -1 or 1, 1 end
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

scenario("airborne roll doublets and spoilers", function()
    local out, t, dt = {}, 0, 1 / 50
    for _ = 1, seconds(15, dt) do
        local f = base(t)
        f.grounded, f.airspeed, f.p3 = 0, 140, 3000
        f.roll = (t % 4 < 1) and 0.8 or (t % 4 < 2) and -0.8 or 0.1 * sweep(t, 1.3)
        f.pitch = 0.3 * sweep(t, 2.2)
        local s = 40 * math.max(0, sweep(t, 6))
        f.sb0, f.sb1, f.sp0, f.sp1 = s, s * 0.9, s * 0.5, s * 0.4
        f.sp2, f.sp3, f.sq4, f.sq5 = math.max(0, 30 * f.roll), math.max(0, -30 * f.roll), s, 70
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

scenario("pressure loss on ground then recovery with hysteresis", function()
    local out, t, dt = {}, 0, 1 / 60
    for _ = 1, seconds(40, dt) do
        local f = base(t)
        f.roll, f.pitch, f.yaw = 0.6, -0.5, 0.4
        f.sb0, f.sb1, f.sp0 = 50, 50, 25
        local p = 3000
        if t > 2 then p = math.max(0, 3000 - (t - 2) * 600) end          -- bleeds through 900 / 450
        if t > 20 then p = math.min(3000, (t - 20) * 120) end            -- recovers through 450 / 900
        if t > 26 and t < 27 then p = 700 end                            -- inside the hysteresis band
        f.p1, f.p2, f.p3 = p, p * 0.8, 0
        f.wind, f.wind_dir = 6, 0
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

scenario("cold aircraft on ground, wind gusts and heading changes", function()
    local out, t, dt, r = {}, 0, 1 / 40, lcg(7)
    for _ = 1, seconds(30, dt) do
        local f = base(t)
        f.p1, f.p2, f.p3 = 0, 0, 0
        f.roll, f.pitch, f.yaw = 1, 1, -1                                -- unpowered stick cannot move them
        f.wind = 3 + 10 * r() + ((t % 7 < 0.5) and 12 or 0)              -- gusts
        f.wind_dir = (200 + 120 * sweep(t, 11) + 5 * r()) % 360
        f.heading = (t < 15) and 90 or (90 + (t - 15) * 20) % 360
        if t > 20 and t <= 22 then f.wind, f.wind_dir = 0.25, 10 end     -- exactly at the 0.25 m/s threshold
        if t > 22 and t <= 24 then f.wind = 0.2499 end                   -- just below it
        if t > 24 then f.wind = 0.1 end                                  -- calm: holds the last pose
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

scenario("unpowered ground roll airflow and lift-off", function()
    local out, t, dt = {}, 0, 1 / 60
    for _ = 1, seconds(30, dt) do
        local f = base(t)
        f.p1, f.p2, f.p3 = 200, 0, 0
        f.airspeed = math.min(80, t * 4)
        f.roll, f.pitch, f.yaw = 0.3 * sweep(t, 2), -0.2, 0.1
        f.wind, f.wind_dir = 5, 45
        if t > 22 then f.grounded = 0 end                                -- airborne without pressure
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

scenario("pause, invalid input and frame-period anomalies", function()
    local out, t = {}, 0
    local function add(f) out[#out + 1] = f end
    for i = 1, 600 do
        local f = base(t)
        f.grounded = (i > 200 and i < 400) and 0 or 1
        f.airspeed = 70
        f.roll, f.pitch, f.yaw = sweep(t, 2), 0.5 * sweep(t, 3), 0.3
        f.sb0, f.sb1 = 20, 20
        if i >= 50 and i < 120 then f.paused = 1 end
        if i == 130 then f.dt = 0 end
        if i == 131 then f.dt = -0.02 end
        if i == 132 then f.dt = 2.5 end
        if i == 133 then f.dt = 0 / 0 end
        if i == 134 then f.dt = math.huge end
        if i >= 140 and i < 150 then f.roll = 0 / 0 end                 -- invalid aileron input
        if i >= 160 and i < 165 then f.grounded = 2 end                  -- invalid flag (both drivers)
        if i >= 170 and i < 175 then f.paused = 3 end
        if i >= 180 and i < 185 then f.pitch = 0 / 0 end                 -- invalid hydraulic input
        if i >= 186 and i < 190 then f.sp2 = math.huge end
        if i >= 450 then f.dt = (i % 3 == 0) and 1 / 144 or (i % 3 == 1) and 1 / 20 or 0.031 end
        add(f); t = t + 1 / 60
    end
    return out
end)

scenario("random stress with mixed frame rates", function()
    local out, t, r = {}, 0, lcg(1234)
    for i = 1, 2400 do
        local f = base(t)
        f.dt = ({1 / 30, 1 / 60, 1 / 90, 1 / 144, 0.004, 0.07})[1 + math.floor(r() * 6)]
        f.grounded = (math.floor(t / 5) % 2 == 0) and 1 or 0
        f.roll, f.pitch, f.yaw = 2.4 * r() - 1.2, 2.4 * r() - 1.2, 2.4 * r() - 1.2
        local p = (math.floor(t / 3) % 3 == 0) and 0 or (math.floor(t / 3) % 3 == 1) and 650 or 3000
        f.p1, f.p2, f.p3 = p, p * r(), 0
        f.airspeed, f.heading, f.wind_dir, f.wind = 90 * r(), 360 * r(), 360 * r(), 15 * r()
        f.sb0, f.sb1, f.sp0, f.sp1 = 80 * r() - 5, 80 * r(), 60 * r(), 60 * r()
        f.sp2, f.sp3, f.sq4, f.sq5 = 60 * r(), 60 * r(), 60 * r(), 60 * r()
        if i % 400 == 0 then f.paused = 1 end
        out[#out + 1] = f; t = t + f.dt
    end
    return out
end)

scenario("aircraft reload mid-flight", function()
    local out, t, dt = {}, 0, 1 / 60
    for i = 1, seconds(10, dt) do
        local f = base(t)
        f.grounded, f.airspeed = 0, 120
        f.roll, f.pitch, f.yaw = 0.7 * sweep(t, 2.5), 0.4, -0.2
        f.sb0, f.sb1 = 15, 15
        if i == 200 or i == 420 then f.reload = true end                -- FlyWithLua script reload
        if i > 300 then f.grounded, f.p1, f.p2 = 1, 0, 0 end            -- landed and shut down
        out[#out + 1] = f; t = t + dt
    end
    return out
end)

-- ---------------------------------------------------------------- run
local fh = assert(io.open(output, "w"))
fh:write("# A380X surface golden v1: lua=", _VERSION, "\n")
fh:write("# columns: ", table.concat(COLUMNS, " "), " | ")
for _, row in ipairs(CUSTOM) do fh:write(row[1], " ") end
fh:write("\n")
local function num(v)
    if v ~= v then return "nan" end
    if v == math.huge then return "inf" end
    if v == -math.huge then return "-inf" end
    return string.format("%.17g", v)
end
local frames_total = 0
for _, sc in ipairs(scenarios) do
    fh:write("S ", sc.name, "\n")
    for _, row in ipairs(CUSTOM) do custom_value[row[1]] = 0 end         -- fresh plugin
    local frame_fn
    for _, f in ipairs(sc.frames) do
        set_inputs(f)
        if frame_fn == nil or f.reload then
            if f.reload then fh:write("R\n") end
            frame_fn = new_host()                                       -- script load (no frame callback)
        else
            frame_fn()
        end
        local cols = {}
        for _, row in ipairs(SIM) do cols[#cols + 1] = num(sim_scalar[row[2]]) end
        for _, row in ipairs(ARRAYS) do
            for k = 1, #row[2] do cols[#cols + 1] = num(sim_array[row[1]][row[3] + k - 1]) end
        end
        local outs = {}
        for _, row in ipairs(CUSTOM) do outs[#outs + 1] = num(custom_value[row[1]]) end
        fh:write("F ", table.concat(cols, " "), " | ", table.concat(outs, " "), "\n")
        frames_total = frames_total + 1
    end
end
fh:close()
for _, text in ipairs(messages) do
    if text:find("disabled", 1, true) or text:find("runtime error", 1, true) then
        error("a Lua driver disabled itself: " .. text)
    end
end
print(string.format("surface golden: %d scenarios, %d frames, %d Lua log lines", #scenarios, frames_total, #messages))
