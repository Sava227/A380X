-- SPDX-License-Identifier: GPL-3.0-or-later
-- No two writers: the FlyWithLua drivers stand down while the aircraft plugin's native driver
-- publishes a380x/xp12/hydraulics/native_driver = 1 (native/aileron_datarefs).
--   lua tests/native_driver_standdown_test.lua <FlyWithLua dir>
local root = assert(arg[1], "pass the FlyWithLua directory")
local FLAG = "a380x/xp12/hydraulics/native_driver"
local function host(flag_value, path)
    local log = {writes = 0, frames = {}, messages = {}, bindings = {}}
    local values, native = {}, {
        ["sim/flightmodel2/controls/roll_ratio"] = 0.5, ["sim/flightmodel/failures/onground_any"] = 1,
        ["sim/time/paused"] = 0, ["sim/operation/misc/frame_rate_period"] = 1 / 60,
        ["sim/flightmodel2/controls/pitch_ratio"] = 0.2, ["sim/flightmodel2/controls/heading_ratio"] = 0.1,
        ["sim/flightmodel/position/psi"] = 90, ["sim/weather/aircraft/wind_now_direction_degt"] = 0,
        ["sim/weather/aircraft/wind_now_speed_msc"] = 3, ["sim/flightmodel/position/true_airspeed"] = 0,
        ["sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1"] = 3000,
        ["sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2"] = 0,
        ["sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3"] = 0,
    }
    local arrays = {["sim/flightmodel2/wing/speedbrake1_deg"] = {[0] = 10, [1] = 10},
        ["sim/flightmodel2/wing/spoiler1_deg"] = {[0] = 0, [1] = 0, [2] = 0, [3] = 0},
        ["sim/flightmodel2/wing/spoiler2_deg"] = {[4] = 0, [5] = 0}}
    for _, side in ipairs({"l", "r"}) do
        for _, panel in ipairs({"inboard", "midboard", "outboard"}) do
            values["a380x/xp12/visual/aileron_" .. side .. "_" .. panel .. "_ratio"] = 0
            values["a380x/xp12/hydraulics/aileron_" .. side .. "_" .. panel .. "_ratio"] = 0
        end
    end
    values["a380x/xp12/hydraulics/elevator_ratio"], values["a380x/xp12/hydraulics/rudder_ratio"] = 0, 0
    for _, panel in ipairs({"inboard", "inner", "middle", "outer"}) do
        for _, side in ipairs({"l", "r"}) do values["a380x/xp12/hydraulics/spoiler_" .. side .. "_" .. panel .. "_deg"] = 0 end
    end
    local flag = {value = flag_value}
    local env = setmetatable({}, {__index = _G})
    env._G, env.PLANE_ICAO, env.AIRCRAFT_PATH = env, "A388", path or "/Aircraft/A380X-XP12/"
    env.logMsg = function(text) log.messages[#log.messages + 1] = text end
    local handles, names = {}, {}
    local function handle(name)
        if not handles[name] then handles[name] = setmetatable({}, {}); names[handles[name]] = name end
        return handles[name]
    end
    env.XPLMFindDataRef = function(name)
        if name == FLAG then return flag.value ~= nil and handle(name) or nil end
        if values[name] ~= nil or native[name] ~= nil or arrays[name] ~= nil then return handle(name) end
        return nil
    end
    env.XPLMGetDataRefTypes = function(h)
        local name = names[h]
        return name == FLAG and 1 or arrays[name] and 8 or values[name] ~= nil and 2 or
            (name:find("onground") or name:find("paused")) and 1 or 2
    end
    env.dataref_table = function(name)
        log.bindings[#log.bindings + 1] = name
        return setmetatable({}, {
            __index = function(_, i)
                if name == FLAG then return flag.value end
                if arrays[name] then return arrays[name][i] end
                if values[name] ~= nil then return values[name] end
                return native[name]
            end,
            __newindex = function(_, i, v)
                assert(values[name] ~= nil and i == 0, "write to " .. name)
                values[name] = v; log.writes = log.writes + 1
            end,
        })
    end
    env.do_every_frame = function(code) log.frames[#log.frames + 1] = assert(load(code, "frame", "t", env)) end
    env.do_on_exit = function() end
    env.require = function(module) return assert(loadfile(root .. "/Modules/" .. module .. ".lua", "t", env))() end
    for _, script in ipairs({"A380X_XP12_Ailerons.lua", "A380X_XP12_Hydraulics.lua"}) do
        assert(loadfile(root .. "/Scripts/" .. script, "t", env))()
    end
    log.frame = function() for _, chunk in ipairs(log.frames) do chunk() end end
    return log, flag, values, env
end
local function count(log, needle)
    local n = 0
    for _, text in ipairs(log.messages) do if text:find(needle, 1, true) then n = n + 1 end end
    return n
end

-- 1. Native driver running at load: neither script binds an output, registers a frame or writes.
local log, _, _, env = host(1)
assert(#log.frames == 0 and log.writes == 0, "a Lua driver started next to the native driver")
assert(env.A380X_AILERON_VISUALS == nil and env.A380X_HYDRAULIC_SURFACES == nil)
assert(count(log, "stands down") == 2)
for _, name in ipairs(log.bindings) do assert(name == FLAG, "bound " .. name .. " while standing down") end

-- 2. Flag present but 0 (native plugin disabled): the Lua drivers run; the moment the flag becomes 1
-- they stop without writing again, leaving the native values in place.
local log2, flag2, values2, env2 = host(0)
assert(#log2.frames == 2 and env2.A380X_AILERON_VISUALS.enabled and env2.A380X_HYDRAULIC_SURFACES.enabled)
for _ = 1, 30 do log2.frame() end
local writes = log2.writes
assert(writes > 0)
flag2.value = 1
values2["a380x/xp12/visual/aileron_l_inboard_ratio"] = 0.123   -- the native driver's value
for _ = 1, 30 do log2.frame() end
assert(log2.writes == writes, "a Lua driver wrote after the native driver took over")
assert(values2["a380x/xp12/visual/aileron_l_inboard_ratio"] == 0.123)
assert(not env2.A380X_AILERON_VISUALS.enabled and not env2.A380X_HYDRAULIC_SURFACES.enabled)

-- 3. No flag (the dev aircraft's older plugin): unchanged behaviour.
local log3 = host(nil)
assert(#log3.frames == 2 and log3.writes > 0 and count(log3, "stands down") == 0)

-- 4. The path gate is unchanged: another folder name never starts the Lua drivers.
local log4 = host(nil, "/Aircraft/A380X-XP12 Latest/")
assert(#log4.frames == 0 and log4.writes == 0)
print("Native-driver stand-down checks passed (" .. _VERSION .. "): no Lua writer while native_driver = 1, "
      .. "hand-over without writes, unchanged without the flag, unchanged path gate")
