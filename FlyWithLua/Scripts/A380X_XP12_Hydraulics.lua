-- SPDX-License-Identifier: GPL-3.0-or-later
-- Exterior hydraulic settling, layered after the existing three-panel driver.
-- All simulator inputs are read-only; only early-registered custom floats change.
local function message(text)
    if type(logMsg) == "function" then logMsg("A380X hydraulic surfaces: " .. text) end
end
local function target_aircraft()
    if PLANE_ICAO ~= "A388" or type(AIRCRAFT_PATH) ~= "string" then return false end
    return ("/" .. AIRCRAFT_PATH:gsub("\\", "/") .. "/"):find("/A380X-XP12/", 1, true) ~= nil
end
if not target_aircraft() then return end
if rawget(_G, "A380X_HYDRAULIC_SURFACES") ~= nil then
    message("callback namespace already owned; driver disabled"); return
end
for _, name in ipairs({"XPLMFindDataRef", "XPLMGetDataRefTypes", "dataref_table",
                       "do_every_frame", "do_on_exit", "require"}) do
    if type(_G[name]) ~= "function" then message("missing API " .. name); return end
end
-- Never a second writer: the aircraft-local A380XAileronDatarefs plugin has its own native driver
-- (a C port of this script and its module, run from a flight loop). It publishes
-- a380x/xp12/hydraulics/native_driver = 1 while it drives these datarefs; then this script stands
-- down without writing anything. The flag is looked up once at load (aircraft plugins start before
-- FlyWithLua loads its scripts) and read every frame. Absent or 0: this Lua driver runs as before.
local NATIVE_DRIVER = "a380x/xp12/hydraulics/native_driver"
local native_flag = nil
do
    local found, reference = pcall(XPLMFindDataRef, NATIVE_DRIVER)
    if found and reference ~= nil and reference ~= false and reference ~= 0 then
        local bound, handle = pcall(dataref_table, NATIVE_DRIVER)
        if bound and type(handle) == "table" then native_flag = handle end
    end
end
local function native_driver_active()
    if native_flag == nil then return false end
    local ok, value = pcall(function() return native_flag[0] end)
    return ok and value == 1
end
if native_driver_active() then
    message("native A380XAileronDatarefs driver active (" .. NATIVE_DRIVER .. " = 1); Lua driver stands down")
    return
end
local ok, core = pcall(require, "a380x_hydraulic_surfaces")
if not ok or type(core) ~= "table" or type(core.new) ~= "function"
        or type(core.valid) ~= "function" or type(core.update) ~= "function"
        or type(core.KEYS) ~= "table" or #core.KEYS ~= 16 then
    message("missing/incompatible hydraulic module; driver disabled"); return
end
local prefix = "a380x/xp12/hydraulics/"
local inputs, outputs = {}, {}
local input_names = {
    grounded = "sim/flightmodel/failures/onground_any", paused = "sim/time/paused",
    dt = "sim/operation/misc/frame_rate_period",
    pitch = "sim/flightmodel2/controls/pitch_ratio",
    rudder = "sim/flightmodel2/controls/heading_ratio",
    heading = "sim/flightmodel/position/psi",
    wind_direction = "sim/weather/aircraft/wind_now_direction_degt",
    wind_speed = "sim/weather/aircraft/wind_now_speed_msc",
    airspeed = "sim/flightmodel/position/true_airspeed",
    speedbrake1 = "sim/flightmodel2/wing/speedbrake1_deg",
    spoiler1 = "sim/flightmodel2/wing/spoiler1_deg",
    spoiler2 = "sim/flightmodel2/wing/spoiler2_deg"
}
for i = 1, 3 do
    input_names["pressure" .. i] = "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_" .. i
end
for i = 1, 6 do input_names["aileron" .. i] = "a380x/xp12/visual/" .. core.KEYS[i] end
local function bind(name, scalar)
    local reference = XPLMFindDataRef(name)
    assert(reference ~= nil and reference ~= false and reference ~= 0, "missing dataref " .. name)
    if scalar then assert(XPLMGetDataRefTypes(reference) == 2, "expected scalar Float " .. name) end
    local handle = dataref_table(name)
    assert(type(handle) == "table", "cannot bind " .. name)
    return handle
end
local bound, err = pcall(function()
    for key, name in pairs(input_names) do inputs[key] = bind(name, key:match("^aileron") ~= nil) end
    for _, key in ipairs(core.KEYS) do outputs[key] = bind(prefix .. key, true) end
end)
if not bound then message(tostring(err) .. "; driver disabled; restart with complete aircraft plugin"); return end
local function input()
    local grounded, paused = inputs.grounded[0], inputs.paused[0]
    if (grounded ~= 0 and grounded ~= 1) or (paused ~= 0 and paused ~= 1) then return nil end
    local current = {grounded = grounded == 1, paused = paused == 1, pressures = {}, ailerons = {}}
    for _, key in ipairs({"pitch", "rudder", "heading", "wind_direction", "wind_speed", "airspeed"}) do
        current[key] = inputs[key][0]
    end
    for i = 1, 3 do current.pressures[i] = inputs["pressure" .. i][0] end
    for i = 1, 6 do current.ailerons[i] = inputs["aileron" .. i][0] end
    current.spoilers = {inputs.speedbrake1[0], inputs.speedbrake1[1],
        inputs.spoiler1[0], inputs.spoiler1[1], inputs.spoiler1[2], inputs.spoiler1[3],
        inputs.spoiler2[4], inputs.spoiler2[5]}
    if not core.valid(current) then return nil end
    return current
end
local function publish(values)
    -- Validate the complete update before publishing any part of it.
    for i, key in ipairs(core.KEYS) do
        local value = values[key]
        assert(type(value) == "number" and value == value and math.abs(value) < math.huge,
               "invalid output " .. key)
        assert(value >= (i <= 8 and -1 or 0) and value <= (i <= 8 and 1 or 65),
               "out-of-range output " .. key)
    end
    for _, key in ipairs(core.KEYS) do outputs[key][0] = values[key] end
end
local state, active, warned_input = nil, false, false
local provider = {enabled = true}
local function frame()
    if native_driver_active() then
        -- The native driver took over: stop without writing (its values stay published).
        provider.enabled, active = false, false
        message("native A380XAileronDatarefs driver active; Lua driver stands down")
        return
    end
    if not target_aircraft() then
        state, active = nil, false
        return
    end
    local current = input()
    if not current then
        if not warned_input then message("invalid source input; holding last surface positions") end
        warned_input = true
        return
    end
    warned_input = false
    if not active then
        state = core.new(current)
        active = true
        publish(state.outputs)
    else publish(core.update(state, current, inputs.dt[0])) end
end
function provider.update()
    if not provider.enabled then return end
    local success, failure = pcall(frame)
    if not success then
        provider.enabled = false
        message("runtime error; holding last positions: " .. tostring(failure))
    end
end
function provider.shutdown()
    provider.enabled, active = false, false
    -- Keep the last supported pose until aircraft unload; no shutdown snap.
end
A380X_HYDRAULIC_SURFACES = provider
do_every_frame("A380X_HYDRAULIC_SURFACES.update()")
do_on_exit("A380X_HYDRAULIC_SURFACES.shutdown()")
provider.update()
message("pressure-dependent exterior driver loaded; simulator flight controls unchanged")
