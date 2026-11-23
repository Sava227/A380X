-- SPDX-License-Identifier: GPL-3.0-or-later
-- Visual animation provider only: no native control writes or flight-law override.
-- Install a380x_aileron_dance.lua in FlyWithLua/Modules. FlyWithLua includes
-- MODULES_DIRECTORY .. "?.lua" in package.path; no platform-specific path needed.
-- The aircraft-local early-registration plugin must publish the six scalar
-- Floats before X-Plane loads the OBJ animations. Lua must never create them.

local function message(text)
    if type(logMsg) == "function" then
        logMsg("A380X aileron visuals: " .. text)
    end
end

local function target_aircraft()
    if PLANE_ICAO ~= "A388" or type(AIRCRAFT_PATH) ~= "string" then return false end
    local path = "/" .. AIRCRAFT_PATH:gsub("\\", "/") .. "/"
    return path:find("/A380X-XP12/", 1, true) ~= nil
end

local function finite(value)
    return type(value) == "number" and value == value and math.abs(value) < math.huge
end

-- FlyWithLua reloads all scripts with a new Lua state when the aircraft changes.
-- The frame guard below also covers the interval before that reload.
if not target_aircraft() then return end
if rawget(_G, "A380X_AILERON_VISUALS") ~= nil then
    message("callback namespace already owned; provider disabled")
    return
end
for _, name in ipairs({"XPLMFindDataRef", "XPLMGetDataRefTypes", "dataref_table",
                        "do_every_frame", "do_on_exit", "require"}) do
    if type(_G[name]) ~= "function" then
        message("missing API " .. name .. "; provider disabled")
        return
    end
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

local keys = {"l_inboard_ratio", "l_midboard_ratio", "l_outboard_ratio",
              "r_inboard_ratio", "r_midboard_ratio", "r_outboard_ratio"}
local prefix = "a380x/xp12/visual/aileron_"
local source_names = {"sim/flightmodel2/controls/roll_ratio",
                      "sim/flightmodel/failures/onground_any", "sim/time/paused",
                      "sim/operation/misc/frame_rate_period"}
local function exists(name)
    local ok, reference = pcall(XPLMFindDataRef, name)
    if not ok then return nil end
    return reference ~= nil and reference ~= false and reference ~= 0, reference
end
-- All six scalar Floats (SDK type bit 2) must already exist from the early
-- aircraft plugin. Reject absent/partial/wrong-type sets: creating them here
-- can be too late for OBJ dataref resolution. Lua reloads only rebind them.
-- These APIs do not expose owner identity; an active duplicate in the same
-- Lua state is rejected above. Another plugin claiming this exact complete
-- six-name/type contract is otherwise indistinguishable.
for _, name in ipairs(source_names) do
    if exists(name) ~= true then
        message("missing source dataref " .. name .. "; provider disabled")
        return
    end
end
local existing_count = 0
for _, key in ipairs(keys) do
    local found, reference = exists(prefix .. key)
    if found == nil then
        message("custom dataref lookup failure: " .. prefix .. key .. "; provider disabled")
        return
    end
    if found then
        local typed, kind = pcall(XPLMGetDataRefTypes, reference)
        if not typed or kind ~= 2 then
            message("custom dataref type collision: " .. prefix .. key .. "; provider disabled")
            return
        end
        existing_count = existing_count + 1
    end
end
if existing_count ~= #keys then
    if existing_count == 0 then
        message("early-registration plugin missing; fully restart X-Plane; provider disabled")
    else
        message("partial early-registration dataref set; fully restart X-Plane; provider disabled")
    end
    return
end

local loaded, core = pcall(require, "a380x_aileron_dance")
if not loaded or type(core) ~= "table" or type(core.new) ~= "function"
        or type(core.reset) ~= "function" or type(core.update) ~= "function" then
    message("missing or incompatible Modules/a380x_aileron_dance.lua; provider disabled")
    return
end
local function valid_outputs(outputs)
    if type(outputs) ~= "table" then return false end
    for _, key in ipairs(keys) do
        if not finite(outputs[key]) then return false end
    end
    return true
end
local neutral = {roll = 0, grounded = true, paused = false}
local ok, state = pcall(core.new)
local initial_ok, initial
if ok and type(state) == "table" then initial_ok, initial = pcall(core.reset, state, neutral) end
if not initial_ok or not valid_outputs(initial) then
    message("core initialization failed; provider disabled")
    return
end

-- These private handles are read ONLY. Do not introduce global dataref aliases,
-- writable native bindings, joystick setters, or control-surface overrides.
local sources = {}
for index, name in ipairs(source_names) do
    local bound, handle = pcall(dataref_table, name)
    if not bound or type(handle) ~= "table" then
        message("cannot bind source " .. name .. "; provider disabled")
        return
    end
    sources[index] = handle
end
local function input()
    local roll, grounded, paused = sources[1][0], sources[2][0], sources[3][0]
    if not finite(roll) or (grounded ~= 0 and grounded ~= 1)
            or (paused ~= 0 and paused ~= 1) then return nil end
    return {roll = math.max(-1, math.min(1, roll)), grounded = grounded == 1, paused = paused == 1}
end
local read_ok, first_input = pcall(input)
if read_ok and first_input then initial_ok, initial = pcall(core.reset, state, first_input) end
if not read_ok or not initial_ok or not valid_outputs(initial) then
    message("initial input or core reset failed; provider disabled")
    return
end

local outputs = {}
for _, key in ipairs(keys) do
    local bound, handle = pcall(dataref_table, prefix .. key)
    if not bound or type(handle) ~= "table" then
        message("host failed to bind custom " .. prefix .. key .. "; no callbacks registered")
        return
    end
    outputs[key] = handle
end
local reported_nonzero = false
local function publish(values)
    assert(valid_outputs(values), "core returned an invalid animation value")
    local nonzero = false
    for _, key in ipairs(keys) do
        outputs[key][0] = math.max(-1, math.min(1, values[key]))
        nonzero = nonzero or math.abs(values[key]) > 0.000001
    end
    if nonzero and not reported_nonzero then
        -- One-time readback shows whether the real SDK-backed tables accepted
        -- the first nonzero publication; a load log alone cannot establish it.
        local requested, actual = {}, {}
        for _, key in ipairs(keys) do
            requested[#requested + 1] = string.format("%.6f", values[key])
            actual[#actual + 1] = tostring(outputs[key][0])
        end
        message("first nonzero publication (L in/mid/out, R in/mid/out): requested="
            .. table.concat(requested, ",") .. "; SDK readback=" .. table.concat(actual, ","))
        reported_nonzero = true
    end
end
local function clear()
    for _, key in ipairs(keys) do outputs[key][0] = 0 end
end

local active = first_input ~= nil
local warned_input = false
local reported_frame = false
local provider = {enabled = true}
local function disable(reason)
    provider.enabled, active = false, false
    pcall(clear)
    message(reason .. "; provider disabled (native flight controls unchanged)")
end
local function frame()
    if native_driver_active() then
        -- The native driver took over: stop without clearing (its values stay published).
        provider.enabled, active = false, false
        message("native A380XAileronDatarefs driver active; Lua driver stands down")
        return
    end
    if not target_aircraft() then
        if active then
            core.reset(state, neutral)
            clear()
            active = false
        end
        return
    end
    local current = input()
    -- FlyWithLua does not define XLua's SIM_PERIOD global. Read X-Plane's
    -- simulation frame-period dataref explicitly through a private handle.
    local dt = sources[4][0]
    if not reported_frame then
        message("first frame callback: roll=" .. tostring(current and current.roll)
            .. "; grounded=" .. tostring(current and current.grounded)
            .. "; paused=" .. tostring(current and current.paused)
            .. "; frame_rate_period=" .. tostring(dt))
        reported_frame = true
    end
    if not current then
        if active then core.reset(state, neutral); clear(); active = false end
        if not warned_input then message("invalid source input; visuals neutral until valid data returns") end
        warned_input = true
        return
    end
    warned_input = false
    if not active then
        publish(core.reset(state, current))
        active = true
        return
    end
    if current.paused then return end
    -- The core handles zero, negative, nonfinite and discontinuous frame periods.
    publish(core.update(state, current, dt))
end
function provider.update()
    if not provider.enabled then return end
    local frame_ok, err = pcall(frame)
    if not frame_ok then disable("runtime error: " .. tostring(err)) end
end
function provider.shutdown()
    if not provider.enabled then return end
    pcall(core.reset, state, neutral)
    local cleared, err = pcall(clear)
    provider.enabled, active = false, false
    if not cleared then message("could not clear visual values on shutdown: " .. tostring(err)) end
end

local published, err = pcall(publish, initial)
if not published then disable("initial publication failed: " .. tostring(err)); return end
A380X_AILERON_VISUALS = provider
do_every_frame("A380X_AILERON_VISUALS.update()")
do_on_exit("A380X_AILERON_VISUALS.shutdown()")
message("visual-only provider loaded; six animation datarefs bound from early-registration plugin"
    .. "; native flight controls unchanged")
