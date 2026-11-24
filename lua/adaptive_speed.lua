--[[
  adaptive_speed.lua - адаптивна швидкість для WildThumper XL (ArduRover)

  У AUTO та GUIDED (рух до точки) ровер їде швидко, поки попереду вільно,
  і сповільнюється, коли лідар бачить перешкоду - щоб BendyRuler встигав об'їхати.
    * перешкода в передній півсфері ближче за ASPD_NEAR  -> ASPD_SLOW
    * попереду вільно далі за ASPD_CLEAR протягом ASPD_HOLD с -> ASPD_FAST
  Між NEAR і CLEAR - гістерезис: швидкість не "смикається" на межі.

  Швидкість задається через vehicle:set_desired_speed() (= wp_nav.set_speed_max).
  ArduPilot просить викликати це не частіше ~3 Гц, тому скрипт змінює швидкість
  лише при переході між рівнями і не частіше разу на 0.5 с.

  Поки ASPD_ENABLE = 1, скрипт керує швидкістю замість DO_CHANGE_SPEED у місії.
  ASPD_ENABLE = 0 - скрипт нічого не робить (діють швидкості місії / WP_SPEED).

  Встановлення (SITL): ~/ardupilot/scripts/adaptive_speed.lua, reboot.
  Параметри з'являться в списку як ASPD_* (після reboot - ще один reboot або
  Refresh Params у Mission Planner).
--]]

local PARAM_TABLE_KEY = 73
assert(param:add_table(PARAM_TABLE_KEY, "ASPD_", 6), "ASPD: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1),   "ASPD: ENABLE")
assert(param:add_param(PARAM_TABLE_KEY, 2, "FAST",   8.0), "ASPD: FAST")   -- м/с на відкритій місцевості
assert(param:add_param(PARAM_TABLE_KEY, 3, "SLOW",   3.0), "ASPD: SLOW")   -- м/с біля перешкод
assert(param:add_param(PARAM_TABLE_KEY, 4, "NEAR",  20.0), "ASPD: NEAR")   -- м, ближче -> SLOW
assert(param:add_param(PARAM_TABLE_KEY, 5, "CLEAR", 28.0), "ASPD: CLEAR")  -- м, далі -> можна FAST
assert(param:add_param(PARAM_TABLE_KEY, 6, "HOLD",   3.0), "ASPD: HOLD")   -- с вільного шляху до FAST

local P_ENABLE = Parameter("ASPD_ENABLE")
local P_FAST   = Parameter("ASPD_FAST")
local P_SLOW   = Parameter("ASPD_SLOW")
local P_NEAR   = Parameter("ASPD_NEAR")
local P_CLEAR  = Parameter("ASPD_CLEAR")
local P_HOLD   = Parameter("ASPD_HOLD")

local PERIOD_MS = 200            -- 5 Гц перевірки
local MIN_SET_INTERVAL = 0.5     -- с між викликами set_desired_speed
local FRONT_HALF_ANGLE = 90      -- град: передня півсфера
local PRX_GOOD = 2
local ACTIVE_MODES = { [10] = "AUTO", [15] = "GUIDED" }
local MAV_SEVERITY_INFO = 6

local level = nil                -- "FAST" / "SLOW" / nil (ще не задано)
local clear_since = nil
local last_set = -100.0
local last_mode = -1

local function now_s()
  return millis():tofloat() * 0.001
end

-- найближча перешкода в передній півсфері (м) або nil
local function nearest_front()
  local best = nil
  for i = 0, proximity:get_object_count() - 1 do
    local angle_deg, dist = proximity:get_object_angle_and_distance(i)
    if angle_deg ~= nil and dist ~= nil then
      local a = (angle_deg + 180) % 360 - 180         -- -180..180, 0 = вперед
      if math.abs(a) <= FRONT_HALF_ANGLE and (best == nil or dist < best) then
        best = dist
      end
    end
  end
  return best
end

local function apply(new_level, dist)
  local t = now_s()
  if new_level == level or t - last_set < MIN_SET_INTERVAL then
    return
  end
  local spd = (new_level == "FAST") and P_FAST:get() or P_SLOW:get()
  if vehicle:set_desired_speed(spd) then
    level = new_level
    last_set = t
    gcs:send_text(MAV_SEVERITY_INFO, string.format("ASPD: %s %.1f m/s (obstacle %s)",
      new_level, spd, dist and string.format("%.0f m", dist) or "none"))
    logger:write("ASPD", "Spd,Dist", "ff", spd, dist or -1)
  end
end

local function update()
  local mode = vehicle:get_mode()
  if P_ENABLE:get() < 1 or not arming:is_armed() or ACTIVE_MODES[mode] == nil then
    level = nil
    clear_since = nil
    last_mode = mode
    return update, PERIOD_MS
  end
  if mode ~= last_mode then
    level = nil                -- новий режим/місія: задати швидкість заново
    last_mode = mode
  end

  -- без даних лідара - тільки повільно (lidar_guard окремо вирішує щодо HOLD)
  if proximity:get_status() ~= PRX_GOOD then
    clear_since = nil
    apply("SLOW", nil)
    return update, PERIOD_MS
  end

  local d = nearest_front()
  local t = now_s()
  if d ~= nil and d < P_NEAR:get() then
    clear_since = nil
    apply("SLOW", d)
  elseif d == nil or d > P_CLEAR:get() then
    if clear_since == nil then
      clear_since = t
    end
    if t - clear_since >= P_HOLD:get() then
      apply("FAST", d)
    elseif level == nil then
      apply("SLOW", d)         -- старт: спершу повільно, поки не переконаємось
    end
  else
    -- між NEAR і CLEAR: лишаємо поточний рівень (гістерезис)
    clear_since = nil
    if level == nil then
      apply("SLOW", d)
    end
  end
  return update, PERIOD_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "ASPD: adaptive speed loaded")
return update, 1000
