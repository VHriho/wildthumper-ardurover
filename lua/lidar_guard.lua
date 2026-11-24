--[[
  lidar_guard.lua - сторож лідара для WildThumper XL (ArduRover, skid-steer)

  Закриває дві прогалини, знайдені на етапі 2:
   1. Втрата даних лідара (AP_Proximity NoData) - у Rover немає failsafe,
      ровер їде далі наосліп. Сторож переводить ровер у HOLD.
   2. Аварійне гальмо: гальмування AC_Avoid за лідаром вимкнене (AVOID_ENABLE 1),
      бо заважало BendyRuler. Сторож зупиняє ровер (HOLD), лише якщо перешкода
      в КОРИДОРІ РУХУ попереду ближче за гальмівний шлях + запас.
      Перешкоди збоку (BendyRuler проходить повз них на ~0.5 м) не чіпає.

  Працює лише коли ровер озброєний і в автоматичних/напівавтоматичних режимах:
  ACRO, AUTO, RTL, GUIDED. У MANUAL оператор має повний контроль.

  Встановлення (SITL): ~/ardupilot/scripts/lidar_guard.lua, SCR_ENABLE 1, reboot.
--]]

-- ---------------- налаштування ----------------
local PERIOD_MS      = 100     -- 10 Гц
local NODATA_HOLD_S  = 0.5     -- скільки секунд без даних лідара до HOLD
local HALF_WIDTH     = 0.71    -- м, від осі ровера до борту
local SIDE_MARGIN    = 0.20    -- м, запас по ширині коридору
local FRONT_EDGE     = 0.475   -- м, від лідара до переднього краю ровера
                               --   (лідар на 0.42 м попереду центру, ровер 2.0 м)
local STOP_MARGIN    = 0.40    -- м, мінімальний зазор перед бампером після зупинки
local LATENCY_S      = 0.30    -- с, затримка: лідар 10 Гц + підтвердження 2 скани + реакція
local MIN_SPEED      = 0.30    -- м/с, нижче - не гальмуємо (дозволяємо розворот на місці)
local REARM_S        = 3.0     -- с, пауза між повторними спрацюваннями

-- режими ArduRover
local MODE_HOLD = 4
local WATCHED = { [1] = "ACRO", [10] = "AUTO", [11] = "RTL", [15] = "GUIDED" }

-- стан proximity (AP_Proximity::Status)
local PRX_GOOD = 2

local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_WARNING  = 4
local MAV_SEVERITY_INFO     = 6

-- ---------------- стан скрипта ----------------
local nodata_since = nil
local last_trigger = -100.0
local was_active = false

local function now_s()
  return millis():tofloat() * 0.001
end

local function decel_max()
  -- гальмівне прискорення ровера (ATC_DECEL_MAX), запасно 1.0 м/с^2
  local d = param:get("ATC_DECEL_MAX")
  if d == nil or d <= 0 then
    return 1.0
  end
  return d
end

-- code: 1 = немає даних лідара, 2 = перешкода в коридорі
local function trigger(code, text, severity, gap, spd)
  local t = now_s()
  if t - last_trigger < REARM_S then
    return
  end
  last_trigger = t
  vehicle:set_mode(MODE_HOLD)
  gcs:send_text(severity, "LidarGuard: " .. text .. " -> HOLD")
  -- запис у DataFlash-лог: повідомлення LGRD (код, зазор до бампера, швидкість)
  logger:write("LGRD", "Code,Gap,Spd", "Bff", code, gap, spd)
end

-- найближча перешкода в коридорі руху попереду; повертає зазор до бампера (м) або nil
local function corridor_gap()
  local best = nil
  local half = HALF_WIDTH + SIDE_MARGIN
  for i = 0, proximity:get_object_count() - 1 do
    local angle_deg, dist = proximity:get_object_angle_and_distance(i)
    if angle_deg ~= nil and dist ~= nil then
      local a = math.rad(angle_deg)          -- 0 = вперед, за годинниковою = праворуч
      local x = dist * math.cos(a)           -- вперед від лідара
      local y = dist * math.sin(a)           -- праворуч від лідара
      if x > 0 and math.abs(y) <= half then
        local gap = x - FRONT_EDGE
        if best == nil or gap < best then
          best = gap
        end
      end
    end
  end
  return best
end

local function update()
  local mode = vehicle:get_mode()
  if not arming:is_armed() or WATCHED[mode] == nil then
    nodata_since = nil
    was_active = false
    return update, PERIOD_MS
  end
  if not was_active then
    -- повідомлення при кожному переході в активний стан (видно, що скрипт працює)
    gcs:send_text(MAV_SEVERITY_INFO, "LidarGuard: active in " .. WATCHED[mode])
    was_active = true
  end

  -- 1. чи є дані лідара
  if proximity:get_status() ~= PRX_GOOD then
    local t = now_s()
    if nodata_since == nil then
      nodata_since = t
    elseif t - nodata_since >= NODATA_HOLD_S then
      trigger(1, "no lidar data", MAV_SEVERITY_CRITICAL, -1, ahrs:groundspeed_vector():length())
      nodata_since = nil
    end
    return update, PERIOD_MS
  end
  nodata_since = nil

  -- 2. аварійне гальмо: перешкода в коридорі ближче за гальмівний шлях
  local v = ahrs:groundspeed_vector():length()
  if v < MIN_SPEED then
    return update, PERIOD_MS
  end
  local gap = corridor_gap()
  if gap ~= nil then
    local need = v * LATENCY_S + (v * v) / (2.0 * decel_max()) + STOP_MARGIN
    if gap < need then
      trigger(2, string.format("obstacle %.1fm ahead at %.1fm/s", gap, v), MAV_SEVERITY_WARNING, gap, v)
    end
  end
  return update, PERIOD_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "LidarGuard: started")
return update, 1000
