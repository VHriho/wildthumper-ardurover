--[[
  net_launchers.lua - керування 6 сіткометами WildThumper XL (ArduRover)

  Сіткомети на сервовиходах з функціями Script1..Script6 (94..99):
    SERVO9..SERVO14_FUNCTION = 94..99
    #1..#3 - лівий борт (спереду назад), #4..#6 - правий борт.
  Постріл = імпульс NETS_PULSE секунд PWM 2000, у спокої 1000.

  Як надходить команда "відстріл":
   1. Aux-функція Scripting1 (300) переходить у HIGH:
        - тумблер на пульті (RCx_OPTION 300), або
        - команда MAV_CMD_DO_AUX_FUNCTION 300 2 з GCS (MAVProxy / Mission Planner), або
        - пункт місії DO_AUX_FUNCTION 300 2.
      Стріляє наступний ще не використаний сіткомет.
      Щоб вистрілити знову, aux має повернутися в LOW (0) і знову в HIGH (2).
   2. Пункт місії DO_SEND_SCRIPT_MESSAGE: param1 = 7301 (ідентифікатор),
      param2 = номер сіткомета 1..6 (0 = наступний).

  Блокування (постріл не відбувається, у GCS - причина):
    - NETS_ENABLE = 0
    - ровер не озброєний
    - швидкість > NETS_MAXSPD
    - сіткомет уже використано / всі використані
  NETS_RESET = 1 -> усі сіткомети знову "заряджені" (після перезарядки), сам скидається в 0.
--]]

local PARAM_TABLE_KEY = 74
assert(param:add_table(PARAM_TABLE_KEY, "NETS_", 4), "NETS: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1),   "NETS: ENABLE")
assert(param:add_param(PARAM_TABLE_KEY, 2, "MAXSPD", 3.0), "NETS: MAXSPD")  -- м/с
assert(param:add_param(PARAM_TABLE_KEY, 3, "PULSE",  1.0), "NETS: PULSE")   -- с
assert(param:add_param(PARAM_TABLE_KEY, 4, "RESET",  0),   "NETS: RESET")

local P_ENABLE = Parameter("NETS_ENABLE")
local P_MAXSPD = Parameter("NETS_MAXSPD")
local P_PULSE  = Parameter("NETS_PULSE")
local P_RESET  = Parameter("NETS_RESET")

local N = 6
local FUNC_BASE = 94            -- Script1
local PWM_SAFE, PWM_FIRE = 1000, 2000
local AUX_FIRE = 300            -- Scripting1
local AUX_HIGH = 2
local MSG_ID = 7301             -- param1 для DO_SEND_SCRIPT_MESSAGE
local PERIOD_MS = 50
local SEV_WARNING, SEV_INFO = 4, 6
local NAMES = { "L1", "L2", "L3", "R1", "R2", "R3" }

local used = { false, false, false, false, false, false }
local pulse_end = {}            -- [i] = час завершення імпульсу (с)
local last_aux = nil

local function now_s() return millis():tofloat() * 0.001 end

local function remaining()
  local n = 0
  for i = 1, N do if not used[i] then n = n + 1 end end
  return n
end

local function fire(idx)
  if P_ENABLE:get() < 1 then gcs:send_text(SEV_WARNING, "NETS: blocked - disabled"); return end
  if not arming:is_armed() then gcs:send_text(SEV_WARNING, "NETS: blocked - disarmed"); return end
  local v = ahrs:groundspeed_vector():length()
  if v > P_MAXSPD:get() then
    gcs:send_text(SEV_WARNING, string.format("NETS: blocked - speed %.1f > %.1f m/s", v, P_MAXSPD:get()))
    return
  end
  if idx == 0 then
    for i = 1, N do if not used[i] then idx = i; break end end
    if idx == 0 then gcs:send_text(SEV_WARNING, "NETS: blocked - all used"); return end
  end
  if idx < 1 or idx > N then gcs:send_text(SEV_WARNING, "NETS: bad launcher " .. tostring(idx)); return end
  if used[idx] then gcs:send_text(SEV_WARNING, "NETS: #" .. idx .. " already used"); return end

  used[idx] = true
  pulse_end[idx] = now_s() + P_PULSE:get()
  SRV_Channels:set_output_pwm(FUNC_BASE + idx - 1, PWM_FIRE)
  gcs:send_text(SEV_INFO, string.format("NETS: fired #%d %s (%d left)", idx, NAMES[idx], remaining()))
  logger:write("NETS", "Idx,Spd,Left", "Bfb", idx, v, remaining())
end

local function update()
  local t = now_s()

  -- 1. завершення імпульсів, вихід у спокої тримаємо явно
  for i = 1, N do
    if pulse_end[i] ~= nil and t >= pulse_end[i] then pulse_end[i] = nil end
    SRV_Channels:set_output_pwm(FUNC_BASE + i - 1, pulse_end[i] and PWM_FIRE or PWM_SAFE)
  end

  -- 2. скидання (перезарядка)
  if P_RESET:get() > 0 then
    for i = 1, N do used[i] = false end
    P_RESET:set(0)
    gcs:send_text(SEV_INFO, "NETS: reloaded, 6 ready")
  end

  -- 3. команда через aux Scripting1: спрацьовує на переході в HIGH
  local aux = rc:get_aux_cached(AUX_FIRE)
  if aux ~= nil then
    if aux == AUX_HIGH and last_aux ~= AUX_HIGH then fire(0) end
    last_aux = aux
  end

  -- 4. команда з місії: DO_SEND_SCRIPT_MESSAGE (param1 = 7301, param2 = номер)
  local time_ms, p1, p2 = mission_receive()
  if time_ms ~= nil and p1 == MSG_ID then
    fire(math.floor((p2 or 0) + 0.5))
  end

  return update, PERIOD_MS
end

for i = 1, N do SRV_Channels:set_output_pwm(FUNC_BASE + i - 1, PWM_SAFE) end
gcs:send_text(SEV_INFO, "NETS: 6 launchers ready")
return update, 1000
