--[[
  test_lidar_guard.lua - офлайн-тест логіки lidar_guard.lua без ArduPilot.
  Підміняє (mock) API автопілота і проганяє сценарії. Запуск:  lua5.3 test_lidar_guard.lua
--]]

local T = 0                    -- "час" у мс
local fn
local S = {}                   -- стан підробленого автопілота
local texts = {}

local function reset(st)
  S = { armed = true, mode = 10, prx = 2, speed = 4.0, objects = {}, decel = 2.0 }
  for k, v in pairs(st or {}) do S[k] = v end
  texts = {}
  fn = nil
end

-- ---- mock API ----
local U32 = {}
U32.__index = U32
function U32:tofloat() return self.v end
function millis() return setmetatable({ v = T }, U32) end
param   = { get = function(_, n) if n == "ATC_DECEL_MAX" then return S.decel end end }
arming  = { is_armed = function() return S.armed end }
vehicle = { get_mode = function() return S.mode end,
            set_mode = function(_, m) S.mode = m; return true end }
gcs     = { send_text = function(_, sev, t) texts[#texts + 1] = t end }
logger  = { write = function() end }
ahrs    = { groundspeed_vector = function()
              return { length = function() return S.speed end } end }
proximity = {
  get_status = function() return S.prx end,
  get_object_count = function() return #S.objects end,
  get_object_angle_and_distance = function(_, i)
    local o = S.objects[i + 1]; return o[1], o[2] end,
}

-- ---- завантажуємо скрипт: він повертає (update, delay) ----
local chunk = assert(loadfile("lidar_guard.lua"))
-- новий запуск скрипта (як після reboot) і крок у часі зі збереженням стану
local function start() fn = select(1, chunk()) end
local function run(seconds)
  if fn == nil then start() end
  local stop = T + seconds * 1000
  while T < stop do
    local delay
    fn, delay = fn()
    T = T + delay
  end
end

local passed, failed = 0, 0
local function check(name, cond)
  if cond then passed = passed + 1; print("PASS  " .. name)
  else failed = failed + 1; print("FAIL  " .. name .. "   texts: " .. table.concat(texts, " | ")) end
end

-- 1. все гаразд: перешкода збоку (BendyRuler проходить повз на ~0.5 м від борту)
reset({ objects = { { 90, 1.3 }, { 270, 1.3 } } })
run(3); check("перешкоди збоку 1.3 м - без втручання", S.mode == 10)

-- 2. перешкода далеко попереду - ще не гальмуємо (4 м/с: треба ~1.2+4+0.4 = 5.6 м)
reset({ objects = { { 0, 12.0 } } })
run(2); check("перешкода попереду 12 м на 4 м/с - без втручання", S.mode == 10)

-- 3. перешкода попереду в межах гальмівного шляху -> HOLD
reset({ objects = { { 0, 5.0 } } })
run(1); check("перешкода попереду 5 м на 4 м/с -> HOLD", S.mode == 4)

-- 4. та сама перешкода, але ровер повільний (розворот на місці) -> не чіпаємо
reset({ speed = 0.1, objects = { { 0, 1.0 } } })
run(2); check("розворот на місці біля перешкоди - без втручання", S.mode == 10)

-- 5. перешкода попереду, але поза коридором (кут 40 град, 3 м: збоку 1.93 м > 0.91)
reset({ objects = { { 40, 3.0 } } })
run(2); check("перешкода поза коридором руху - без втручання", S.mode == 10)

-- 6. немає даних лідара -> HOLD через 0.5 с, не раніше
reset({ prx = 1 })
run(0.3); local early = S.mode
run(0.5); check("немає даних лідара: не раніше 0.5 с", early == 10)
check("немає даних лідара -> HOLD", S.mode == 4)

-- 7. MANUAL - сторож не втручається
reset({ mode = 0, prx = 1, objects = { { 0, 1.0 } } })
run(2); check("MANUAL - без втручання", S.mode == 0)

-- 8. роззброєний - не втручається
reset({ armed = false, prx = 1 })
run(2); check("роззброєний - без втручання", S.mode == 10)

print(string.format("\n%d passed, %d failed", passed, failed))
os.exit(failed == 0 and 0 or 1)
