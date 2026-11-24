--[[
  test_net_launchers.lua - офлайн-тест net_launchers.lua (mock API ArduPilot).
  Запуск: lua5.3 test_net_launchers.lua
--]]
local T, fn = 0, nil
local S, P, pwm, texts, msgq = {}, {}, {}, {}, {}

local U32 = {}; U32.__index = U32
function U32:tofloat() return self.v end
function millis() return setmetatable({ v = T }, U32) end
local tables = {}
param = {
  add_table = function(_, key, prefix) tables[key] = prefix; return true end,
  add_param = function(_, key, _, name, default) P[tables[key] .. name] = default; return true end,
}
function Parameter(name)
  return { get = function() return P[name] end, set = function(_, v) P[name] = v; return true end }
end
arming = { is_armed = function() return S.armed end }
ahrs   = { groundspeed_vector = function() return { length = function() return S.speed end } end }
gcs    = { send_text = function(_, _, t) texts[#texts + 1] = t end }
logger = { write = function() end }
rc     = { get_aux_cached = function(_, fnum) if fnum == 300 then return S.aux end end }  -- S.aux = nil: ще не задавався
SRV_Channels = { set_output_pwm = function(_, f, v) pwm[f] = v end }
function mission_receive()
  local m = table.remove(msgq, 1)
  if m then return setmetatable({ v = T }, U32), m[1], m[2], 0, 0 end
end

local chunk = assert(loadfile("net_launchers.lua"))
local function reset(st)
  S = { armed = true, speed = 0.0, aux = 0 }
  for k, v in pairs(st or {}) do S[k] = v end
  pwm, texts, msgq, fn = {}, {}, {}, nil
  fn = select(1, chunk())
end
local function run(seconds)
  local stop = T + seconds * 1000
  while T < stop do local d; fn, d = fn(); T = T + d end
end
local function fired() local n = 0; for f = 94, 99 do if pwm[f] == 2000 then n = n + 1 end end; return n end
local function last() return texts[#texts] or "" end

local passed, failed = 0, 0
local function check(name, cond)
  if cond then passed = passed + 1; print("PASS  " .. name)
  else failed = failed + 1; print("FAIL  " .. name .. "  | " .. table.concat(texts, " ; ")) end
end

-- 1. старт: усі виходи в спокої
reset(); run(1.1)
check("старт: 6 виходів = 1000", pwm[94] == 1000 and pwm[99] == 1000 and fired() == 0)

-- 2. aux LOW -> HIGH: стріляє #1 (L1) імпульсом 1 с
S.aux = 2; run(0.1)
check("aux HIGH -> постріл #1", pwm[94] == 2000 and last():find("fired #1") ~= nil)
run(1.0)
check("імпульс завершився через 1 с", pwm[94] == 1000)

-- 3. aux тримається HIGH - повторного пострілу немає
run(2); check("aux тримається HIGH - без повтору", last():find("fired #1") ~= nil and fired() == 0)

-- 4. LOW -> HIGH ще раз: стріляє наступний #2
S.aux = 0; run(0.2); S.aux = 2; run(0.1)
check("повторний перехід -> постріл #2", pwm[95] == 2000)

-- 5. команда з місії: конкретний #5
run(1.2); msgq[#msgq + 1] = { 7301, 5 }; run(0.1)
check("DO_SEND_SCRIPT_MESSAGE 7301, 5 -> постріл #5 R2", pwm[98] == 2000 and last():find("R2") ~= nil)

-- 6. той самий #5 ще раз -> блок "already used"
run(1.2); msgq[#msgq + 1] = { 7301, 5 }; run(0.1)
check("#5 повторно -> already used", last():find("already used") ~= nil and fired() == 0)

-- 7. чужий ідентифікатор повідомлення ігнорується
msgq[#msgq + 1] = { 1234, 3 }; run(0.1)
check("чужий param1 -> ігнор", pwm[96] == 1000)

-- 8. блок за швидкістю
S.speed = 5.0; S.aux = 0; run(0.2); S.aux = 2; run(0.1)
check("швидкість 5 > 3 м/с -> blocked", last():find("speed") ~= nil and fired() == 0)

-- 9. блок, якщо роззброєний
S.speed = 0; S.armed = false; S.aux = 0; run(0.2); S.aux = 2; run(0.1)
check("роззброєний -> blocked", last():find("disarmed") ~= nil and fired() == 0)

-- 10. вистріляти всі -> "all used"; NETS_RESET -> знову 6
S.armed = true
for _ = 1, 4 do S.aux = 0; run(0.2); S.aux = 2; run(1.2) end
S.aux = 0; run(0.2); S.aux = 2; run(0.1)
check("усі використані -> all used", last():find("all used") ~= nil)
P["NETS_RESET"] = 1; run(0.1)
check("NETS_RESET -> перезарядка, параметр скинуто", last():find("reloaded") ~= nil and P["NETS_RESET"] == 0)

-- 11. перша команда після завантаження, коли aux ще ніколи не задавався (nil -> HIGH)
reset({ aux = nil }); run(1.1); S.aux = 2; run(0.1)
check("перша команда після старту (aux nil -> HIGH) -> постріл", pwm[94] == 2000)

-- 12. NETS_ENABLE 0 -> blocked
run(1.2)
P["NETS_ENABLE"] = 0; S.aux = 0; run(0.2); S.aux = 2; run(0.1)
check("NETS_ENABLE 0 -> blocked", last():find("disabled") ~= nil and fired() == 0)

print(string.format("\n%d passed, %d failed", passed, failed))
os.exit(failed == 0 and 0 or 1)
