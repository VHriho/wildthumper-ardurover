--[[
  test_adaptive_speed.lua - офлайн-тест adaptive_speed.lua (mock API ArduPilot).
  Запуск: lua5.3 test_adaptive_speed.lua
--]]
local T, fn = 0, nil
local S, set_calls, texts = {}, {}, {}
local P = {}

local function reset(st)
  S = { armed = true, mode = 10, prx = 2, objects = {} }
  for k, v in pairs(st or {}) do S[k] = v end
  set_calls, texts, fn = {}, {}, nil
end

local U32 = {}; U32.__index = U32
function U32:tofloat() return self.v end
function millis() return setmetatable({ v = T }, U32) end

local tables = {}
param = {
  add_table = function(_, key, prefix) tables[key] = prefix; return true end,
  add_param = function(_, key, idx, name, default) P[tables[key] .. name] = default; return true end,
}
function Parameter(name)
  return { get = function() return P[name] end }
end
arming  = { is_armed = function() return S.armed end }
vehicle = { get_mode = function() return S.mode end,
            set_desired_speed = function(_, v) set_calls[#set_calls + 1] = { T, v }; return true end }
gcs     = { send_text = function(_, _, t) texts[#texts + 1] = t end }
logger  = { write = function() end }
proximity = {
  get_status = function() return S.prx end,
  get_object_count = function() return #S.objects end,
  get_object_angle_and_distance = function(_, i) local o = S.objects[i + 1]; return o[1], o[2] end,
}

local chunk = assert(loadfile("adaptive_speed.lua"))
local function run(seconds)
  if fn == nil then fn = select(1, chunk()) end
  local stop = T + seconds * 1000
  while T < stop do local d; fn, d = fn(); T = T + d end
end
local function last_speed() local c = set_calls[#set_calls]; return c and c[2] end

local passed, failed = 0, 0
local function check(name, cond)
  if cond then passed = passed + 1; print("PASS  " .. name)
  else failed = failed + 1; print("FAIL  " .. name .. "  | calls=" .. #set_calls .. " last=" .. tostring(last_speed()) .. " | " .. table.concat(texts, " ; ")) end
end

-- 1. вільно -> спершу SLOW, через HOLD (3 с) -> FAST
reset()
run(1); check("старт: повільно", last_speed() == 3.0)
run(3); check("вільно 3+ с -> швидко 8 м/с", last_speed() == 8.0)

-- 2. перешкода попереду 15 м -> SLOW
S.objects = { { 10, 15.0 } }
run(1); check("перешкода 15 м попереду -> 3 м/с", last_speed() == 3.0)

-- 3. гістерезис: 24 м (між NEAR 20 і CLEAR 28) - лишаємось SLOW
S.objects = { { 0, 24.0 } }
run(5); check("24 м (гістерезис) -> лишаємось 3 м/с", last_speed() == 3.0)

-- 4. перешкода позаду не впливає
S.objects = { { 180, 5.0 } }
run(4); check("перешкода позаду -> 8 м/с", last_speed() == 8.0)

-- 5. не частіше ніж раз на 0.5 с і лише при зміні рівня
local n = #set_calls
run(5); check("без змін - без зайвих викликів", #set_calls == n)

-- 6. перешкода то з'являється, то зникає швидко - не "смикаємо" FAST
reset(); run(4)                                   -- вже FAST
S.objects = { { 0, 10.0 } }; run(0.4)             -- SLOW
for _ = 1, 5 do S.objects = {}; run(0.6); S.objects = { { 0, 10.0 } }; run(0.4) end
check("мерехтлива перешкода -> не повертаємось у FAST", last_speed() == 3.0)

-- 7. немає даних лідара -> SLOW
reset(); run(4); S.prx = 1; run(1)
check("немає даних лідара -> 3 м/с", last_speed() == 3.0)

-- 8. ASPD_ENABLE 0 / MANUAL / роззброєний -> жодних викликів
reset(); fn = select(1, chunk()); P["ASPD_ENABLE"] = 0; run(3); check("ASPD_ENABLE 0 -> не втручається", #set_calls == 0)
P["ASPD_ENABLE"] = 1
reset({ mode = 0 }); run(3); check("MANUAL -> не втручається", #set_calls == 0)
reset({ armed = false }); run(3); check("роззброєний -> не втручається", #set_calls == 0)

print(string.format("\n%d passed, %d failed", passed, failed))
os.exit(failed == 0 and 0 or 1)
