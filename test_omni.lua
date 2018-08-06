-- dofile('test_omni.lua')

m=m or require('omni')

m.set_enable()

local d=1000

-- raw_write(v1,v2,v3)
-- v* in -90..90

-- m.drive(xdot, ydot, wdot, [phi=0])
-- *dot in -90..90  (actually less)

--[[
m.raw_write(0,0,90)
tmr.sleepms(2*d);
m.raw_write(0,0,45)
tmr.sleepms(2*d);
m.raw_write(0,0,15)
tmr.sleepms(2*d);
--]]

---[[
local v = 0.05   --m/s

--[[
for i=1, 5 do
  m.drive(v,0,0)
  tmr.sleepms(d)
  m.drive(0,-v,0)
  tmr.sleepms(d);
end
--]]

m.drive(v,0,0)
tmr.sleepms(d)
m.drive(0,v,0)
tmr.sleepms(d)
m.drive(-v,0,0)
tmr.sleepms(d)
m.drive(0,-v,0)
tmr.sleepms(d)

m.set_enable(false)

--m.set_enable();m.raw_write(0,0,45);tmr.sleepms(2000);m.set_enable(false)
