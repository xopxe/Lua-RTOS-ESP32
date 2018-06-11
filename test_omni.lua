m=require('omni')


-- initialize with tobot radius
m.init(5.0)

m.set_enable()

local d=1000

-- raw_write(v1,v2,v3)
-- v* in -90..90

-- m.drive(xdot, ydot, wdot, [phi=0])
-- *dot in -90..90  (actually less)


for i=1, 10 do  
  m.drive(0,70,0)
  tmr.sleepms(d)
  m.drive(0,-70,0)
  tmr.sleepms(d);   
end

m.set_enable(false)
