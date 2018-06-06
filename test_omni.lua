m=require('omni')
m.init(5.0)
m.set_enable()

--for v=0, 50 do m.drive(0,v,0); tmr.delayms(100) end; m.drive(0.0,0.0,0.0)
--for v=0, -50, -1 do m.drive(0,v,0); tmr.delayms(100) end; m.drive(0.0,0.0,0.0)

local d=1000
for i=1, 10 do  
  m.drive(0,70,0)
  tmr.sleepms(d)
  m.drive(0,-70,0)
  tmr.sleepms(d);   
end
m.set_enable(false)
