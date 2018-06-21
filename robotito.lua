local ROBOT_RADIUS = 7.0 --units?
local DIST_PERIOD  = 50  --ms

local drive =require('omni')
assert(drive.init(ROBOT_RADIUS))

local dist = require('vl53ring')
assert(dist.init())
dist.set_measurement_timing_budget(20000);

local dist_callback= function(...)
  print('dist:', ...) 
end



drive.set_enable(true)
dist.get_continuous(DIST_PERIOD, dist_callback)

local d=1000
for i=1, 10 do
  drive.drive(0,70,0)
  tmr.sleepms(d)
  drive.drive(0,-70,0)
  tmr.sleepms(d);
end
dist.get_continuous(false)
drive.set_enable(false)
