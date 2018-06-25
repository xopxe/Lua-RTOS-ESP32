local ROBOT_RADIUS = 7.0 --units?
local DIST_PERIOD  = 50  --ms

local drive =require('omni')
assert(drive.init(ROBOT_RADIUS))

local dist = require('vl53ring')
assert(dist.init())
dist.set_measurement_timing_budget(20000);


local d1, d2, d3, d4, d5, d6 = 0,0,0,0,0,0

local dist_callback= function(_d1, _d2, _d3, _d4, _d5, _d6)
  d1, d2, d3, d4, d5, d6 = _d1, _d2, _d3, _d4, _d5, _d6
end


drive.set_enable(true)
dist.get_continuous(DIST_PERIOD, dist_callback)

local sleepdrive=1000
for i=1, 10 do
  drive.drive(0,70,0)
  tmr.sleepms(sleepdrive)
  drive.drive(0,-70,0)
  tmr.sleepms(sleepdrive);
end


dist.get_continuous(false)
drive.set_enable(false)
