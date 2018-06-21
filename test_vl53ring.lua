v=require('vl53ring')
assert(v.init())

-- faster, less precise measuremente
v.set_measurement_timing_budget(20000);

ms = ms or 1000  -- period of distance measurements

-- the callback will be called with all sensor readings
local dist_callback= function(d1, d2)
  print('dist:', d1, d2)
end

-- start monitoring distances
v.get_continuous(ms, f)

-- run for 60 seconds
tmr.sleepms(60*1000)

-- stop monitoring distances
v.get_continuous(false)
