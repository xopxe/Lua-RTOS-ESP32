v=require('vl53ring')
assert(v.init())

v.set_measurement_timing_budget(20000);

ms = ms or 1000

f= function(...) 
  print(...) 
end
v.get_continuous(ms, f)


