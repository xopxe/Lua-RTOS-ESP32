v=require('apds9660')
assert(v.init())

ms = ms or 1000

f= function(...) 
  print(...) 
end
v.get_color_continuous(ms, f)


