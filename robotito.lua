--dofile('robotito.lua')

-- { {xshutpin, [newadddr]}, ... }
local sensors = {
 {16},
 {17},
 {4},
 {14},
 {12},
 {13},
}
local omni_conf = {5.0, 27,26, 33,25, 23,18}

local ledpin = pio.GPIO19
local n_pins = 24

local max_bright = 70
local colors = {
 {max_bright, 0, 0},
 {max_bright/2, max_bright/2, 0},
 {0, max_bright, 0},
 {0, max_bright/2, max_bright/2},
 {0, 0, max_bright},
 {max_bright/2, 0, max_bright/2},
}

local first_led = {0, 20, 16, 12, 8, 4}

neo = neo or neopixel.attach(neopixel.WS2812B, ledpin, n_pins)

local ms = ms or 100  -- period of distance measurements

local enabled = false

local vlring=require('vl53ring')
assert(vlring.init(sensors))
-- faster, less precise measuremente
vlring.set_measurement_timing_budget(20000);


local omni=require('omni_hbridge')
assert(omni.init(table.unpack(omni_conf)))

button = sensor.attach("PUSH_SWITCH", pio.GPIO0)


local sin60 = math.sqrt(3)/2
local dmin = 50
local dmax = 400
local norm_x = 100 / math.sqrt(3)
local norm_y = 100 / 2
local d_range = dmax - dmin
local k_cuad = -1 / d_range^2

local floor = math.floor

local vel_lineal = function(dist)

  local vel = (dmax-dist) / (d_range)
  return vel

end

local vel_cuadratic = function(dist)

  local vel = k_cuad*(dist - dmin)^2 + 1
  return vel

end

local vel_from_dist = vel_cuadratic

-- the callback will be called with all sensor readings
local dist_callback= function(d1, d2, d3, d4, d5, d6)
  --print('dist:', d1, d2, d3, d4, d5, d6)
  local d={d1, d2, d3, d4, d5, d6}

  -- print('dist:', d1, d2, d3, d4, d5, d6)


  for i=1,6 do
    if d[i]<dmin or d[i]>dmax then
      d[i]=0

      for j = 0, 3 do
          neo:setPixel(first_led[i]+j, 0,0,0)
      end
      neo:update()

    else
      --d[i]=dmax-d[i]
      d[i] = vel_from_dist(d[i])   -- 0..1

      local color = {
        floor(d[i]*colors[i][1]),
        floor(d[i]*colors[i][2]),
        floor(d[i]*colors[i][3])
      }
      for j = 0, 3 do
        neo:setPixel(first_led[i]+j, table.unpack(color))
      end
      neo:update()

    end
  end

  local xdot = ( (-d[3]-d[2]+d[6]+d[5])*sin60 ) *norm_x
  local ydot = ( (-d[3]-d[5]+d[2]+d[6])/2 + d[4]-d[1] ) *norm_y

  print(xdot, ydot, '', 'dist:', table.unpack(d))
  omni.drive(xdot,ydot,0)

end

local function button_callback(data)
  if data.on==0 then return end
  if enabled then
    print("off")
    omni.set_enable(false)
    -- stop monitoring distances
    vlring.get_continuous(false)
    enabled = false
  elseif not enabled then
    print("on")
    omni.set_enable()
    -- start monitoring distances
    vlring.get_continuous(ms, dist_callback)
    enabled = true
  end
end

button:callback(button_callback)

print("on")
omni.set_enable()
vlring.get_continuous(ms, dist_callback)
-- local readings = {6}
-- while true do
--     for i= 1, #sensors do
--         readings[i] = vlring.get(i)
--     end
--     print (table.unpack(d))
--     tmr.sleepms(100*1000)
-- end




--[[
tmr.sleepms(20*1000)

print("off")
vlring.get_continuous(false)
omni.set_enable(false)
--vlring.release()
--]]
