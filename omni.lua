--WHEEL_DIAMETER   = 0.038   --m
--WHEEL_PERIMETER = WHEEL_DIAMETER*3.141592654
local ENC_CPR = 12  --counts per revolution
local MOTOR_REDUCTION = 50
local TICS_PER_REVOLUTION = ENC_CPR*MOTOR_REDUCTION
local RAD_PER_TICK = 2*math.pi / TICS_PER_REVOLUTION

-- forward feed parameter
-- at 90% does about 1080 tics/s
local KF = 90/1080     --ENC_CPR*90/1080

local omni = require('omni_hbridge')

local KP = 0.1     --0.1/ENC_CPR

-- initialize with tobot radius and drivers' pins
omni.init(5.0, 27,26,39,37, 33,25,38,36, 23,18,34,35)
omni.set_pid(KP, 0.0, 0.0, KF)


return omni


