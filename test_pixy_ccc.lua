pixy = require 'pixy2'
pixy.init()


pccc = function(blocks)
for i, b in ipairs(blocks) do
print('block '..i..' '..b.signature..' ('..b.x..','..b.y..') '..b.width..'x'..b.height..' '..b.angle..' '..b.index..' '..b.age)
end
end

pixy.ccc.set_blocks_callback(pccc)
pixy.ccc.enable(100)

