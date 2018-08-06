
local ledpin = pio.GPIO19
local n_pins = 24

local colors = {
 {50, 0, 0},
 {0, 50, 0},
 {0, 0, 50},
 {50, 50, 0},
 {0, 50, 50},
 {50, 0, 50},
}

neo = neo or neopixel.attach(neopixel.WS2812B, ledpin, n_pins)

local pixel = 0
local direction = 0

local cant_vueltas = 2

while cant_vueltas ~= 0 do
    neo:setPixel(pixel, 0, 50, 0)
    neo:update()
    tmr.delayms(100)
    neo:setPixel(pixel, 0, 0, 0)

    if (direction == 0) then
      if (pixel == 23) then
        direction = 1
        pixel = 22
      else
        pixel = pixel + 1
      end
    else
      if (pixel == 0) then
        direction = 0
        pixel = 1
        print('Fin vuelta ', cant_vueltas)
        cant_vueltas = cant_vueltas -1
      else
        pixel = pixel - 1
      end
    end
end


-- shutdown
local contador = 0
for j = 0, 5 do
    for i = 0, 3 do
        neo:setPixel(contador, table.unpack(colors[j+1]))
        contador = contador + 1
    end
end
neo:update()
