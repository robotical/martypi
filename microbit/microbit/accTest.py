# test script to display the accelerometer x and y on the LEDs
import microbit

def accToPixel(acc_x, acc_y):
	px = 5*(acc_x - min_x)/range_x
	px = int(min(max(0, px), 4))
	py = 5*(acc_y - min_y)/range_y
	py = int(min(max(0, py), 4))

	return [px, py]

def accToUnsignedBytes(acc_x, acc_y, acc_z):
	acc_x /= 16
	acc_y /= 16
	acc_z /= 16
	bx = acc_x + 128
	bx = int(min(max(0, bx), 255))
	by = acc_y + 128
	by = int(min(max(0, by), 255))
	bz = acc_z + 128
	bz = int(min(max(0, bz), 255))

	return [bx, by, bz]

# inspired by/borrowed from the simple slalom script
min_x = -16
max_x = 16
range_x = max_x - min_x

min_y = -16
max_y = 16
range_y = max_y - min_y

im = microbit.Image('00000:'*5)
update_pixel = False

ax = microbit.accelerometer.get_x()
ay = microbit.accelerometer.get_y()
az = microbit.accelerometer.get_z()

microbit.display.show(im)
px, py = 2, 2

uax, uay, uaz = accToUnsignedBytes(ax, ay, az)

filtersize = 4
axH = filtersize*[uax]
axHi = 0
filteredAX = uax
ayH = filtersize*[uay]
ayHi = 0
filteredAY = uay
azH = filtersize*[uaz]
azHi = 0
filteredAZ = uaz
#for x in range(10):
while True:
	uax, uay, uaz = accToUnsignedBytes(ax, ay, az)
	filteredAX = filteredAX - (axH[axHi] - uax)/filtersize
	axH[axHi] = uax
	axHi = (axHi+1)%filtersize
	filteredAY = filteredAY - (ayH[ayHi] - uay)/filtersize
	ayH[ayHi] = uay
	ayHi = (ayHi+1)%filtersize
	filteredAZ = filteredAZ - (azH[azHi] - uaz)/filtersize
	azH[azHi] = uaz
	azHi = (azHi+1)%filtersize

	microbit.uart.write(bytearray([int(filteredAX), int(filteredAY), int(microbit.button_a.is_pressed()), int(microbit.button_b.is_pressed()), int(filteredAZ)]))
	microbit.display.set_pixel(px, py, 0)
	px, py = accToPixel(int(filteredAX-128), int(filteredAY-128))
	microbit.display.set_pixel(px, py, 5)

	#print(ax/16, ay/16)
	'''
	while not microbit.uart.any():
		microbit.sleep(50)

	microbit.uart.readall()
	'''

	microbit.sleep(10)

	ax = microbit.accelerometer.get_x()
	ay = microbit.accelerometer.get_y()
	az = microbit.accelerometer.get_z()


