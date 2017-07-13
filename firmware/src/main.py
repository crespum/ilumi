from machine import I2C, Pin, Timer


class PCA9685():

    def __init__(self, scl, sda, oe):
        self.i2c = I2C(scl=Pin(scl), sda=Pin(sda))

        pin_oe = Pin(oe, Pin.OUT)
        pin_oe.off()

        self.i2c.writeto_mem(64, 0, b'\x01') # Leave sleep mode

    def _led_in_range(self, led):
        return led in range(0, 15)

    def _read_memory(self, start_addr, length):
        return self.i2c.readfrom_mem(64, start_addr, length)

    def _write_memory(self, start_addr, content):
        return self.i2c.writeto_mem(64, start_addr, content)

    def turn_led_on(self, led):
        self._write_memory(led*4+6, b'\x00')
        self._write_memory(led*4+7, b'\x00')
        self._write_memory(led*4+8, b'\x00')
        self._write_memory(led*4+9, b'\x10')

    def turn_led_off(self, led):
        self._write_memory(led*4+6, b'\x00')
        self._write_memory(led*4+7, b'\x10')
        self._write_memory(led*4+8, b'\x00')
        self._write_memory(led*4+9, b'\x00')

    def turn_all_leds_on(self):
        self._write_memory(250, b'\x00')
        self._write_memory(251, b'\x00')
        self._write_memory(252, b'\x00')
        self._write_memory(253, b'\x10')

    def turn_all_leds_off(self):
        self._write_memory(250, b'\x00')
        self._write_memory(251, b'\x10')
        self._write_memory(252, b'\x00')
        self._write_memory(253, b'\x00')

    def set_all_leds_brightness(self, brightness):
        # Validate input
        pwm_off_time = int((100-brightness) * 4095/100)
        pwm_off_time_bytes = pwm_off_time.to_bytes(2, 'little')

        self._write_memory(250, b'\x00') # Delay = 0
        self._write_memory(251, b'\x00')
        self._write_memory(252, pwm_off_time_bytes[0].to_bytes(1, 'little'))
        self._write_memory(253, pwm_off_time_bytes[1].to_bytes(1, 'little'))


class Ilumi():
    def __init__(self):
        self.leds = PCA9685(scl=2, sda=0, oe=4)
        self.leds.turn_all_leds_off()
        self.timer = Timer(-1)

    def _blink_on_cb(self, timer):
        self.leds.turn_all_leds_on()
        self.timer.init(period=self.blink_period_ms, mode=Timer.ONE_SHOT, callback=self._blink_off_cb)

    def _blink_off_cb(self, timer):
        self.leds.turn_all_leds_off()
        self.timer.init(period=self.blink_period_ms, mode=Timer.ONE_SHOT, callback=self._blink_on_cb)

    def blink_start(self, period_ms):
        self.blink_period_ms = period_ms
        self.timer.init(period=period_ms, mode=Timer.ONE_SHOT, callback=self._blink_on_cb)

    def blink_stop(self):
        self.timer.deinit()


    def _fade_in_cb(self, timer):
        if self.fade_current_brightness == self.fade_max:
            self.timer.init(period=20, mode=Timer.PERIODIC, callback=self._fade_out_cb)
        else:
            self.fade_current_brightness = self.fade_current_brightness + 1
            self.leds.set_all_leds_brightness(self.fade_current_brightness)

    def _fade_out_cb(self, timer):
        if self.fade_current_brightness == self.fade_min:
            self.timer.init(period=20, mode=Timer.PERIODIC, callback=self._fade_in_cb)
        else:
            self.fade_current_brightness = self.fade_current_brightness - 1
            self.leds.set_all_leds_brightness(self.fade_current_brightness)

    def fade(self, max_brightness=100, min_brightness=0):
        self.fade_max = max_brightness
        self.fade_min = min_brightness
        self.leds.turn_all_leds_off()
        self.fade_current_brightness = min_brightness
        self.leds.set_all_leds_brightness(min_brightness)
        self.timer.init(period=20, mode=Timer.PERIODIC, callback=self._fade_in_cb)

    def fade_stop(self):
        self.timer.deinit()


ilumi = Ilumi()
ilumi.fade(max_brightness=80, min_brightness=20)
# ilumi.blink_start(500)
