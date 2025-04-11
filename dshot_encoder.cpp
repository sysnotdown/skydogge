#include "dshot_encoder.h"

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "pico_pio_loader.h"

#include "dshot_encoder.pio.h"

bool DShotEncoder::init(uint8_t pinNum, PIO pio) {

  _pio=pio;
  _dshot_gpio=pinNum;

  _pio_sm = pio_claim_unused_sm(_pio, /*required=*/false);
  if (_pio_sm < 0) {
    return false;
  }

  if (!pio_loader_add_or_get_offset(_pio, &dshot_encoder_program, &_pio_offset)) {
    pio_sm_unclaim(_pio, _pio_sm);
    _pio_sm = -1;
    return false;
  }

  
  dshot_encoder_program_init(_pio, _pio_sm, _pio_offset, _dshot_gpio);
  return true;
}

void DShotEncoder::setCommand(uint16_t c) {
  // Shift for telemetry bit (0)
  c = c << 1;

  // Shift and include checksum
  uint16_t checksum = (c ^ (c >> 4) ^ (c >> 8)) & 0x0F;
  c = (c << 4) | checksum;

  pio_sm_put_blocking(_pio, _pio_sm, c);
  pio_sm_set_enabled(_pio, _pio_sm, true);
}

//油门分辨率是2000，比Pwm可能更细微。
void DShotEncoder::setThrottle(float t) {
  if (t < 0) t = 0;
  if (t > 1) t = 1;

  uint16_t c = MIN_THROTTLE_COMMAND + t * (MAX_THROTTLE_COMMAND - MIN_THROTTLE_COMMAND);
  if (c < MIN_THROTTLE_COMMAND) c = MIN_THROTTLE_COMMAND;
  if (c > MAX_THROTTLE_COMMAND) c = MAX_THROTTLE_COMMAND;
  setCommand(c);
  _throttle=t;
}

void DShotEncoder::stop() {
  pio_sm_set_enabled(_pio, _pio_sm, false);
}

float DShotEncoder::GetThrottle()
{
  return _throttle;
}

void DShotEncoder::SpinNormal()
{
  for(int i=0;i<6;i++)
  {
    setCommand(20);
  }
}

void DShotEncoder::SpinReverse()
{
  for(int i=0;i<6;i++)
  {
    setCommand(21);
  }
}

void DShotEncoder::SaveConfig()
{
  for(int i=0;i<6;i++)
  {
    setCommand(12);
  }
  sleep_ms(35);
}

void DShotEncoder::BeepTest()
{
  setCommand(1);
}