#ifndef __PICO_DSHOT_DSHOT_ENCODER_H__
#define __PICO_DSHOT_DSHOT_ENCODER_H__

#include <stdint.h>

//这张表格是betaflight那边得到的命令码。dshot的特殊命令码。
// 0  DIGITAL_CMD_MOTOR_STOP 		      // Currently not implemented
// 1  DIGITAL_CMD_BEEP1 			      // Wait at least length of beep (260ms) before next command
// 2  DIGITAL_CMD_BEEP2 			      // Wait at least length of beep (260ms) before next command
// 3  DIGITAL_CMD_BEEP3 			      // Wait at least length of beep (280ms) before next command
// 4  DIGITAL_CMD_BEEP4 			      // Wait at least length of beep (280ms) before next command
// 5  DIGITAL_CMD_BEEP5 			      // Wait at least length of beep (1020ms) before next command
// 6  DIGITAL_CMD_ESC_INFO  		      // Wait at least 12ms before next command
// 7  DIGITAL_CMD_SPIN_DIRECTION_1 	      // Need 6x, no wait required
// 8  DIGITAL_CMD_SPIN_DIRECTION_2 	      // Need 6x, no wait required
// 9  DIGITAL_CMD_3D_MODE_OFF 		      // Need 6x, no wait required
// 10 DIGITAL_CMD_3D_MODE_ON  		      // Need 6x, no wait required
// 11 DIGITAL_CMD_SETTINGS_REQUEST  	      // Currently not implemented
// 12 DIGITAL_CMD_SAVE_SETTINGS  		      // Need 6x, wait at least 35ms before next command
// 20 DIGITAL_CMD_SPIN_DIRECTION_NORMAL 	      // Need 6x, no wait required
// 21 DIGITAL_CMD_SPIN_DIRECTION_REVERSED 	      // Need 6x, no wait required
// 22 DIGITAL_CMD_LED0_ON 			      // No wait required
// 23 DIGITAL_CMD_LED1_ON 			      // No wait required
// 24 DIGITAL_CMD_LED2_ON 			      // No wait required
// 25 DIGITAL_CMD_LED3_ON 			      // No wait required
// 26 DIGITAL_CMD_LED0_OFF 		      // No wait required
// 27 DIGITAL_CMD_LED1_OFF 		      // No wait required
// 28 DIGITAL_CMD_LED2_OFF 		      // No wait required
// 29 DIGITAL_CMD_LED3_OFF 		      // No wait required
// 30 Audio_Stream mode on/Off		      // Currently not implemented
// 31 Silent Mode on/Off			      // Currently not implemented
// 32 DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_DISABLE                  // Need 6x, no wait required. Disables commands 42 to 47
// 33 DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_ENABLE                   // Need 6x, no wait required. Enables commands 42 to 47
// 34 DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY          // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm if normal Dshot frame
// 35 DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY   // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm period if normal Dshot frame
// Commands above are only executed when motors are stopped
// 36 // Not yet assigned 
// 37 // Not yet assigned 
// 38 // Not yet assigned 
// 39 // Not yet assigned 
// 40 // Not yet assigned 
// 41 // Not yet assigned 
// Commands below are executed at any time
// 42 DIGITAL_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY      // No wait required 
// 43 DIGITAL_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY          // No wait required 
// 44 DIGITAL_CMD_SIGNAL_LINE_CURRENT_TELEMETRY          // No wait required 
// 45 DIGITAL_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY      // No wait required 
// 46 DIGITAL_CMD_SIGNAL_LINE_ERPM_TELEMETRY             // No wait required 
// 47 DIGITAL_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY      // No wait required 




#include "hardware/pio.h"

class DShotEncoder {
 public:
  // DShotEncoder(uint dshot_gpio, PIO pio = pio0)
  //   : dshot_gpio(dshot_gpio), pio(pio) {}
  // TODO: cleanup in destructor

  // Init PIO, but do not output data yet
  bool init(uint8_t pinNum, PIO pio=pio0);

  // Set methods begin continuously generating output, repeating the last value
  void setCommand(uint16_t c); // Set the DShot command value
  void setThrottle(float t);  // Set the throttle in range [0, 1]

  // Stop generating output (until next set command)
  void stop();
  float GetThrottle();

  void SpinNormal();
  void SpinReverse();
  void SaveConfig();
  void BeepTest();
  
 private:
  static constexpr uint16_t MIN_THROTTLE_COMMAND = 48;
  static constexpr uint16_t MAX_THROTTLE_COMMAND = 2047;

  uint _dshot_gpio;

  PIO _pio;
  uint _pio_offset;
  int _pio_sm = -1;
  float _throttle=0;
};

#endif
