  #include <Arduino.h>
  #include <SimpleFOC.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  #include <Wire.h>
  #include <math.h>
  #include <stdio.h>
  #include <string.h>
  #include <stdbool.h>
  #include <ICM_20948.h>
  #include <stm32f4xx_hal.h>
  #include "encoders/mt6835/MagneticSensorMT6835.h"

  #define MOSI1 PA7 //black
  #define MISO1 PA6 //yellow
  #define SCLK1 PA5 //red
  #define CS1 PA4 //white

  extern "C" void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 24;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
      Error_Handler();
    }
  }

  //Create Sensor Object
  SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
  MagneticSensorMT6835 sensor = MagneticSensorMT6835(CS1, myMT6835SPISettings);
  SPIClass SPI_1(MOSI1, MISO1, SCLK1);

  //Create Motor Object
  BLDCMotor motor = BLDCMotor(11);

  //Create Driver Object
  BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PC9);

  //Create commander object
  Commander command = Commander(Serial);

  //Assign variables
  float target_angle = 0;

  //camera data
  float pixel = 0.0f;
  float v_ff = 0.0f;

  //gains
  const float alpha = 0.9;
  const float pid_Kp = 0.03;
  const float pid_Ki = 0.03;
  const float pid_Kd = 0.000;

  //control variables
  float pid_output = 0;
  float derivative = 0;
  float pid_integral = 0;
  float last_error = 0;
  float error = 0;

  //deadzone
  unsigned long watchdog = 0; 
  unsigned long last_time = 0;

  //Assign commands
  void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
  void doMotor(char* cmd) { command.motor(&motor, cmd); }


  void setup() {

    SystemClock_Config();

    Serial.begin(115200);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
    motor.useMonitoring(Serial);
    SimpleFOCDebug::enable(&Serial);

    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

    sensor.init(&SPI_1);
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_sensor_align = 5;
    motor.modulation_centered = 1.0;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity;

    motor.PID_velocity.P = 1.1f;
    motor.PID_velocity.I = 26.0f;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.0085f;
    motor.PID_velocity.limit = 100.0;

    motor.velocity_limit = 100.0;
    motor.voltage_limit = 12.0;
    motor.current_limit = 2.0;

    motor.phase_resistance = 2.897;
    motor.monitor_downsample = 100;

    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "target angle");
    command.add('M', doMotor, "motor");

    _delay(1000);
  }


  void loop() {

    motor.loopFOC();

    if (watchdog == 0) {

      motor.voltage_limit = 0;  // when no lock
      pid_integral = 0.0f;
      last_error = 0.0f;
      v_ff = 0.0f;

      motor.move(0);
    }

    else if (pixel == 999.0f) {
      
      motor.voltage_limit = 0;  // when no lock
      pid_integral = 0.0f;
      last_error = 0.0f;
      v_ff = 0.0f;

      motor.move(0);
    } 
    
    else {

      if (watchdog > 0) watchdog--;

      motor.voltage_limit = 12.0;

      unsigned long now = micros();
      float dt = (now - last_time) * 1e-6f;
      last_time = now;

      error = pixel;

      if (error > 100.0f) error = 100.0f;
      if (error < -100.0f) error = -100.0f;

      pid_integral += error * dt;
      if (pid_integral > 2.2f) pid_integral = 2.2f;
      else if (pid_integral < -2.2f) pid_integral = -2.2f;

      derivative = (error - last_error) / dt;
      pid_output = (pid_Kp * error) + (pid_Ki * pid_integral) + (pid_Kd * derivative);
      last_error = error;

      pid_output = pid_output + (v_ff * 0.000f);

      if (pid_output > 4.0f) pid_output = 4.0f;
      if (pid_output < -4.0f) pid_output = -4.0f;

      motor.move(pid_output);
    }

    
    while (Serial.available()) {

      uint8_t b = Serial.read();

      static uint8_t i = 0;
      static uint8_t buf[8];

        if (i == 0) {
          if (b != 0xAA) continue;
          i = 1;
        } else {

          buf[i - 1] = b;
          i++;

          if (i == 9) {

            memcpy(&pixel, &buf[0], 4);
            memcpy(&v_ff, &buf[4], 4);
            
            watchdog = 500;
            i = 0;
          }
        }
    }

    //motor.monitor();
    //command.run();

  }