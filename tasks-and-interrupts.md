
# The execution loop
There seems to be no periodic triggering of tasks. Instead, there is a free-running loop (with maybe some delays inside) in [src/main/main.c](src/main/main.c), which directly calls function ```scheduler``` of [src/main/scheduler/scheduler.c](src/main/scheduler/scheduler.c), with header in [src/main/scheduler/scheduler.h](src/main/scheduler/scheduler.h). I guess performance trumps predictability, and that the system is both loaded enough and stable enough to trust this free-running loop plus the bookkeeping needed to enforce the various periods.

# The tasks

They are all listed in emacs [src/main/fc/tasks.c](src/main/fc/tasks.c), in the task_attributes data table,
each task defining its name, optional sub-name, check and task functions, desired period,
and priority.

Each task receives a pre-designated number (cf. "Designated initializers" in https://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html).
The identifiers, as well as TASK_COUNT, are set up in ./src/main/scheduler/scheduler.h.

Most periods are defined as frequencies using macro TASK_PERIOD_HZ:

| Frequency                     | Tasks                                                               |
| ----------------------------- | ------------------------------------------------------------------- |
| 1Hz                           | ADC_INTERNAL |
| 5Hz                           | BATTERY_ALERTS, VTXCTRL, CAMCTRL |
| 10Hz                          | SYSTEM, STACK_CHECK, DASHBOARD, RANGEFINDER |
| 20Hz                          | CMS, RCDEVICE, PINIOBOX |
| 33Hz                          | RX |
| 50Hz                          | BATTERY_CURRENT, BST_MASTER_PROCESS |
| 100Hz                         | SERIAL, ATTITUDE, BEEPER, ESC_SENSOR, SPEED_NEGOTIATION, RC_STATS |
| 250Hz                         | TRANSPONDER, TELEMETRY |
| 1000Hz                        | MAIN, ACCEL, DISPATCH |
| SLOW_VOLTAGE_TASK_FREQ_HZ     | BATTERY_VOLTAGE |
| TASK_GPS_RATE                 | GPS |
| TASK_GPS_RESCUE_RATE_HZ       | GPS_RESCUE |
| ALTHOLD_TASK_RATE_HZ          | ALTHOLD |
| TASK_COMPASS_RATE_HZ          | COMPASS |
| TASK_BARO_RATE_HZ             | BARO |
| TASK_ALTITUDE_RATE_HZ         | ALTITUDE |
| OSD_FRAMERATE_DEFAULT_HZ      | OSD |
| TASK_LEDSTRIP_RATE_HZ         | LEDSTRIP |

But some are directly defined as periods:

| Period                        | Tasks                                                               |
| ----------------------------- | ------------------------------------------------------------------- |
| TASK_GYROPID_DESIRED_PERIOD   | GYRO, FILTER, PID |

Task priorities:

| Priority    | Tasks                                                               |
| ------------| ------------------------------------------------------------------- |
| REALTIME    | GYRO, FILTER, PID |
| HIGH        | RX, DISPATCH |
| MEDIUM_HIGH | SYSTEM, MAIN |
| MEDIUM      | BATTERY_ALERTS, BATTERY_VOLTAGE, BATTERY_CURRENT, ACCEL, ATTITUDE, GPS, GPS_RESCUE, RCDEVICE |
| LOW         | SERIAL, TRANSPONDER, BEEPER, ATHOLD, COMPASS, BARO, ALTITUDE, DASHBOARD, OSD, TELEMETRY, LEDSTRIP, SENSOR, CMS, CAMCTRL, SPEED_NEGOTIATION, RC_STATS |
| LOWEST      | STACH_CHECK, BST_MASTER_PROCESS, VTXCTRL, ADC_INTERNAL, PINIOBOX, RANGEFINDER |
  

The original description:


| Task id                 | Task name          | Subtask | Check function | Task function            | Period                    | Priority    | 
| ----------------------- | ------------------ | ------ | -------------- | ------------------------- | ------------------------- | ----------- |
| TASK_SYSTEM             | SYSTEM             | LOAD   |                | taskSystemLoad            | 10Hz                      | MEDIUM_HIGH |
| TASK_MAIN               | SYSTEM             | UPDATE |                | taskMain                  | 1000Hz                    | MEDIUM_HIGH |
| TASK_SERIAL             | SERIAL             |        |                | taskHandleSerial          | 100Hz                     | LOW |
| TASK_BATTERY_ALERTS     | BATTERY_ALERTS     |        |                | taskBatteryAlerts         | 5Hz                       | MEDIUM|
| TASK_BATTERY_VOLTAGE    | BATTERY_VOLTAGE    |        |                | batteryUpdateVoltage      | SLOW_VOLTAGE_TASK_FREQ_HZ | MEDIUM|
| TASK_BATTERY_CURRENT    | BATTERY_CURRENT    |        |                | batteryUpdateCurrentMeter | 50Hz                      | MEDIUM|
| TASK_TRANSPONDER        | TRANSPONDER        |        |                | transponderUpdate         | 250Hz                     | LOW|
| TASK_STACK_CHECK        | STACKCHECK         |        |                | taskStackCheck            | 10Hz                      | LOWEST|
| TASK_GYRO               | GYRO               |        |                | taskGyroSample            | GYROPID_DESIRED_PERIOD    | REALTIME|
| TASK_FILTER             | FILTER             |        |                | taskFiltering             | GYROPID_DESIRED_PERIOD    | REALTIME|
| TASK_PID                | PID                |        |                | taskMainPidLoop           | GYROPID_DESIRED_PERIOD    | REALTIME|
| TASK_ACCEL              | ACC                |        |                | taskUpdateAccelerometer   | 1000Hz                    | MEDIUM|
| TASK_ATTITUDE           | ATTITUDE           |        |                | imuUpdateAttitude         | 100Hz                     | MEDIUM|
| TASK_RX                 | RX                 |        | rxUpdateCheck  | taskUpdateRxMain          | 33Hz                      | HIGH|
| TASK_DISPATCH           | DISPATCH           |        |                | dispatchProcess           | 1000Hz                    | HIGH|
| TASK_BEEPER             | BEEPER             |        |                | beeperUpdate              | 100Hz                     | LOW|
| TASK_GPS                | GPS                |        |                | gpsUpdate                 | TASK_GPS_RATE             | MEDIUM|
| TASK_GPS_RESCUE         | GPS_RESCUE         |        |                | taskGpsRescue             | TASK_GPS_RESCUE_RATE_HZ   | MEDIUM|
| TASK_ALTHOLD            | ALTHOLD            |        |                | updateAltHoldState        | ALTHOLD_TASK_RATE_HZ      | LOW|
| TASK_COMPASS            | COMPASS            |        |                | taskUpdateMag             | TASK_COMPASS_RATE_HZ      | LOW|
| TASK_BARO               | BARO               |        |                | taskUpdateBaro            | TASK_BARO_RATE_HZ         | LOW|
| TASK_ALTITUDE           | ALTITUDE           |        |                | taskCalculateAltitude     | TASK_ALTITUDE_RATE_HZ     | LOW|
| TASK_DASHBOARD          | DASHBOARD          |        |                | dashboardUpdate           | 10Hz                      | LOW|
| TASK_OSD                | OSD                |        | osdUpdateCheck | osdUpdate                 | OSD_FRAMERATE_DEFAULT_HZ  | LOW|
| TASK_TELEMETRY          | TELEMETRY          |        |                | taskTelemetry             | 250Hz                     | LOW|
| TASK_LEDSTRIP           | LEDSTRIP           |        |                | ledStripUpdate            | TASK_LEDSTRIP_RATE_HZ     | LOW|
| TASK_BST_MASTER_PROCESS | BST_MASTER_PROCESS |        |                | taskBstMasterProcess      | 50Hz                      | LOWEST|
| TASK_ESC_SENSOR         | ESC_SENSOR         |        |                | escSensorProcess          | 100Hz                     | LOW|
| TASK_CMS                | CMS                |        |                | cmsHandler                | 20Hz                      | LOW|
| TASK_VTXCTRL            | VTXCTRL            |        |                | vtxUpdate                 | 5Hz                       | LOWEST|
| TASK_RCDEVICE           | RCDEVICE           |        |                | rcdeviceUpdate            | 20Hz                      | MEDIUM|
| TASK_CAMCTRL            | CAMCTRL            |        |                | taskCameraControl         | 5Hz                       | LOW|
| TASK_ADC_INTERNAL       | ADCINTERNAL        |        |                | adcInternalProcess        | 1Hz                       | LOWEST|
| TASK_PINIOBOX           | PINIOBOX           |        |                | pinioBoxUpdate            | 20Hz                      | LOWEST|
| TASK_RANGEFINDER        | RANGEFINDER        |        |                | taskUpdateRangefinder     | 10Hz                      | LOWEST|
| TASK_SPEED_NEGOTIATION  | SPEED_NEGOTIATION  |        |                | speedNegotiationProcess   | 100Hz                     | LOW|
| TASK_RC_STATS           | RC_STATS           |        |                | rcStatsUpdate             | 100Hz                     | LOW|


| Task id              | Note                      |
| -------------------- | --------------------------| 
| TASK_SERIAL          | 100 Hz should be enough to flush up to 115 bytes @ 115200 baud |
| TASK_BATTERY_VOLTAGE | Freq may be updated in tasksInit |
| TASK_RX              | If event-based scheduling doesn't work| fallback to periodic scheduling |
| TASK_GPS             | Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer |



# The vector table

Base taken from betaflight/src/main/startup/stm32//startup_stm32f40xx.s

  .word  _estack                -- system stack. The memory organization is under betaflight/src/link/stm32_flash_f405.ld
  .word  Reset_Handler          -- betaflight/src/main/startup/stm32/startup_stm32f40xx.s
  .word  NMI_Handler            -- empty function
  .word  HardFault_Handler      -- non-empty, but terminates execution src/main/fc/hardfaults.c 
  .word  MemManage_Handler      -- empty function
  .word  BusFault_Handler       -- empty function
  .word  UsageFault_Handler     -- empty function
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler            -- empty function, no SVC!
  .word  DebugMon_Handler       -- empty function
  .word  0
  .word  PendSV_Handler         -- empty function
  .word  SysTick_Handler        -- non-empty (lib/main/CMSIS/Core/Include/core_cm4.h) not clear what it does.
                                   It manipulates the system timer (SysTick, control defined in betaflight/lib/main/CMSIS/Core/Include/core_cm4.h)
				   but calls no function (HAL_IncTick is not called as it is not present in the memory map).
  
  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */ empty function                                     
  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */ empty function            
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */ empty function         
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */ empty function
  .word     FLASH_IRQHandler                  /* FLASH                        */ empty function                       
  .word     RCC_IRQHandler                    /* RCC                          */ empty function                             
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask b0001
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask b0010
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask b0100
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask b1000
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask b1010
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */ src/main/drivers/dma.[hc]. DEFINE_DMA_IRQ_HANDLER is used to define the IRQ handlers
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */ --//--
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */ --//--
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */ --//--
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */ --//--
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */ --//--
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */ --//--
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */ empty function
  .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */ empty function                      
  .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */ --//--           
  .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */ --//--                         
  .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */ --//--                         
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask 0x3e0
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */ src/main/drivers/mcu/stm32/timer_hal.c, declared using _TIM_IRQ_HANDLER        
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */ --//--    
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger/Commutation, TIM11 */ --//--
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */ --//--                         
  .word     TIM2_IRQHandler                   /* TIM2                         */ --//--            
  .word     TIM3_IRQHandler                   /* TIM3                         */ --//--                  
  .word     TIM4_IRQHandler                   /* TIM4                         */ --//--                  
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */ emacs src/main/drivers/mcu/stm32/bus_i2c_hal.c                     
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */ --//--
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */ --//--                   
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */ --//--                     
  .word     SPI1_IRQHandler                   /* SPI1                         */ empty function               
  .word     SPI2_IRQHandler                   /* SPI2                         */ --//--    
  .word     USART1_IRQHandler                 /* USART1                       */ emacs src/main/drivers/serial_uart.c                  
  .word     USART2_IRQHandler                 /* USART2                       */ --//--
  .word     USART3_IRQHandler                 /* USART3                       */ --//--            
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */ src/main/drivers/mcu/stm32/exti.c, fun. EXTI_IRQHandler, mask 0xfc00   
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */ empty function               
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */ src/main/drivers/mcu/stm32/vcpf4/stm32f4xx_it.c                   
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */ src/main/drivers/mcu/stm32/timer_hal.c, declared using _TIM_IRQ_HANDLER        
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */ --//--
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger/Commutation, TIM14 */ --//--
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */ --//--                         
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */ src/main/drivers/dma.[hc]. DEFINE_DMA_IRQ_HANDLER is used to define the IRQ handlers
  .word     FSMC_IRQHandler                   /* FSMC                         */ empty function                 
  .word     SDIO_IRQHandler                   /* SDIO                         */ empty function    
  .word     TIM5_IRQHandler                   /* TIM5                         */ src/main/drivers/mcu/stm32/timer_hal.c, declared using _TIM_IRQ_HANDLER                
  .word     SPI3_IRQHandler                   /* SPI3                         */ --//--                
  .word     UART4_IRQHandler                  /* UART4                        */ emacs src/main/drivers/serial_uart.c                  
  .word     UART5_IRQHandler                  /* UART5                        */ --//--
  .word     TIM6_DAC_IRQHandler               /* TIM6, DAC1&2 underrun errors */ src/main/drivers/mcu/stm32/timer_hal.c, declared using _TIM_IRQ_HANDLER                  
  .word     TIM7_IRQHandler                   /* TIM7                         */ --//--
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */ src/main/drivers/dma.[hc]. DEFINE_DMA_IRQ_HANDLER is used to define the IRQ handlers
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */ --//--
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */ --//--
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */ --//--
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */ --//--            
  .word     ETH_IRQHandler                    /* Ethernet                     */ empty function
  .word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */ empty function                    
  .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */ empty function                        
  .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */ --//--           
  .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */ --//--                         
  .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */ --//--                         
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */ src/main/drivers/mcu/stm32/vcpf4/stm32f4xx_it.c              
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */ src/main/drivers/dma.[hc]. DEFINE_DMA_IRQ_HANDLER is used to define the IRQ handlers
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */ --//--
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */ --//--
  .word     USART6_IRQHandler                 /* USART6                       */ src/main/drivers/serial_uart.c                 
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */ src/main/drivers/mcu/stm32/bus_i2c_hal.c                           
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */ --//--                        
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */ empty function                 
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */ empty function              
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */ empty function                         
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */ empty function                 
  .word     DCMI_IRQHandler                   /* DCMI                         */ empty function                   
  .word     CRYP_IRQHandler                   /* CRYP crypto                  */ empty function                  
  .word     HASH_RNG_IRQHandler               /* Hash and Rng                 */ empty function
  .word     FPU_IRQHandler                    /* FPU                          */ empty function

NOTE1: USB OTG is activated in FS mode (full speed = 12Mbps), not HS mode (high speed = 480Mbps),
       cf https://electronics.stackexchange.com/questions/234516/stm32-usb-device-vs-usb-otg-hs-what-is-the-difference#:~:text=USB%20OTG%20FS%3A%20able%20to,to%20actually%20support%20high%20speed.

NOTE2: The non-empty interrupts are:
       * reset handler 
       * HardFault      src/main/fc/hardfaults.c
           - Terminates execution
       * SysTick        src/main/drivers/system.c
                          > the handler
                        lib/main/CMSIS/Core/Include/core_cm4.h
			  > the SysTick_Type
			  > the default configuration function (static inline) SysTick_Config
           - Calls no function, only some bookkeeping
	   - Runs on 1ms (?)
       * EXTI           src/main/drivers/mcu/stm32/exti.c
       * DMA            src/main/drivers/dma.[hc]
       * TIM            src/main/drivers/mcu/stm32/timer_hal.c
       * I2C            src/main/drivers/mcu/stm32/bus_i2c_hal.c
       * USART          src/main/drivers/serial_uart.c  
       * OTG FS         src/main/drivers/mcu/stm32/vcpf4/stm32f4xx_it.c

NOTE3: It's unfortunate that my current setting does not yet include SPI, as I will have to
       use it for the extra inertial unit.
