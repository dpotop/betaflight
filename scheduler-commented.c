
// Time accounting
// ===============
//
// All timing accounting is done in one of two units: cycles or microseconds (us).
// By cycles we understand here processor clock cycles:
// - The STM32f405 clock runs at 168MHz
// - The getCycleCounter routine (of src/main/drivers/system.c) is used to
//   determine the current cycle counter by reading the DWT->CYCCNT 32-bit HW counter
//   which is incremented at every clock cycle (more information on DWT
//   in lib/main/CMSIS/Core/Include/core_cm4.h, in DWT_Type, and in
//   https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/CYCCNT-cycle-counter-and-related-timers).
// - By dividing the uint32 range by 168MHz, one gets 25, meaning that
//   every 25 seconds the timer register will overflow.
// The conversions between microseconds and cycles can be found in
// src/main/drivers/system.c. For instance, the number of cycles in a microsecond
// is given by:
// - usTicks = cpuClockFrequency/1000000. If the frequency is 168MHz, then
//   usTicks = 168.
// For comparison, when the desired PID frequency is 8000 Hz, I will have
// a 1250us period, which equals 125*168=21000 cycles.

// Scheduling
// ==========
//
// Scheduling is not time-triggered, but event-driven, time-aware.
// It consists in a free-running loop dynamically aligning itself on
// a desired period. This desired period is stored in desiredPeriodCycles.
//
// Initialization of this desired period is done with the period specified
// for TASK_GYRO.
//
// The value can then evolve to:
// - align on gyro device interrupt frequency (there can always be
//   small few-cycles error due to time counting rounding errors, so
//   some robustness here cannot hurt).
// - deadline misses and other real-time statistics. As it happens,
//   longer tasks are never preempted, so they *will* stop the
//   gyro tasks processing for some time.
static int32_t desiredPeriodCycles;


// From main/common/time.h:
static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b) { return (int32_t)(a - b); }


FAST_CODE void scheduler(void)
{
    static uint32_t checkCycles = 0;
    static uint32_t scheduleCount = 0;

    const timeUs_t schedulerStartTimeUs = micros();

    timeUs_t currentTimeUs;
    uint32_t nowCycles;
    timeUs_t taskExecutionTimeUs = 0;
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    uint32_t nextTargetCycles = 0;
    int32_t schedLoopRemainingCycles;
    bool firstSchedulingOpportunity = false;

    // DPB: step 1: gyro/filtering/PID
    // Q1: can betaflight run if the gyro is not enabled?!
    // I guess yes, if the drone is not flying, but only communicating with the
    // computer by USB. From my point of view, this part is always enabled. 
    if (gyroEnabled) {
        // Realtime gyro/filtering/PID tasks get complete priority
        task_t *gyroTask = getTask(TASK_GYRO);

        nowCycles = getCycleCounter();
	// DPB: Set the deadline of the current cycle (which is also start of next cycle)
	// assuming the previous cycle finished exactly on time. Given that the previous
	// cycle takes a little more or a little less than predicted, this is never exact.
        nextTargetCycles = lastTargetCycles + desiredPeriodCycles;
	// DPB: Determine how many cycles of execution remain, given the deadline computation
	// and the current actual date.
        schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

	// DPB: If actual execution is **very** late (more than two full periods late, given
	// that I compare with the deadline), then replenish with as many periods as
	// needed so that the deadline is more than one period into the future.
	// This seems a quite brutal form of deadline miss management, by direct replenish.
	// Note that this method should allow for tasks of any length to execute...
        if (schedLoopRemainingCycles < -desiredPeriodCycles) {
	  /* A task has so grossly overrun that at entire gyro cycle has been skipped
	   * This is most likely to occur when connected to the configurator via USB as the serial
	   * task is non-deterministic
	   * Recover as best we can, advancing scheduling by a whole number of cycles
	   */
	  // Form of replenish
	  nextTargetCycles += desiredPeriodCycles * (1 + (schedLoopRemainingCycles / -desiredPeriodCycles));
	  schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

	// DPB: This is a weird tuning phase.
	// The tuned value is schedLoopStartCycles, which can vary between:
	// - schedLoopStartMinCycles = SCHED_START_LOOP_MIN_US =  1us =  168 cycles
	// - schedLoopStartMaxCycles = SCHED_START_LOOP_MAX_US = 12us = 2016 cycles
	// Tuning is done with different up and down increments (durations in cycles):
	// - SCHED_START_LOOP_DOWN_STEP = 50
	// - SCHED_START_LOOP_UP_STEP   =  1
	
	// If the previous replenishing did not take place, and if the remaining
	// cycles are few enough, and if I did not go over a fixed limit with the
	// schedLoopStartCycles, then increase this last quantity by
	// schedLoopStartDeltaUpCycles.
        // Tune out the time lost between completing the last task execution and re-entering the scheduler
        if ((schedLoopRemainingCycles < schedLoopStartMinCycles) &&
            (schedLoopStartCycles < schedLoopStartMaxCycles)) {
            schedLoopStartCycles += schedLoopStartDeltaUpCycles;
        }

        // Once close to the timing boundary, poll for it's arrival
        if (schedLoopRemainingCycles < schedLoopStartCycles) {
            if (schedLoopStartCycles > schedLoopStartMinCycles) {
                schedLoopStartCycles -= schedLoopStartDeltaDownCycles;
            }

            while (schedLoopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }

	    // DPB: second point where I don't understand the code: functions gyroFilterReady
	    // and pidLoopReady below  check if a modulo counter has a specific value. The
	    // problem is that I don't understand how the modulo base (activePidLoopDenom)
	    // is computed. Because, of course, there's a catch in how it's computed:
	    // - The default value DEFAULT_PID_PROCESS_DENOM is 1
	    // - 
            currentTimeUs = micros();
            taskExecutionTimeUs += schedulerExecuteTask(gyroTask, currentTimeUs);

	    // Weird: why is currentTimeUs used in both Filter and PID? Does this mean
	    // that 
            if (gyroFilterReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
            }
            if (pidLoopReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
            }

            // Check for incoming RX data. Don't do this in the checker as that is called repeatedly within
            // a given gyro loop, and ELRS takes a long time to process this and so can only be safely processed
            // before the checkers
            rxFrameCheck(currentTimeUs, cmpTimeUs(currentTimeUs, getTask(TASK_RX)->lastExecutedAtUs));

            // Check for failsafe conditions without reliance on the RX task being well behaved
            if (cmp32(millis(), lastFailsafeCheckMs) > PERIOD_RXDATA_FAILURE) {
                // This is very low cost taking less that 4us every 10ms
                failsafeCheckDataFailurePeriod();
                failsafeUpdateState();
                lastFailsafeCheckMs = millis();
            }

            // This is the first scheduling opportunity after the realtime tasks have run
            firstSchedulingOpportunity = true;

            lastTargetCycles = nextTargetCycles;

            gyroDev_t *gyro = gyroActiveDev();

            // Bring the scheduler into lock with the gyro
            if (gyro->gyroModeSPI != GYRO_EXTI_NO_INT) {
                // Track the actual gyro rate over given number of cycle times and set the expected timebase
                static uint32_t terminalGyroRateCount = 0;
                static int32_t sampleRateStartCycles;

                if (terminalGyroRateCount == 0) {
                    terminalGyroRateCount = gyro->detectedEXTI + GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                }

                if (gyro->detectedEXTI >= terminalGyroRateCount) {
                    // Calculate the number of clock cycles on average between gyro interrupts
                    uint32_t sampleCycles = nowCycles - sampleRateStartCycles;
                    desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                    terminalGyroRateCount += GYRO_RATE_COUNT;
                }

                // Track the actual gyro rate over given number of cycle times and remove skew
                static uint32_t terminalGyroLockCount = 0;
                static int32_t accGyroSkew = 0;

                int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyro->gyroSyncEXTI) % desiredPeriodCycles;
                if (gyroSkew > (desiredPeriodCycles / 2)) {
                    gyroSkew -= desiredPeriodCycles;
                }

                accGyroSkew += gyroSkew;


                if (terminalGyroLockCount == 0) {
                    terminalGyroLockCount = gyro->detectedEXTI + GYRO_LOCK_COUNT;
                }

                if (gyro->detectedEXTI >= terminalGyroLockCount) {
                    terminalGyroLockCount += GYRO_LOCK_COUNT;

                    // Move the desired start time of the gyroTask
                    lastTargetCycles -= (accGyroSkew/GYRO_LOCK_COUNT);

                    accGyroSkew = 0;
                }
            }
       }
    }

    nowCycles = getCycleCounter();
    schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

    // DPB: step 2: all other tasks
    // If gyro not enabled or I have enough time left
    if (!gyroEnabled || (schedLoopRemainingCycles > (int32_t)clockMicrosToCycles(CHECK_GUARD_MARGIN_US))) {
        currentTimeUs = micros();

        // Update task dynamic priorities
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
            if (task->attribute->staticPriority != TASK_PRIORITY_REALTIME) {
                // Task has checkFunc - event driven
                if (task->attribute->checkFunc) {
                    // Increase priority for event driven tasks
                    if (task->dynamicPriority > 0) {
                        task->taskAgePeriods = 1 + (cmpTimeUs(currentTimeUs, task->lastSignaledAtUs) / task->attribute->desiredPeriodUs);
                        task->dynamicPriority = 1 + task->attribute->staticPriority * task->taskAgePeriods;
                    } else if (task->attribute->checkFunc(currentTimeUs, cmpTimeUs(currentTimeUs, task->lastExecutedAtUs))) {
                        const uint32_t checkFuncExecutionTimeUs = cmpTimeUs(micros(), currentTimeUs);
                        checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
                        checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
                        task->lastSignaledAtUs = currentTimeUs;
                        task->taskAgePeriods = 1;
                        task->dynamicPriority = 1 + task->attribute->staticPriority;
                    } else {
                        task->taskAgePeriods = 0;
                    }
                } else {
                    // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
                    // Task age is calculated from last execution
                    task->taskAgePeriods = (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) / task->attribute->desiredPeriodUs);
                    if (task->taskAgePeriods > 0) {
                        task->dynamicPriority = 1 + task->attribute->staticPriority * task->taskAgePeriods;
                    }
                }

                if (task->dynamicPriority > selectedTaskDynamicPriority) {
                    timeDelta_t taskRequiredTimeUs = task->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
                    int32_t taskRequiredTimeCycles = (int32_t)clockMicrosToCycles((uint32_t)taskRequiredTimeUs);
                    // Allow a little extra time
                    taskRequiredTimeCycles += checkCycles + taskGuardCycles;

                    // If there's no time to run the task, discount it from prioritisation unless aged sufficiently
                    // Don't block the SERIAL task.
                    if ((taskRequiredTimeCycles < schedLoopRemainingCycles) ||
                        ((scheduleCount & SCHED_TASK_DEFER_MASK) == 0) ||
                        ((task - tasks) == TASK_SERIAL)) {
                        selectedTaskDynamicPriority = task->dynamicPriority;
                        selectedTask = task;
                    }
                }
            }

        }

        // The number of cycles taken to run the checkers is quite consistent with some higher spikes, but
        // that doesn't defeat its use
        checkCycles = cmpTimeCycles(getCycleCounter(), nowCycles);

        if (selectedTask) {
            // Recheck the available time as checkCycles is only approximate
            timeDelta_t taskRequiredTimeUs = selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;

            int32_t taskRequiredTimeCycles = (int32_t)clockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            nowCycles = getCycleCounter();
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            // Allow a little extra time
            taskRequiredTimeCycles += taskGuardCycles;

            if (!gyroEnabled || firstSchedulingOpportunity || (taskRequiredTimeCycles < schedLoopRemainingCycles)) {
                uint32_t antipatedEndCycles = nowCycles + taskRequiredTimeCycles;
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
                nowCycles = getCycleCounter();
                int32_t cyclesOverdue = cmpTimeCycles(nowCycles, antipatedEndCycles);

                if ((currentTask - tasks) == TASK_RX) {
                    skippedRxAttempts = 0;
                }

                if ((cyclesOverdue > 0) || (-cyclesOverdue < taskGuardMinCycles)) {
                    if (taskGuardCycles < taskGuardMaxCycles) {
                        taskGuardCycles += taskGuardDeltaUpCycles;
                    }
                } else if (taskGuardCycles > taskGuardMinCycles) {
                    taskGuardCycles -= taskGuardDeltaDownCycles;
                }
		
            } else if ((selectedTask->taskAgePeriods > TASK_AGE_EXPEDITE_COUNT) ||

                       (((selectedTask - tasks) == TASK_RX) && (TASK_AGE_EXPEDITE_RX != 0) && (++skippedRxAttempts > TASK_AGE_EXPEDITE_RX))) {
                // If a task has been unable to run, then reduce it's recorded estimated run time to ensure
                // it's ultimate scheduling
                selectedTask->anticipatedExecutionTime *= TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

    DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - schedulerStartTimeUs - taskExecutionTimeUs); // time spent in scheduler

    scheduleCount++;
}
