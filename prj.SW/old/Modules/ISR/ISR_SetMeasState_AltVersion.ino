/* ==========================
   Function: SetMeasurementState
   Purpose : Control hardware & ISR states
   ========================== */
void SetMeasurementState(bool on)
{
    digitalWrite(MEASURE_ON, on ? HIGH : LOW);
    digitalWrite(MEASURE_OFF, on ? LOW : HIGH);

    detachInterrupt(startButton.PIN);
    detachInterrupt(stopButton.PIN);

    delayMicroseconds(2000); // debounce + signal stabilization

    if (on)
    {
        attachInterrupt(stopButton.PIN, StopBtn_ISRHdl, FALLING);
        Serial.println("SetMeasurementState: Attached STOP ISR");
    }
    else
    {
        attachInterrupt(startButton.PIN, StartBtn_ISRHdl, FALLING);
        Serial.println("SetMeasurementState: Attached START ISR");
    }
}

/* ==========================
   ISR: Start Button
   ========================== */
void IRAM_ATTR StartBtn_ISRHdl()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(TaskC0Handle, &xHigherPriorityTaskWoken);
    SetMeasurementState(true);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ==========================
   ISR: Stop Button
   ========================== */
void IRAM_ATTR StopBtn_ISRHdl()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    stopFlag = true;
    SetMeasurementState(false);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}