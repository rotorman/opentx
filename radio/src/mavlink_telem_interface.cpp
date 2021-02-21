/*
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"

// -- CoOS RTOS mavlink task handlers --

RTOS_TASK_HANDLE mavlinkTaskId;
RTOS_DEFINE_STACK(mavlinkStack, MAVLINK_STACK_SIZE);

struct MavlinkTaskStat {
  uint16_t start = 0;
  uint16_t last = 0;
  uint16_t max = 0;
  uint16_t load = 0;
};
struct MavlinkTaskStat mavlinkTaskStat;

uint16_t mavlinkTaskRunTime(void)
{
  return mavlinkTaskStat.start/2;
}

uint16_t mavlinkTaskRunTimeMax(void)
{
  return mavlinkTaskStat.max/2;
}

uint16_t mavlinkTaskLoad(void)
{
  return mavlinkTaskStat.load;
}

TASK_FUNCTION(mavlinkTask)
{
  while (true) {
    uint16_t start_last = mavlinkTaskStat.start;
    mavlinkTaskStat.start = getTmr2MHz();

    mavlinkTelem.wakeup();

    mavlinkTaskStat.last = getTmr2MHz() - mavlinkTaskStat.start;
    if (mavlinkTaskStat.last > mavlinkTaskStat.max) mavlinkTaskStat.max = mavlinkTaskStat.last;
    mavlinkTaskStat.load = mavlinkTaskStat.last / (mavlinkTaskStat.start - start_last);

    RTOS_WAIT_TICKS(2);
  }
}

void mavlinkStart()
{
  RTOS_CREATE_TASK(mavlinkTaskId, mavlinkTask, "mavlink", mavlinkStack, MAVLINK_STACK_SIZE, MAVLINK_TASK_PRIO);
}

// -- SERIAL and USB CDC handlers --

uint32_t _cvtBaudrate(uint16_t baud)
{
  switch (baud) {
    case 0: return 57600;
    case 1: return 115200;
    case 2: return 38400;
    case 3: return 19200;
  }
  return 57600;
}

uint32_t mavlinkTelemBaudrate(void)
{
  return _cvtBaudrate(g_eeGeneral.mavlinkBaudrate);
}

uint32_t mavlinkTelemBaudrate2(void)
{
  return _cvtBaudrate(g_eeGeneral.mavlinkBaudrate2);
}

#if !defined(AUX_SERIAL)
uint32_t mavlinkTelemAvailable(void){ return 0; }
uint8_t mavlinkTelemGetc(uint8_t *c){ return 0; }
bool mavlinkTelemHasSpace(uint16_t count){ return false; }
bool mavlinkTelemPutBuf(const uint8_t *buf, const uint16_t count){ return false; }
#endif

#if !defined(AUX2_SERIAL)
uint32_t mavlinkTelem2Available(void){ return 0; }
uint8_t mavlinkTelem2Getc(uint8_t *c){ return 0; }
bool mavlinkTelem2HasSpace(uint16_t count){ return false; }
bool mavlinkTelem2PutBuf(const uint8_t *buf, const uint16_t count){ return false; }
#endif

#if !defined(TELEMETRY_MAVLINK_USB_SERIAL)
uint32_t mavlinkTelem3Available(void){ return 0; }
uint8_t mavlinkTelem3Getc(uint8_t *c){ return 0; }
bool mavlinkTelem3HasSpace(uint16_t count){ return false; }
bool mavlinkTelem3PutBuf(const uint8_t *buf, const uint16_t count){ return false; }
#endif

#if defined(AUX_SERIAL)

MAVLINK_RAM_SECTION Fifo<uint8_t, 2*512> auxSerialTxFifo;
MAVLINK_RAM_SECTION Fifo<uint8_t, 2*512> auxSerialRxFifo_4MavlinkTelem;

uint32_t mavlinkTelemAvailable(void)
{
  if (auxSerialMode != UART_MODE_MAVLINK) return 0;
  return auxSerialRxFifo_4MavlinkTelem.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelemGetc(uint8_t *c)
{
  return auxSerialRxFifo_4MavlinkTelem.pop(*c);
}

bool mavlinkTelemHasSpace(uint16_t count)
{
  if (auxSerialMode != UART_MODE_MAVLINK) return false;
  return auxSerialTxFifo.hasSpace(count);
}

bool mavlinkTelemPutBuf(const uint8_t *buf, const uint16_t count)
{
  if (auxSerialMode != UART_MODE_MAVLINK || !buf || !auxSerialTxFifo.hasSpace(count)) {
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    uint8_t c = buf[i];
    auxSerialTxFifo.push(c);
  }
  USART_ITConfig(AUX_SERIAL_USART, USART_IT_TXE, ENABLE);
  return true;
}

#endif

#if defined(AUX2_SERIAL)

MAVLINK_RAM_SECTION Fifo<uint8_t, 2*512> aux2SerialTxFifo;
MAVLINK_RAM_SECTION Fifo<uint8_t, 2*512> aux2SerialRxFifo_4MavlinkTelem;

uint32_t mavlinkTelem2Available(void)
{
  if (aux2SerialMode != UART_MODE_MAVLINK) return 0;
  return aux2SerialRxFifo_4MavlinkTelem.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem2Getc(uint8_t *c)
{
  return aux2SerialRxFifo_4MavlinkTelem.pop(*c);
}

bool mavlinkTelem2HasSpace(uint16_t count)
{
  if (aux2SerialMode != UART_MODE_MAVLINK) return false;
  return aux2SerialTxFifo.hasSpace(count);
}

bool mavlinkTelem2PutBuf(const uint8_t *buf, const uint16_t count)
{
  if (aux2SerialMode != UART_MODE_MAVLINK || !buf || !aux2SerialTxFifo.hasSpace(count)) {
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    uint8_t c = buf[i];
    aux2SerialTxFifo.push(c);
  }
  USART_ITConfig(AUX2_SERIAL_USART, USART_IT_TXE, ENABLE);
  return true;
}

#endif

#if defined(TELEMETRY_MAVLINK_USB_SERIAL)

MAVLINK_RAM_SECTION Fifo<uint8_t, 2*512> mavlinkTelemUsbRxFifo;

uint32_t mavlinkTelem3Available(void)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE) return 0;
  return mavlinkTelemUsbRxFifo.size();
}

// call only after check with mavlinkTelem2Available()
uint8_t mavlinkTelem3Getc(uint8_t *c)
{
  return mavlinkTelemUsbRxFifo.pop(*c);
}

bool mavlinkTelem3HasSpace(uint16_t count)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE) return false;
  return true; //??
}

bool mavlinkTelem3PutBuf(const uint8_t *buf, const uint16_t count)
{
  if (getSelectedUsbMode() != USB_MAVLINK_MODE || !buf) {
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    usbSerialPutc(buf[i]);
  }
  return true;
}
#endif

// -- Miscellaneous stuff --

void MavlinkTelem::telemetrySetValue(uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec)
{
  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;
  }

  if (g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, id, subId, instance, value, unit, prec);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
  }
}

// only for MAVLINK_MSG_ID_RADIO_STATUS, MAVLINK_MSG_ID_RC_CHANNELS, MAVLINK_MSG_ID_RC_CHANNELS_RAW
void MavlinkTelem::telemetrySetRssiValue(uint8_t rssi)
{
  if (g_model.mavlinkRssiScale > 0) {
    if (g_model.mavlinkRssiScale < 255) { //if not full range, respect  UINT8_MAX
      if (rssi == UINT8_MAX) rssi = 0;
    }
    if (rssi > g_model.mavlinkRssiScale) rssi = g_model.mavlinkRssiScale; //constrain
    rssi = (uint8_t)( ((uint16_t)rssi * 100) / g_model.mavlinkRssiScale); //scale to 0..99
  }
  else { //mavlink default
    if (rssi == UINT8_MAX) rssi = 0;
  }

  radio.rssi_scaled = rssi;

  if (g_model.mavlinkRssi) {
    if (!radio.is_receiving && !radio.is_receiving65 && !radio.is_receiving35) return;
  }

  if (g_model.mavlinkRssi) {
    telemetryData.rssi.set(rssi);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
  }

  if (g_model.mavlinkRssi || g_model.mavlinkMimicSensors) {
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, (int32_t)rssi, UNIT_DB, 0);
    telemetryStreaming = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT; //2 * TELEMETRY_TIMEOUT10ms; // 2 seconds
  }
  //#if defined(MULTIMODULE)
  //{ TX_RSSI_ID, TX_RSSI_ID, 0, ZSTR_TX_RSSI   , UNIT_DB , 0 },
  //{ TX_LQI_ID , TX_LQI_ID,  0, ZSTR_TX_QUALITY, UNIT_RAW, 0 },
}

// is probably not needed, aren't they reset by telementryStreaming timeout?
void MavlinkTelem::telemetryResetRssiValue(void)
{
  if (radio.is_receiving || radio.is_receiving65 || radio.is_receiving35) return;

  radio.rssi_scaled = 0;

  if (g_model.mavlinkRssi)
    telemetryData.rssi.reset();

  if (g_model.mavlinkRssi || g_model.mavlinkMimicSensors)
    setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, 0, UNIT_DB, 0);
}

bool MavlinkTelem::telemetryVoiceEnabled(void)
{
  if (!g_model.mavlinkRssi && !g_model.mavlinkMimicSensors) return true;

  if (g_model.mavlinkRssi && !radio.rssi_voice_disabled) return true;

  return false;
}

