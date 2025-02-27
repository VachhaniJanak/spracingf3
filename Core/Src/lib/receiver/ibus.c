
#define IBUS_BAUD_RATE 115200 // iBUS runs at 115200 baud rate
#define IBUS_PACKET_SIZE 32  // iBUS packets are 32 bytes long
#define IBUS_MAX_CHANNELS 14 // Max number of channels in iBUS
#define IBUS_MAX_SLOTS 14
#define IBUS_BUFFSIZE 32
#define IBUS_MODEL_IA6B 0
#define IBUS_MODEL_IA6 1
#define IBUS_FRAME_GAP 500

#define IBUS_BAUDRATE 115200

uint16_t channels[IBUS_MAX_CHANNELS];  // Store channel values

void loop() {
  if (IBUS_SERIAL.available() >= IBUS_PACKET_SIZE) {
    uint8_t packet[IBUS_PACKET_SIZE];
    if (readIBusPacket(packet)) {
      if (validateIBusChecksum(packet)) {
        parseIBusChannels(packet);
        printChannels();
      } else {
        Serial.println("Checksum Error!");
      }
    }
  }
}

int readIBusPacket(uint8_t *packet) {
  // Wait for the start of an iBUS packet
  if (IBUS_SERIAL.read() != 0x20) return false;  // First byte of the header
  if (IBUS_SERIAL.read() != 0x40) return false;  // Second byte of the header

  // Read the rest of the packet
  packet[0] = 0x20;
  packet[1] = 0x40;
  for (int i = 2; i < IBUS_PACKET_SIZE; i++) {
    packet[i] = IBUS_SERIAL.read();
  }

  return true;
}

int validateIBusChecksum(uint8_t *packet) {
  uint16_t checksum = 0xFFFF;
  for (int i = 0; i < IBUS_PACKET_SIZE - 2; i++) {
    checksum -= packet[i];
  }
  uint16_t receivedChecksum = packet[IBUS_PACKET_SIZE - 2] | (packet[IBUS_PACKET_SIZE - 1] << 8);
  return checksum == receivedChecksum;
}

void parseIBusChannels(uint8_t *packet) {
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
    channels[i] = packet[2 + i * 2] | (packet[3 + i * 2] << 8);  // Combine low and high bytes
  }
}



static uint8_t ibusModel;
static uint8_t ibusSyncByte;
static uint8_t ibusFrameSize;
static uint8_t ibusChannelOffset;
static uint8_t rxBytesToIgnore;
static uint16_t ibusChecksum;

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };
static timeUs_t lastFrameTimeUs = 0;

// Receive ISR callback
static void ibusDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static timeUs_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    const timeUs_t now = microsISR();

    if (cmpTimeUs(now, ibusTimeLast) > IBUS_FRAME_GAP) {
        ibusFramePosition = 0;
        rxBytesToIgnore = 0;
    } else if (rxBytesToIgnore) {
        rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = now;

    if (ibusFramePosition == 0) {
        if (isValidIa6bIbusPacketLength(c)) {
            ibusModel = IBUS_MODEL_IA6B;
            ibusSyncByte = c;
            ibusFrameSize = c;
            ibusChannelOffset = 2;
            ibusChecksum = 0xFFFF;
        } else if ((ibusSyncByte == 0) && (c == 0x55)) {
            ibusModel = IBUS_MODEL_IA6;
            ibusSyncByte = 0x55;
            ibusFrameSize = 31;
            ibusChecksum = 0x0000;
            ibusChannelOffset = 1;
        } else if (ibusSyncByte != c) {
            return;
        }
    }

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibusFrameSize - 1) {
        lastFrameTimeUs = now;
        ibusFrameDone = true;
    } else {
        ibusFramePosition++;
    }
}

static bool isChecksumOkIa6(void)
{
    uint8_t offset;
    uint8_t i;
    uint16_t chksum, rxsum;
    chksum = ibusChecksum;
    rxsum = ibus[ibusFrameSize - 2] + (ibus[ibusFrameSize - 1] << 8);
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        chksum += ibus[offset] + (ibus[offset + 1] << 8);
    }
    return chksum == rxsum;
}

static bool checksumIsOk(void)
{
    if (ibusModel == IBUS_MODEL_IA6 ) {
        return isChecksumOkIa6();
    } else {
        return isChecksumOkIa6b(ibus, ibusFrameSize);
    }
}

static void updateChannelData(void)
{
    uint8_t i;
    uint8_t offset;
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        ibusChannelData[i] = ibus[offset] + ((ibus[offset + 1] & 0x0F) << 8);
    }
    //latest IBUS recievers are using prviously not used 4 bits on every channel to incresse total channel count
    for (i = IBUS_MAX_SLOTS, offset = ibusChannelOffset + 1; i < IBUS_MAX_CHANNEL; i++, offset += 6) {
        ibusChannelData[i] = ((ibus[offset] & 0xF0) >> 4) | (ibus[offset + 2] & 0xF0) | ((ibus[offset + 4] & 0xF0) << 4);
    }
}

static uint8_t ibusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibusFrameDone) {
        return frameStatus;
    }

    ibusFrameDone = false;

    if (checksumIsOk()) {
        if (ibusModel == IBUS_MODEL_IA6 || ibusSyncByte == IBUS_SERIAL_RX_PACKET_LENGTH) {
            updateChannelData();
            frameStatus = RX_FRAME_COMPLETE;
            rxRuntimeState->lastRcFrameTimeUs = lastFrameTimeUs;
        }
    }

    return frameStatus;
}

static float ibusReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibusChannelData[chan];
}

/*
 *  supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */