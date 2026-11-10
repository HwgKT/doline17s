#include "bt_telemetry.h"
#include <BluetoothSerial.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP requires original ESP32 and Bluedroid"
#endif
namespace {
BluetoothSerial SerialBT;
constexpr size_t FRAME_BYTES=2+sizeof(Telemetry)+1;
constexpr UBaseType_t QUEUE_FRAMES=12;
constexpr UBaseType_t QUEUE_REPLIES=12;
constexpr size_t REPLY_BYTES=512; // DIAG/PERF lines can exceed 192 bytes.
struct Frame { uint8_t bytes[FRAME_BYTES]; };
struct Reply { char bytes[REPLY_BYTES]; size_t len; };
QueueHandle_t txQueue=nullptr;
QueueHandle_t replyQueue=nullptr;
TaskHandle_t txTask=nullptr;
volatile uint32_t queued=0,sent=0,dropped=0,failed=0;
void txWorker(void *) {
  Frame frame{};
  Reply reply{};
  for(;;){
    // Control replies have priority over bulk telemetry. Only this task
    // writes to SerialBT, so text can never split a binary 72-byte frame.
    if(xQueueReceive(replyQueue,&reply,0)==pdTRUE){
      if(SerialBT.hasClient()){
        const size_t n=SerialBT.write(
          reinterpret_cast<const uint8_t *>(reply.bytes),reply.len);
        if(n!=reply.len)++failed;
      }
      continue;
    }
    if(xQueueReceive(txQueue,&frame,pdMS_TO_TICKS(5))!=pdTRUE)continue;
    // Check again before writing a frame: FINISH/STOP must go first.
    if(uxQueueMessagesWaiting(replyQueue)>0){
      // Control burst: skip a sample rather than delay a stop ACK.
      ++dropped;
      continue;
    }
    if(!SerialBT.hasClient()){++dropped;continue;}
    // SPP can block under backpressure; never block the PID loop.
    const size_t n=SerialBT.write(frame.bytes,sizeof(frame.bytes));
    if(n==sizeof(frame.bytes))++sent; // Accepted by SPP, NOT delivered to PC.
    else ++failed;
  }
}
}
bool btBegin(const char *name){
  if(!SerialBT.begin(name))return false;
  txQueue=xQueueCreate(QUEUE_FRAMES,sizeof(Frame));
  replyQueue=xQueueCreate(QUEUE_REPLIES,sizeof(Reply));
  if(!txQueue||!replyQueue){
    if(txQueue)vQueueDelete(txQueue);
    if(replyQueue)vQueueDelete(replyQueue);
    txQueue=nullptr;replyQueue=nullptr;SerialBT.end();return false;
  }
  if(xTaskCreatePinnedToCore(txWorker,"spp_telemetry",4096,nullptr,1,&txTask,0)!=pdPASS){
    vQueueDelete(txQueue);vQueueDelete(replyQueue);
    txQueue=nullptr;replyQueue=nullptr;SerialBT.end();return false;
  }
  return true;
}
bool btConnected(){return SerialBT.hasClient();}
void btSendLine(const String &line){
  if(!btConnected()||!replyQueue)return;
  Reply reply{};
  const size_t length=line.length();
  if(length+1>=REPLY_BYTES){++failed;return;}
  memcpy(reply.bytes,line.c_str(),length);
  reply.bytes[length]='\n';
  reply.len=length+1;
  // Never hold the 650-Hz control loop for Bluetooth writes.
  if(xQueueSend(replyQueue,&reply,0)!=pdTRUE){
    ++failed; // Observable through DIAG; never block the motion loop.
  }
}
bool btSendTelemetry(const Telemetry &sample){
  if(!btConnected()||!txQueue)return false;
  Frame frame{};
  frame.bytes[0]=0xAA;frame.bytes[1]=0x55;
  memcpy(frame.bytes+2,&sample,sizeof(sample));
  uint8_t check=0;
  for(size_t i=0;i<sizeof(sample);++i)check^=frame.bytes[2+i];
  frame.bytes[FRAME_BYTES-1]=check;
  if(xQueueSend(txQueue,&frame,0)!=pdTRUE){++dropped;return false;}
  ++queued;return true;
}
BtTxStats btGetTxStats(){
  BtTxStats s{};
  s.queued=queued;s.sent=sent;s.dropped=dropped;s.failed=failed;
  s.pending=txQueue?uxQueueMessagesWaiting(txQueue):0;
  return s;
}
void btDropPendingTelemetry(){
  if(txQueue){
    const uint32_t n=uxQueueMessagesWaiting(txQueue);
    xQueueReset(txQueue);dropped+=n;
  }
  // Keep replyQueue intact: queued RUN_ORIGIN / FINISH / STOP events
  // are critical and must survive telemetry flushes.
}
void btPollCommands(BtCommandHandler handler){
  static String line;
  uint16_t n=0;
  while(SerialBT.available()&&n++<128){
    char c=(char)SerialBT.read();
    if(c=='\n'||c=='\r'){
      if(line.length()&&handler)handler(line);
      line="";
    } else if(c>=32&&c<=126&&line.length()<80)line+=c;
    else if(line.length()>=80)line="";
  }
}
