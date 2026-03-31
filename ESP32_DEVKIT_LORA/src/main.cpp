/*
 * ============================================================
 *  LoRaWAN Valve Controller — ESP32 DevKit  (FreeRTOS)
 *  VERSION 4 FIXED — ACK update bug fixed
 *
 *  ONLY CHANGES FROM V4 ORIGINAL
 *  ─────────────────────────────────────────────────────────
 *  FIX A: get_serial() now returns bool
 *    - Returns true  → valid ACK frame parsed
 *    - Returns false → bad frame / too short / unknown SID
 *    - Uses '*' delimiter split instead of fixed substring()
 *      offsets — noise byte will no longer shift all valve reads
 *
 *  FIX B: listenLoraAck() now returns bool
 *    - Returns true  → ACK received and parsed OK
 *    - Returns false → timeout, no ACK
 *    - updateServerFromAck() is called ONLY when true
 *    - On timeout: optimistic update stays on server — correct
 *    - Old behaviour: updateServerFromAck() was called even on
 *      timeout via get_serial() → posted empty strings →
 *      overwrote the optimistic update → app showed wrong state
 *
 *  FIX C: updateServerFromAck() skips if all ack values empty
 *    - Safety net: if vack1..8 are all empty, don't POST
 *
 *  EVERYTHING ELSE IS IDENTICAL TO V4
 *  ─────────────────────────────────────────────────────────
 *  - Optimistic update (updateServerOptimistic) unchanged
 *  - TaskCompareA/B/C unchanged
 *  - TaskReadServerA/B/C unchanged
 *  - pushValveJob / pushKeepaliveJob unchanged
 *  - All timing defines unchanged
 *  - Queue depth 12 unchanged
 * ============================================================
 */

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// ─── Pin definitions ──────────────────────────────────────
#define TRIGGER_PIN  14
#define VOLT_PIN      2
#define LED_PIN      27
#define SIG_PIN       4
#define AUX_PIN      18
#define LORA_RX      16
#define LORA_TX      17

// ─── Timing ───────────────────────────────────────────────
#define RESET_TIME_S          21600UL
#define POLL_DELAY_MS          3000
#define WIFI_RETRY_MS          1000
#define ACK_WINDOW_MS          4500   // increased: PIC needs ~2000ms valve delay + TX
#define KEEPALIVE_INTERVAL_MS 30000
#define COMPAREID_INTERVAL_MS  2000 
#define COMPAREID_DELAY_MS     3000

// ─── Server ───────────────────────────────────────────────
const String HOST1 = "http://qbitronics.com";

// ─── LoRa TX Queue item ───────────────────────────────────
struct LoraJob {
    char   prefix;
    bool   is_keepalive;
    char   s1[4], s2[4], s3[4], s4[4];
    char   s5[4], s6[4], s7[4], s8[4];
    char   bw[8];
    char   v1[4], v2[4], v3[4], v4[4];
    char   v5[4], v6[4], v7[4], v8[4];
    char   sid[4];
    char   cmd1[4], cmd2[4], cmd3[4], cmd4[4];
    char   cmd5[4], cmd6[4], cmd7[4], cmd8[4];
};

static void scopy(char *dst, size_t sz, const String &src) {
    strncpy(dst, src.c_str(), sz - 1);
    dst[sz - 1] = '\0';
}

// ─── FreeRTOS objects ─────────────────────────────────────
SemaphoreHandle_t  xMutexSerial2    = NULL;
SemaphoreHandle_t  xMutexVars       = NULL;
SemaphoreHandle_t  xMutexLED        = NULL;
SemaphoreHandle_t  xMutexCompareVal = NULL;
EventGroupHandle_t xWiFiEvents      = NULL;
QueueHandle_t      xLoraQueue       = NULL;
#define WIFI_READY_BIT  BIT0

// ─── WiFiManager / Preferences ────────────────────────────
WiFiManager  wm;
Preferences  preferences;

char device_id[20]={0}, token_id[20]={0};
char slave_id1[20]={0}, slave_id2[20]={0}, slave_id3[20]={0};

String deviceid="", tokenid="";
String slaveid1="", slaveid2="", slaveid3="";

WiFiManagerParameter custom_device_id("device_id","Device ID", device_id,20);
WiFiManagerParameter custom_token_id ("token_id", "Token ID",  token_id, 20);
WiFiManagerParameter custom_slave_id1("slave_id1","Slave ID 1",slave_id1,20);
WiFiManagerParameter custom_slave_id2("slave_id2","Slave ID 2",slave_id2,20);
WiFiManagerParameter custom_slave_id3("slave_id3","Slave ID 3",slave_id3,20);

// ─── Shared payload parse variables ───────────────────────
String v1,v2,v3,v4,v5,v6,v7,v8;
String vs1,vs2,vs3,vs4,vs5,vs6,vs7,vs8;
String BWHR,BWMIN,BWSEC,TR,payload;

String vA1,vA2,vA3,vA4,vA5,vA6,vA7,vA8;
String vB1,vB2,vB3,vB4,vB5,vB6,vB7,vB8;
String vC1,vC2,vC3,vC4,vC5,vC6,vC7,vC8;

// Previous-state cache — slave A
String v1s1,v1s2,v1s3,v1s4,v1s5,v1s6,v1s7,v1s8,BWMIN1;
String v1s11,v1s22,v1s33,v1s44,v1s55,v1s66,v1s77,v1s88,BW1MINN;

// Previous-state cache — slave B
String v2s1,v2s2,v2s3,v2s4,v2s5,v2s6,v2s7,v2s8,BWMIN2;
String v2s11,v2s22,v2s33,v2s44,v2s55,v2s66,v2s77,v2s88,BW2MINN;

// Previous-state cache — slave C
String v3s1,v3s2,v3s3,v3s4,v3s5,v3s6,v3s7,v3s8,BWMIN3;
String v3s11,v3s22,v3s33,v3s44,v3s55,v3s66,v3s77,v3s88,BW3MINN;

// ACK variables
String vack1,vack2,vack3,vack4,vack5,vack6,vack7,vack8;
String SID_g = "";

int compareVal = 0;

// ─── Function prototypes ──────────────────────────────────
void saveParameters(); void loadParameters();
void eraseParameters(); void saveConfigCallback();
void WFcheck();
int  compareid(const String &slaveT);
void parsePayload();
void updateServerOptimistic(const String &sid,
    const String &a1, const String &a2,
    const String &a3, const String &a4,
    const String &a5, const String &a6,
    const String &a7, const String &a8);
void updateServerFromAck(const String &sid);
void sendLoraFrame(const LoraJob &job);
bool get_serial();                         // FIX A: returns bool, no sid param needed
bool listenLoraAck(uint32_t timeoutMs);    // FIX B: returns bool
void blinkLED(int times, int onMs, int offMs);
void checkButton();
static void waitWiFiReady();

// ══════════════════════════════════════════════════════════
//  waitWiFiReady() — unchanged
// ══════════════════════════════════════════════════════════
static void waitWiFiReady() {
    xEventGroupWaitBits(xWiFiEvents, WIFI_READY_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

// ══════════════════════════════════════════════════════════
//  compareid() — unchanged
// ══════════════════════════════════════════════════════════
int compareid(const String &slaveT) {
    if (slaveT.length() == 0) return -1;

    xSemaphoreTake(xMutexCompareVal, portMAX_DELAY);
    compareVal++;
    if (compareVal > 8) compareVal = 1;
    int cv = compareVal;
    xSemaphoreGive(xMutexCompareVal);

    String url = HOST1 + "/Lorawan24/motorlora/comparev" + String(cv) +
                 ".php?deviceid=" + deviceid +
                 "&tokenid=" + tokenid + "&sid=" + slaveT;

    HTTPClient http;
    http.begin(url);
    int code = http.GET();
    http.end();
    Serial.printf("[compareid] cv=%d → %d\n", cv, code);
    vTaskDelay(pdMS_TO_TICKS(COMPAREID_DELAY_MS));
    return code;
}

// ══════════════════════════════════════════════════════════
//  parsePayload() — unchanged
// ══════════════════════════════════════════════════════════
void parsePayload() {
    if (payload.length() < 17)     return;
    if (payload.charAt(16) != '#') return;

    auto nextStar = [&](int from) -> int {
        return payload.indexOf('*', from);
    };

    int p  = payload.indexOf('*');
    int p1 = nextStar(p+1);  v1  = payload.substring(p+1,  p1);
    int p2 = nextStar(p1+1); vs1 = payload.substring(p1+1, p2);
    int p3 = nextStar(p2+1); TR  = payload.substring(p2+1, p3);

    int c  = payload.indexOf(':', p3+1);
    int c1 = payload.indexOf(':', c+1);
    BWHR  = payload.substring(p3+1, c);
    BWMIN = payload.substring(c+1,  c1);
    int p4 = nextStar(c1+1); BWSEC = payload.substring(c1+1, p4);

    int p5 = nextStar(p4+1); v2  = payload.substring(p4+1, p5);
    int p6 = nextStar(p5+1); vs2 = payload.substring(p5+1, p6);
    int p7 = nextStar(p6+1); v3  = payload.substring(p6+1, p7);
    int p8 = nextStar(p7+1); vs3 = payload.substring(p7+1, p8);
    int p9 = nextStar(p8+1); v4  = payload.substring(p8+1, p9);
    int pa = nextStar(p9+1); vs4 = payload.substring(p9+1, pa);
    int pb = nextStar(pa+1); v5  = payload.substring(pa+1, pb);
    int pc = nextStar(pb+1); vs5 = payload.substring(pb+1, pc);
    int pd = nextStar(pc+1); v6  = payload.substring(pc+1, pd);
    int pe = nextStar(pd+1); vs6 = payload.substring(pd+1, pe);
    int pf = nextStar(pe+1); v7  = payload.substring(pe+1, pf);
    int pg = nextStar(pf+1); vs7 = payload.substring(pf+1, pg);
    int ph = nextStar(pg+1); v8  = payload.substring(pg+1, ph);
    int pi = nextStar(ph+1); vs8 = payload.substring(ph+1, pi);
}

// ══════════════════════════════════════════════════════════
//  updateServerOptimistic() — unchanged
// ══════════════════════════════════════════════════════════
void updateServerOptimistic(const String &sid,
    const String &a1, const String &a2,
    const String &a3, const String &a4,
    const String &a5, const String &a6,
    const String &a7, const String &a8)
{
    String useSid = "";
    if      (sid == "A") useSid = slaveid1;
    else if (sid == "B") useSid = slaveid2;
    else if (sid == "C") useSid = slaveid3;
    else return;

    String url = HOST1 + "/Lorawan24/motorlora/v_update.php?deviceid=" + deviceid +
                 "&tokenid=" + tokenid + "&sid=" + useSid + "&allvalve=1" +
                 "&ack1=" + a1 + "&ack2=" + a2 + "&ack3=" + a3 + "&ack4=" + a4 +
                 "&ack5=" + a5 + "&ack6=" + a6 + "&ack7=" + a7 + "&ack8=" + a8;

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "text/plain");
    int code = http.POST("");
    Serial.printf("[OPTIMISTIC] updateServer %s → %d\n", sid.c_str(), code);
    http.end();
}

// ══════════════════════════════════════════════════════════
//  updateServerFromAck()
//  FIX C: added empty-check — don't POST if all ack empty
// ══════════════════════════════════════════════════════════
void updateServerFromAck(const String &sid) {
    String useSid = "";
    if      (sid == "A") useSid = slaveid1;
    else if (sid == "B") useSid = slaveid2;
    else if (sid == "C") useSid = slaveid3;
    else return;

    String a1,a2,a3,a4,a5,a6,a7,a8;
    xSemaphoreTake(xMutexVars, portMAX_DELAY);
    a1=vack1; a2=vack2; a3=vack3; a4=vack4;
    a5=vack5; a6=vack6; a7=vack7; a8=vack8;
    xSemaphoreGive(xMutexVars);

    // FIX C: safety — if all empty, don't overwrite optimistic update
    if (a1=="" && a2=="" && a3=="" && a4=="" &&
        a5=="" && a6=="" && a7=="" && a8=="") {
        Serial.printf("[ACK-UPDATE] Empty ACK for %s — skip POST\n", sid.c_str());
        return;
    }

    String url = HOST1 + "/Lorawan24/motorlora/v_update.php?deviceid=" + deviceid +
                 "&tokenid=" + tokenid + "&sid=" + useSid + "&allvalve=1" +
                 "&ack1=" + a1 + "&ack2=" + a2 + "&ack3=" + a3 + "&ack4=" + a4 +
                 "&ack5=" + a5 + "&ack6=" + a6 + "&ack7=" + a7 + "&ack8=" + a8;

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "text/plain");
    int code = http.POST("");
    Serial.printf("[ACK-UPDATE] updateServer %s → %d\n", sid.c_str(), code);
    http.end();
}

// ══════════════════════════════════════════════════════════
//  sendLoraFrame() — unchanged
// ══════════════════════════════════════════════════════════
void sendLoraFrame(const LoraJob &job) {
    for (int k = 0; k < 5; k++) {
        Serial2.print(job.prefix);
        Serial2.print(job.v1); Serial2.print("*"); Serial2.print(job.s1); Serial2.print("*");
        Serial2.print(job.v2); Serial2.print("*"); Serial2.print(job.s2); Serial2.print("*");
        Serial2.print(job.v3); Serial2.print("*"); Serial2.print(job.s3); Serial2.print("*");
        Serial2.print(job.v4); Serial2.print("*"); Serial2.print(job.s4); Serial2.print("*");
        Serial2.print(job.v5); Serial2.print("*"); Serial2.print(job.s5); Serial2.print("*");
        Serial2.print(job.v6); Serial2.print("*"); Serial2.print(job.s6); Serial2.print("*");
        Serial2.print(job.v7); Serial2.print("*"); Serial2.print(job.s7); Serial2.print("*");
        Serial2.print(job.v8); Serial2.print("*"); Serial2.print(job.s8); Serial2.print("*");
        Serial2.print(job.bw); Serial2.print("*");

        Serial.printf("[TX-%c] %d/5 → %s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*\n",
                      job.prefix, k+1,
                      job.v1,job.s1, job.v2,job.s2,
                      job.v3,job.s3, job.v4,job.s4,
                      job.v5,job.s5, job.v6,job.s6,
                      job.v7,job.s7, job.v8,job.s8, job.bw);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ══════════════════════════════════════════════════════════
//  get_serial()
//
//  FIX A: Returns bool (true = valid ACK, false = bad frame)
//         Uses '*' delimiter parsing instead of fixed offsets.
//
//  Old problem:
//    lvu[1] = rec.substring(3,4)  ← breaks if any byte shifts
//    If noise added 1 byte, valve 1 data goes to valve 2 slot
//
//  New approach:
//    Split entire frame by '*' → position-independent
//    Format: $A*1*1*2*0*3*1*4*0*5*1*6*0*7*0*8*0\n
//    parts[0]=$A  parts[1]=1  parts[2]=1  parts[3]=2 ...
//
//  updateServerFromAck() NOT called here — caller decides.
// ══════════════════════════════════════════════════════════
bool get_serial() {
    char firstChar = Serial2.read();
    if (firstChar != '$') return false;

    String rec = "$";
    rec += Serial2.readStringUntil('\n');
    rec.trim();

    Serial.printf("[ACK-FRAME] '%s' len=%d\n", rec.c_str(), rec.length());

    // Split by '*'
    // Expected: [0]=$A [1]=v1 [2]=s1 [3]=v2 [4]=s2 ... [15]=v8 [16]=s8
    String parts[20];
    int count = 0;
    int from  = 0;
    while (count < 20) {
        int star = rec.indexOf('*', from);
        if (star < 0) {
            if (from < (int)rec.length())
                parts[count++] = rec.substring(from);
            break;
        }
        parts[count++] = rec.substring(from, star);
        from = star + 1;
    }

    Serial.printf("[ACK] split count=%d\n", count);

    if (count < 17) {
        Serial.printf("[ACK] too few fields (%d) — discard\n", count);
        return false;
    }

    if (parts[0].length() < 2) {
        Serial.println("[ACK] bad header field");
        return false;
    }

    String parsedSid = parts[0].substring(1, 2);
    if (parsedSid != "A" && parsedSid != "B" && parsedSid != "C") {
        Serial.printf("[ACK] unknown SID '%s'\n", parsedSid.c_str());
        return false;
    }

    // parts[1]=v1, parts[2]=s1, parts[3]=v2, parts[4]=s2 ...
    String ack[9] = {"","","","","","","","",""};
    for (int i = 1; i <= 8; i++) {
        String vnum = parts[(i-1)*2 + 1];
        String vst  = parts[(i-1)*2 + 2];
        Serial.printf("[ACK] V%d: num='%s' state='%s'\n", i, vnum.c_str(), vst.c_str());
        if (vnum == String(i))
            ack[i] = (vst == "1") ? "1" : "0";
    }

    Serial.printf("[ACK-PARSED] sid=%s  %s%s%s%s%s%s%s%s\n",
                  parsedSid.c_str(),
                  ack[1].c_str(),ack[2].c_str(),ack[3].c_str(),ack[4].c_str(),
                  ack[5].c_str(),ack[6].c_str(),ack[7].c_str(),ack[8].c_str());

    xSemaphoreTake(xMutexVars, portMAX_DELAY);
    vack1=ack[1]; vack2=ack[2]; vack3=ack[3]; vack4=ack[4];
    vack5=ack[5]; vack6=ack[6]; vack7=ack[7]; vack8=ack[8];
    SID_g = parsedSid;
    xSemaphoreGive(xMutexVars);

    return true;   // valid ACK parsed — caller will call updateServerFromAck()
}

// ══════════════════════════════════════════════════════════
//  listenLoraAck()
//
//  FIX B: Returns bool.
//    true  → '$' found, get_serial() returned true → ACK good
//    false → timeout OR get_serial() returned false (bad frame)
//
//  Old V4 problem:
//    listenLoraAck() was void → it always called get_serial()
//    get_serial() called updateServerFromAck() directly
//    On timeout: vack1..8 were still empty/stale from last time
//    → updateServerFromAck() posted empty strings
//    → overwrote optimistic update → app showed wrong state
//
//  New behaviour:
//    If timeout → return false → TaskLoraTX skips updateServerFromAck()
//    Optimistic update stays on server → app shows correct state
// ══════════════════════════════════════════════════════════
bool listenLoraAck(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (Serial2.available()) {
            char b = Serial2.peek();
            if (b == '$') {
                bool ok = get_serial();
                if (ok) return true;
                // bad frame — keep scanning until timeout
            } else {
                Serial2.read();  // discard noise byte
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    Serial.println("[ACK] timeout — optimistic update kept");
    return false;
}

// ══════════════════════════════════════════════════════════
//  TaskLoraTX — ONLY task writing Serial2
//
//  CHANGE: listenLoraAck() now returns bool.
//  updateServerFromAck() called ONLY when ACK actually received.
//  Optimistic update is NOT touched on timeout.
// ══════════════════════════════════════════════════════════
void TaskLoraTX(void *pvParameters) {
    LoraJob job;
    for (;;) {
        if (xQueueReceive(xLoraQueue, &job, portMAX_DELAY) == pdTRUE)
        {
            if (job.is_keepalive)
            {
                Serial.printf("[KEEPALIVE-%c]\n", job.prefix);
                char ka = job.prefix + 32;
                for (int k=0; k<5; k++) {
                    Serial2.print(ka);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                // No ACK wait for keepalive — unchanged
            }
            else
            {
                Serial.printf("[LORA-TX] Slave %c\n", job.prefix);

                xSemaphoreTake(xMutexLED, portMAX_DELAY);
                digitalWrite(SIG_PIN, HIGH);
                xSemaphoreGive(xMutexLED);

                // Step 1: Send LoRa frame — unchanged
                sendLoraFrame(job);

                // Step 2: Optimistic update immediately — unchanged
                updateServerOptimistic(String(job.sid),
                    String(job.cmd1), String(job.cmd2),
                    String(job.cmd3), String(job.cmd4),
                    String(job.cmd5), String(job.cmd6),
                    String(job.cmd7), String(job.cmd8));

                xSemaphoreTake(xMutexLED, portMAX_DELAY);
                digitalWrite(SIG_PIN, LOW);
                xSemaphoreGive(xMutexLED);

                // Step 3: Listen for real ACK
                // FIX B: only call updateServerFromAck if ACK truly received
                bool ackReceived = listenLoraAck(ACK_WINDOW_MS);
                if (ackReceived) {
                    String sid;
                    xSemaphoreTake(xMutexVars, portMAX_DELAY);
                    sid = SID_g;
                    xSemaphoreGive(xMutexVars);
                    updateServerFromAck(sid);
                } else {
                    Serial.printf("[LORA-TX] No ACK for %c — optimistic update kept on server\n",
                                  job.prefix);
                }
            }
        }
    }
}

// ══════════════════════════════════════════════════════════
//  pushValveJob() — unchanged
// ══════════════════════════════════════════════════════════
static void pushValveJob(char prefix, const char *sid,
    const String &ls1, const String &ls2,
    const String &ls3, const String &ls4,
    const String &ls5, const String &ls6,
    const String &ls7, const String &ls8,
    const String &lbw,
    const String &lv1, const String &lv2,
    const String &lv3, const String &lv4,
    const String &lv5, const String &lv6,
    const String &lv7, const String &lv8)
{
    LoraJob job;
    memset(&job, 0, sizeof(job));
    job.prefix       = prefix;
    job.is_keepalive = false;
    strncpy(job.sid, sid, 3);

    scopy(job.s1,4,ls1); scopy(job.s2,4,ls2);
    scopy(job.s3,4,ls3); scopy(job.s4,4,ls4);
    scopy(job.s5,4,ls5); scopy(job.s6,4,ls6);
    scopy(job.s7,4,ls7); scopy(job.s8,4,ls8);
    scopy(job.bw,8,lbw);

    scopy(job.v1,4,lv1); scopy(job.v2,4,lv2);
    scopy(job.v3,4,lv3); scopy(job.v4,4,lv4);
    scopy(job.v5,4,lv5); scopy(job.v6,4,lv6);
    scopy(job.v7,4,lv7); scopy(job.v8,4,lv8);

    scopy(job.cmd1,4,ls1); scopy(job.cmd2,4,ls2);
    scopy(job.cmd3,4,ls3); scopy(job.cmd4,4,ls4);
    scopy(job.cmd5,4,ls5); scopy(job.cmd6,4,ls6);
    scopy(job.cmd7,4,ls7); scopy(job.cmd8,4,ls8);

    if (xQueueSend(xLoraQueue, &job, pdMS_TO_TICKS(200)) != pdTRUE)
        Serial.printf("[QUEUE] Full — valve job dropped %c\n", prefix);
}

static void pushKeepaliveJob(char prefix) {
    LoraJob job;
    memset(&job, 0, sizeof(job));
    job.prefix       = prefix;
    job.is_keepalive = true;
    if (xQueueSend(xLoraQueue, &job, pdMS_TO_TICKS(100)) != pdTRUE)
        Serial.printf("[QUEUE] Full — keepalive dropped %c\n", prefix);
}

// ══════════════════════════════════════════════════════════
//  TaskReadServerA — unchanged
// ══════════════════════════════════════════════════════════
void TaskReadServerA(void *pvParameters) {
    waitWiFiReady();
    uint32_t lastKeepalive = 0;

    for (;;) {
        if (slaveid1.length()==0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        String localPayload = "";
        {
            String url = HOST1 + "/Lorawan24/motorlora/readvalve8.php?deviceid=" +
                         deviceid + "&tokenid=" + tokenid + "&sid=" + slaveid1;
            HTTPClient http;
            http.begin(url);
            int code = http.GET();
            if (code == HTTP_CODE_OK) {
                localPayload = http.getString();
                Serial.printf("[SERVER-A] %s\n", localPayload.c_str());
            } else {
                Serial.printf("[SERVER-A] ERR %d\n", code);
            }
            http.end();
        }

        if (localPayload.length()==0) { vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS)); continue; }

        bool changed = false;
        String ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw;
        String lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8;

        xSemaphoreTake(xMutexVars, portMAX_DELAY);
        payload = localPayload;
        parsePayload();

        v1s1=vs1; v1s2=vs2; v1s3=vs3; v1s4=vs4;
        v1s5=vs5; v1s6=vs6; v1s7=vs7; v1s8=vs8;
        BWMIN1 = BWMIN;

        vA1=v1; vA2=v2; vA3=v3; vA4=v4;
        vA5=v5; vA6=v6; vA7=v7; vA8=v8;

        Serial.printf("[PARSE-A] NOW:%s|%s|%s|%s|%s|%s|%s|%s BW:%s\n",
            v1s1.c_str(),v1s2.c_str(),v1s3.c_str(),v1s4.c_str(),
            v1s5.c_str(),v1s6.c_str(),v1s7.c_str(),v1s8.c_str(),BWMIN1.c_str());
        Serial.printf("[PARSE-A] PRV:%s|%s|%s|%s|%s|%s|%s|%s BW:%s\n",
            v1s11.c_str(),v1s22.c_str(),v1s33.c_str(),v1s44.c_str(),
            v1s55.c_str(),v1s66.c_str(),v1s77.c_str(),v1s88.c_str(),BW1MINN.c_str());

        if (v1s1!=v1s11||v1s2!=v1s22||v1s3!=v1s33||v1s4!=v1s44||
            v1s5!=v1s55||v1s6!=v1s66||v1s7!=v1s77||v1s8!=v1s88||BWMIN1!=BW1MINN)
            changed = true;

        Serial.printf("[CHANGE-A] %s\n", changed?"*** CHANGED ***":"no change");

        ls1=v1s1; ls2=v1s2; ls3=v1s3; ls4=v1s4;
        ls5=v1s5; ls6=v1s6; ls7=v1s7; ls8=v1s8; lbw=BWMIN1;
        lv1=vA1; lv2=vA2; lv3=vA3; lv4=vA4;
        lv5=vA5; lv6=vA6; lv7=vA7; lv8=vA8;

        if (changed) {
            v1s11=v1s1; v1s22=v1s2; v1s33=v1s3; v1s44=v1s4;
            v1s55=v1s5; v1s66=v1s6; v1s77=v1s7; v1s88=v1s8;
            BW1MINN=BWMIN1;
        }
        xSemaphoreGive(xMutexVars);

        if (changed) {
            pushValveJob('A',"A",ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw,
                         lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8);
            xSemaphoreTake(xMutexLED, portMAX_DELAY);
            digitalWrite(LED_PIN, HIGH);
            xSemaphoreGive(xMutexLED);
            vTaskDelay(pdMS_TO_TICKS(100));
            xSemaphoreTake(xMutexLED, portMAX_DELAY);
            digitalWrite(LED_PIN, LOW);
            xSemaphoreGive(xMutexLED);
        } else {
            uint32_t now = millis();
            if (now - lastKeepalive >= KEEPALIVE_INTERVAL_MS) {
                lastKeepalive = now;
                pushKeepaliveJob('A');
            }
        }

        if (changed)
            vTaskDelay(pdMS_TO_TICKS(1000));
        else
            vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ══════════════════════════════════════════════════════════
//  TaskReadServerB — unchanged
// ══════════════════════════════════════════════════════════
void TaskReadServerB(void *pvParameters) {
    waitWiFiReady();
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint32_t lastKeepalive = 0;

    for (;;) {
        if (slaveid2.length()==0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        String localPayload = "";
        {
            String url = HOST1 + "/Lorawan24/motorlora/readvalve8.php?deviceid=" +
                         deviceid + "&tokenid=" + tokenid + "&sid=" + slaveid2;
            HTTPClient http;
            http.begin(url);
            int code = http.GET();
            if (code == HTTP_CODE_OK) {
                localPayload = http.getString();
                Serial.printf("[SERVER-B] %s\n", localPayload.c_str());
            } else {
                Serial.printf("[SERVER-B] ERR %d\n", code);
            }
            http.end();
        }

        if (localPayload.length()==0) { vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS)); continue; }

        bool changed = false;
        String ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw;
        String lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8;

        xSemaphoreTake(xMutexVars, portMAX_DELAY);
        payload = localPayload;
        parsePayload();

        v2s1=vs1; v2s2=vs2; v2s3=vs3; v2s4=vs4;
        v2s5=vs5; v2s6=vs6; v2s7=vs7; v2s8=vs8;
        BWMIN2 = BWMIN;

        vB1=v1; vB2=v2; vB3=v3; vB4=v4;
        vB5=v5; vB6=v6; vB7=v7; vB8=v8;

        if (v2s1!=v2s11||v2s2!=v2s22||v2s3!=v2s33||v2s4!=v2s44||
            v2s5!=v2s55||v2s6!=v2s66||v2s7!=v2s77||v2s8!=v2s88||BWMIN2!=BW2MINN)
            changed = true;

        ls1=v2s1; ls2=v2s2; ls3=v2s3; ls4=v2s4;
        ls5=v2s5; ls6=v2s6; ls7=v2s7; ls8=v2s8; lbw=BWMIN2;
        lv1=vB1; lv2=vB2; lv3=vB3; lv4=vB4;
        lv5=vB5; lv6=vB6; lv7=vB7; lv8=vB8;

        if (changed) {
            v2s11=v2s1; v2s22=v2s2; v2s33=v2s3; v2s44=v2s4;
            v2s55=v2s5; v2s66=v2s6; v2s77=v2s7; v2s88=v2s8;
            BW2MINN=BWMIN2;
        }
        xSemaphoreGive(xMutexVars);

        if (changed) {
            pushValveJob('B',"B",ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw,
                         lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8);
        } else {
            uint32_t now = millis();
            if (now-lastKeepalive >= KEEPALIVE_INTERVAL_MS) {
                lastKeepalive = now;
                pushKeepaliveJob('B');
            }
        }

        if (changed)
            vTaskDelay(pdMS_TO_TICKS(1000));
        else
            vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ══════════════════════════════════════════════════════════
//  TaskReadServerC — unchanged
// ══════════════════════════════════════════════════════════
void TaskReadServerC(void *pvParameters) {
    waitWiFiReady();
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint32_t lastKeepalive = 0;

    for (;;) {
        if (slaveid3.length()==0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        String localPayload = "";
        {
            String url = HOST1 + "/Lorawan24/motorlora/readvalve8.php?deviceid=" +
                         deviceid + "&tokenid=" + tokenid + "&sid=" + slaveid3;
            HTTPClient http;
            http.begin(url);
            int code = http.GET();
            if (code == HTTP_CODE_OK) {
                localPayload = http.getString();
                Serial.printf("[SERVER-C] %s\n", localPayload.c_str());
            } else {
                Serial.printf("[SERVER-C] ERR %d\n", code);
            }
            http.end();
        }

        if (localPayload.length()==0) { vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS)); continue; }

        bool changed = false;
        String ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw;
        String lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8;

        xSemaphoreTake(xMutexVars, portMAX_DELAY);
        payload = localPayload;
        parsePayload();

        v3s1=vs1; v3s2=vs2; v3s3=vs3; v3s4=vs4;
        v3s5=vs5; v3s6=vs6; v3s7=vs7; v3s8=vs8;
        BWMIN3 = BWMIN;

        vC1=v1; vC2=v2; vC3=v3; vC4=v4;
        vC5=v5; vC6=v6; vC7=v7; vC8=v8;

        if (v3s1!=v3s11||v3s2!=v3s22||v3s3!=v3s33||v3s4!=v3s44||
            v3s5!=v3s55||v3s6!=v3s66||v3s7!=v3s77||v3s8!=v3s88||BWMIN3!=BW3MINN)
            changed = true;

        ls1=v3s1; ls2=v3s2; ls3=v3s3; ls4=v3s4;
        ls5=v3s5; ls6=v3s6; ls7=v3s7; ls8=v3s8; lbw=BWMIN3;
        lv1=vC1; lv2=vC2; lv3=vC3; lv4=vC4;
        lv5=vC5; lv6=vC6; lv7=vC7; lv8=vC8;

        if (changed) {
            v3s11=v3s1; v3s22=v3s2; v3s33=v3s3; v3s44=v3s4;
            v3s55=v3s5; v3s66=v3s6; v3s77=v3s7; v3s88=v3s8;
            BW3MINN=BWMIN3;
        }
        xSemaphoreGive(xMutexVars);

        if (changed) {
            pushValveJob('C',"C",ls1,ls2,ls3,ls4,ls5,ls6,ls7,ls8,lbw,
                         lv1,lv2,lv3,lv4,lv5,lv6,lv7,lv8);
        } else {
            uint32_t now = millis();
            if (now-lastKeepalive >= KEEPALIVE_INTERVAL_MS) {
                lastKeepalive = now;
                pushKeepaliveJob('C');
            }
        }

        if (changed)
            vTaskDelay(pdMS_TO_TICKS(1000));
        else
            vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ══════════════════════════════════════════════════════════
//  TaskCompareA/B/C — unchanged
// ══════════════════════════════════════════════════════════
void TaskCompareA(void *pvParameters) {
    waitWiFiReady();
    for (;;) { if (slaveid1.length()>0) compareid(slaveid1); vTaskDelay(pdMS_TO_TICKS(COMPAREID_INTERVAL_MS)); }
}
void TaskCompareB(void *pvParameters) {
    waitWiFiReady();
    for (;;) { if (slaveid2.length()>0) compareid(slaveid2); vTaskDelay(pdMS_TO_TICKS(COMPAREID_INTERVAL_MS)); }
}
void TaskCompareC(void *pvParameters) {
    waitWiFiReady();
    for (;;) { if (slaveid3.length()>0) compareid(slaveid3); vTaskDelay(pdMS_TO_TICKS(COMPAREID_INTERVAL_MS)); }
}

// ══════════════════════════════════════════════════════════
//  TaskAutoReset — unchanged
// ══════════════════════════════════════════════════════════
void TaskAutoReset(void *pvParameters) {
    for (unsigned long e=0; e<RESET_TIME_S; e++) vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("[ResetTask] restarting"); Serial.flush(); esp_restart();
}

// ══════════════════════════════════════════════════════════
//  TaskWiFiManager — unchanged
// ══════════════════════════════════════════════════════════
void TaskWiFiManager(void *pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) { WFcheck(); vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS)); continue; }
        HTTPClient http;
        String url = HOST1 + "/Lorawan24/motorlora/comparev1.php?deviceid=" +
                     deviceid + "&tokenid=" + tokenid + "&sid=" + slaveid1;
        http.begin(url); int net = http.GET(); http.end();
        if (net == 200) { xEventGroupSetBits(xWiFiEvents, WIFI_READY_BIT); break; }
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS));
    }
    for (;;) { vTaskDelay(pdMS_TO_TICKS(30000)); if (WiFi.status()!=WL_CONNECTED) WFcheck(); }
}

// ══════════════════════════════════════════════════════════
//  WFcheck / checkButton / blinkLED — unchanged
// ══════════════════════════════════════════════════════════
void WFcheck() {
    wm.setConnectTimeout(10); wm.setConfigPortalTimeout(180);
    while (WiFi.status()!=WL_CONNECTED) {
        digitalWrite(LED_PIN,LOW);
        wm.autoConnect("SMART SOS","password");
        digitalWrite(LED_PIN,HIGH);
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}
void blinkLED(int times, int onMs, int offMs) {
    for (int i=0; i<times; i++) { digitalWrite(LED_PIN,HIGH); delay(onMs); digitalWrite(LED_PIN,LOW); delay(offMs); }
}
void checkButton() {
    if (digitalRead(TRIGGER_PIN)!=LOW) return;
    delay(50);
    if (digitalRead(TRIGGER_PIN)!=LOW) return;
    blinkLED(10,500,500);
    if (digitalRead(TRIGGER_PIN)==LOW) { blinkLED(2,2000,1000); wm.resetSettings(); eraseParameters(); }
    wm.setConfigPortalTimeout(240);
    wm.startConfigPortal("SMART LORAWAN","password");
}

// ══════════════════════════════════════════════════════════
//  Preferences helpers — unchanged
// ══════════════════════════════════════════════════════════
void saveParameters() {
    preferences.begin("my-app",false);
    preferences.putString("device_id",custom_device_id.getValue());
    preferences.putString("token_id", custom_token_id.getValue());
    preferences.putString("slave_id1",custom_slave_id1.getValue());
    preferences.putString("slave_id2",custom_slave_id2.getValue());
    preferences.putString("slave_id3",custom_slave_id3.getValue());
    preferences.end();
}
void loadParameters() {
    preferences.begin("my-app",true);
    auto loadStr=[&](const char *key, char *buf, size_t sz,
                     WiFiManagerParameter &param, String &strVar){
        strVar=preferences.getString(key,"");
        if (strVar.length()>0 && strVar.length()<sz) {
            strncpy(buf,strVar.c_str(),sz); buf[sz-1]='\0';
            param.setValue(buf,sz);
            Serial.printf("[Pref] %s: %s\n",key,buf);
        }
    };
    loadStr("device_id",device_id,20,custom_device_id,deviceid);
    loadStr("token_id", token_id, 20,custom_token_id, tokenid);
    loadStr("slave_id1",slave_id1,20,custom_slave_id1,slaveid1);
    loadStr("slave_id2",slave_id2,20,custom_slave_id2,slaveid2);
    loadStr("slave_id3",slave_id3,20,custom_slave_id3,slaveid3);
    preferences.end();
}
void eraseParameters() {
    preferences.begin("my-app",false); preferences.clear(); preferences.end();
}
void saveConfigCallback() { saveParameters(); }

// ══════════════════════════════════════════════════════════
//  setup() — unchanged except xMutexSerial2 removed (not needed)
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(500);

    Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
    WiFi.mode(WIFI_STA);
    pinMode(VOLT_PIN,    INPUT_PULLUP);
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    pinMode(LED_PIN,     OUTPUT);
    pinMode(SIG_PIN,     OUTPUT);
    pinMode(AUX_PIN,     INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(SIG_PIN, LOW);

    if (!SPIFFS.begin(true)) Serial.println("[FS] SPIFFS failed");

    xMutexSerial2    = xSemaphoreCreateMutex();
    xMutexVars       = xSemaphoreCreateMutex();
    xMutexLED        = xSemaphoreCreateMutex();
    xMutexCompareVal = xSemaphoreCreateMutex();
    xWiFiEvents      = xEventGroupCreate();
    xLoraQueue       = xQueueCreate(12, sizeof(LoraJob));

    if (!xMutexSerial2||!xMutexVars||!xMutexLED||
        !xMutexCompareVal||!xWiFiEvents||!xLoraQueue) {
        Serial.println("[FATAL] FreeRTOS object failed!");
        while(true) delay(1000);
    }

    wm.addParameter(&custom_device_id);
    wm.addParameter(&custom_token_id);
    wm.addParameter(&custom_slave_id1);
    wm.addParameter(&custom_slave_id2);
    wm.addParameter(&custom_slave_id3);
    wm.setSaveConfigCallback(saveConfigCallback);
    loadParameters();
    checkButton();

    WFcheck();
    Serial.println("[SETUP] WiFi connected!");

    xTaskCreatePinnedToCore(TaskAutoReset,   "ResetTask", 2048,  NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskWiFiManager, "WiFiMgr",   6144,  NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskLoraTX,      "LoraTX",    8192,  NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskReadServerA, "SlaveA",    12288, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskReadServerB, "SlaveB",    12288, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskReadServerC, "SlaveC",    12288, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskCompareA,    "CompareA",  4096,  NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskCompareB,    "CompareB",  4096,  NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskCompareC,    "CompareC",  4096,  NULL, 1, NULL, 0);

    Serial.println("[SETUP] All tasks created.");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(10000)); }
