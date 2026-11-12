// CYD weather clock -> ESP32-S3 4.3" 800x480 (LovyanGFX + LVGL 8 + GT911)
// Logic ported from Garionhk/CYD-2.8-Wifi-Weather-Clock (Apache-2.0)
#define LGFX_USE_V1
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include "config.h"

// ---------- 1. จอ ----------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_RGB _panel_instance;
  lgfx::Bus_RGB _bus_instance;
  lgfx::Light_PWM _light_instance;
public:
  LGFX(void) {
    { auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      cfg.pin_d0 = GPIO_NUM_8;  cfg.pin_d1 = GPIO_NUM_3;  cfg.pin_d2 = GPIO_NUM_46; cfg.pin_d3 = GPIO_NUM_9;
      cfg.pin_d4 = GPIO_NUM_1;  cfg.pin_d5 = GPIO_NUM_5;  cfg.pin_d6 = GPIO_NUM_6;  cfg.pin_d7 = GPIO_NUM_7;
      cfg.pin_d8 = GPIO_NUM_15; cfg.pin_d9 = GPIO_NUM_16; cfg.pin_d10 = GPIO_NUM_4; cfg.pin_d11 = GPIO_NUM_45;
      cfg.pin_d12 = GPIO_NUM_48; cfg.pin_d13 = GPIO_NUM_47; cfg.pin_d14 = GPIO_NUM_21; cfg.pin_d15 = GPIO_NUM_14;
      cfg.pin_henable = GPIO_NUM_40; cfg.pin_vsync = GPIO_NUM_41; cfg.pin_hsync = GPIO_NUM_39; cfg.pin_pclk = GPIO_NUM_42;
      cfg.freq_write = 14000000;
      cfg.hsync_polarity = 0; cfg.hsync_front_porch = 8; cfg.hsync_pulse_width = 4; cfg.hsync_back_porch = 8;
      cfg.vsync_polarity = 0; cfg.vsync_front_porch = 8; cfg.vsync_pulse_width = 4; cfg.vsync_back_porch = 8;
      cfg.pclk_idle_high = 1;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance); }
    { auto cfg = _panel_instance.config();
      cfg.memory_width = 800; cfg.memory_height = 480;
      cfg.panel_width = 800;  cfg.panel_height = 480;
      cfg.offset_x = 0; cfg.offset_y = 0;
      _panel_instance.config(cfg); }
    { auto cfg = _light_instance.config();
      cfg.pin_bl = GPIO_NUM_2; cfg.freq = 1000; cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); }
    setPanel(&_panel_instance);
  }
};
LGFX lcd;

// ---------- 2. ทัช GT911 (Wire ตรง) ----------
#define TOUCH_SDA 19
#define TOUCH_SCL 20
#define TOUCH_INT 18
#define TOUCH_RST 38
#define GT_MAX_X 480
#define GT_MAX_Y 272
#define GT_FLIP_X 1
#define GT_FLIP_Y 1
static uint8_t gtAddr = 0;

static bool gtRead(uint16_t reg, uint8_t *b, uint8_t len) {
  Wire.beginTransmission(gtAddr); Wire.write(reg >> 8); Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)gtAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) b[i] = Wire.read();
  return true;
}
static void gtWrite(uint16_t reg, uint8_t v) {
  Wire.beginTransmission(gtAddr); Wire.write(reg >> 8); Wire.write(reg & 0xFF); Wire.write(v);
  Wire.endTransmission();
}
static void gtInit() {
  Wire.setClock(400000);
  const uint8_t cand[2] = {0x5D, 0x14};
  uint8_t id[4];
  for (uint8_t k = 0; k < 2; k++) {
    gtAddr = cand[k];
    if (gtRead(0x8140, id, 4)) { Serial.printf("GT911 0x%02X\n", gtAddr); return; }
  }
  gtAddr = 0; Serial.println("GT911 not found");
}
static bool gtGetPoint(int &x, int &y) {
  static bool touched = false; static int lx = 0, ly = 0;
  if (gtAddr) {
    uint8_t st;
    if (gtRead(0x814E, &st, 1) && (st & 0x80)) {
      uint8_t n = st & 0x0F;
      if (n > 0 && n <= 5) {
        uint8_t p[4];
        if (gtRead(0x8150, p, 4)) { lx = p[0] | (p[1] << 8); ly = p[2] | (p[3] << 8); touched = true; }
      } else touched = false;
      gtWrite(0x814E, 0);
    }
  }
  x = lx; y = ly; return touched;
}

// ---------- 3. LVGL bridge ----------
static const uint16_t SW = 800, SH = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *lvbuf;

static void my_disp_flush(lv_disp_drv_t *d, const lv_area_t *a, lv_color_t *c) {
  lcd.pushImage(a->x1, a->y1, a->x2 - a->x1 + 1, a->y2 - a->y1 + 1, (lgfx::rgb565_t *)&c->full);
  lv_disp_flush_ready(d);
}
static void my_touch_read(lv_indev_drv_t *d, lv_indev_data_t *data) {
  int rx, ry;
  if (gtGetPoint(rx, ry)) {
    int x = (long)rx * 800 / GT_MAX_X, y = (long)ry * 480 / GT_MAX_Y;
    if (GT_FLIP_X) x = 799 - x;
    if (GT_FLIP_Y) y = 479 - y;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = constrain(x, 0, 799);
    data->point.y = constrain(y, 0, 479);
  } else data->state = LV_INDEV_STATE_REL;
}

// ---------- 4. ข้อมูล (เขียนโดย fetch task, อ่านโดย UI) ----------
struct AppData {
  float tempC = 0, feelsC = 0, hiC = 0, loC = 0, windKph = 0, pressHpa = 0, uv = 0;
  int code = 0, cloud = 0, hum = 0, aqi = 0, pm25 = 0, pm10 = 0;
  bool wxValid = false, aqValid = false;
  uint32_t wxAt = 0, aqAt = 0;
  time_t sunrise = 0, sunset = 0, nextRise = 0, nextSet = 0;
  float moonPhase = 0, moonIllum = 0;
};
static AppData g;

#if USE_IMPERIAL
static float dTemp(float c) { return c * 9 / 5 + 32; }
static float dWind(float k) { return k * 0.621371f; }
static float dPress(float h) { return h * 0.02953f; }
static const char *U_T = "F", *U_W = "mph", *U_P = "inHg";
#else
static float dTemp(float c) { return c; }
static float dWind(float k) { return k; }
static float dPress(float h) { return h; }
static const char *U_T = "C", *U_W = "km/h", *U_P = "hPa";
#endif

// ---------- 5. Sun / Moon (NOAA) ----------
static double d2r(double d) { return d * M_PI / 180.0; }
static double r2d(double r) { return r * 180.0 / M_PI; }
static double julianDay(int y, int m, int d) {
  if (m <= 2) { y--; m += 12; }
  int A = y / 100, B = 2 - A + A / 4;
  return floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + d + B - 1524.5;
}
static time_t sunEventUTC(int y, int m, int d, double elev, bool morning) {
  double JD = julianDay(y, m, d), T = (JD - 2451545.0) / 36525.0;
  double L0 = fmod(280.46646 + T * (36000.76983 + 0.0003032 * T), 360.0);
  double M = 357.52911 + T * (35999.05029 - 0.0001537 * T);
  double e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T);
  double Mr = d2r(M);
  double C = sin(Mr) * (1.914602 - T * (0.004817 + 0.000014 * T)) + sin(2 * Mr) * (0.019993 - 0.000101 * T) + sin(3 * Mr) * 0.000289;
  double omega = 125.04 - 1934.136 * T;
  double lambda = L0 + C - 0.00569 - 0.00478 * sin(d2r(omega));
  double eps0 = 23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60.0) / 60.0;
  double eps = eps0 + 0.00256 * cos(d2r(omega));
  double delta = r2d(asin(sin(d2r(eps)) * sin(d2r(lambda))));
  double yv = tan(d2r(eps / 2.0)); yv *= yv;
  double eqT = 4.0 * r2d(yv * sin(2 * d2r(L0)) - 2 * e * sin(Mr) + 4 * e * yv * sin(Mr) * cos(2 * d2r(L0)) - 0.5 * yv * yv * sin(4 * d2r(L0)) - 1.25 * e * e * sin(2 * Mr));
  double lat = LATITUDE, lon = LONGITUDE;
  double cosH = (cos(d2r(90.0 - elev)) - sin(d2r(lat)) * sin(d2r(delta))) / (cos(d2r(lat)) * cos(d2r(delta)));
  if (cosH > 1.0 || cosH < -1.0) return 0;
  double H = r2d(acos(cosH));
  double minUTC = 720.0 - 4.0 * (lon + (morning ? H : -H)) - eqT;
  return (time_t)llround((JD - 2440587.5) * 86400.0 + minUTC * 60.0);
}
static const char *moonName(float p) {
  if (p < 0.03f || p > 0.97f) return "New Moon";
  if (p < 0.22f) return "Waxing Crescent";
  if (p < 0.28f) return "First Quarter";
  if (p < 0.47f) return "Waxing Gibbous";
  if (p < 0.53f) return "Full Moon";
  if (p < 0.72f) return "Waning Gibbous";
  if (p < 0.78f) return "Last Quarter";
  return "Waning Crescent";
}
static void sunmoonRecompute() {
  struct tm t;
  if (!getLocalTime(&t, 10)) return;
  int y = t.tm_year + 1900, mo = t.tm_mon + 1, d = t.tm_mday;
  g.sunrise = sunEventUTC(y, mo, d, -0.833, true);
  g.sunset = sunEventUTC(y, mo, d, -0.833, false);
  struct tm n = t; n.tm_mday++; n.tm_hour = 12; n.tm_min = 0; n.tm_sec = 0;
  time_t tt = mktime(&n); struct tm o; localtime_r(&tt, &o);
  g.nextRise = sunEventUTC(o.tm_year + 1900, o.tm_mon + 1, o.tm_mday, -0.833, true);
  g.nextSet = sunEventUTC(o.tm_year + 1900, o.tm_mon + 1, o.tm_mday, -0.833, false);
  double JDn = 2440587.5 + (double)time(nullptr) / 86400.0;
  double ph = (JDn - 2451550.1) / 29.530588853; ph -= floor(ph);
  g.moonPhase = (float)ph;
  g.moonIllum = (float)((1.0 - cos(2 * M_PI * ph)) / 2.0 * 100.0);
}

// ---------- 6. Fetch (FreeRTOS task, ไม่บล็อก UI) ----------
static bool httpGet(const String &url, String &out) {
  WiFiClientSecure cl; cl.setInsecure();
  HTTPClient h;
  if (!h.begin(cl, url)) return false;
  int c = h.GET();
  if (c != 200) { Serial.printf("HTTP %d\n", c); h.end(); return false; }
  out = h.getString(); h.end(); return true;
}
static bool fetchWeather() {
  String p;
  if (!httpGet(String("https://api.open-meteo.com/v1/forecast?latitude=") + String(LATITUDE, 4) + "&longitude=" + String(LONGITUDE, 4) +
               "&current=temperature_2m,apparent_temperature,weather_code,cloud_cover,relative_humidity_2m,wind_speed_10m,surface_pressure,uv_index"
               "&daily=temperature_2m_max,temperature_2m_min&forecast_days=1&timezone=auto", p)) return false;
  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, p)) return false;
  JsonObject c = doc["current"]; if (c.isNull()) return false;
  g.tempC = c["temperature_2m"] | g.tempC;
  g.feelsC = c["apparent_temperature"] | g.feelsC;
  g.code = c["weather_code"] | g.code;
  g.cloud = c["cloud_cover"] | g.cloud;
  g.hum = c["relative_humidity_2m"] | g.hum;
  g.windKph = c["wind_speed_10m"] | g.windKph;
  g.pressHpa = c["surface_pressure"] | g.pressHpa;
  g.uv = c["uv_index"] | g.uv;
  JsonArray hi = doc["daily"]["temperature_2m_max"], lo = doc["daily"]["temperature_2m_min"];
  if (hi.size() && lo.size()) { g.hiC = hi[0] | g.hiC; g.loC = lo[0] | g.loC; }
  g.wxAt = (uint32_t)time(nullptr); g.wxValid = true;
  Serial.printf("wx %.1fC code %d\n", g.tempC, g.code);
  return true;
}
static bool fetchAir() {
  String p;
  if (!httpGet(String("https://air-quality-api.open-meteo.com/v1/air-quality?latitude=") + String(LATITUDE, 4) + "&longitude=" + String(LONGITUDE, 4) +
               "&current=us_aqi,pm2_5,pm10&timezone=auto", p)) return false;
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, p)) return false;
  JsonObject c = doc["current"]; if (c.isNull() || c["us_aqi"].isNull()) return false;
  g.aqi = c["us_aqi"] | g.aqi;
  g.pm25 = (int)lroundf(c["pm2_5"] | (float)g.pm25);
  g.pm10 = (int)lroundf(c["pm10"] | (float)g.pm10);
  g.aqAt = (uint32_t)time(nullptr); g.aqValid = true;
  Serial.printf("aqi %d pm2.5 %d\n", g.aqi, g.pm25);
  return true;
}
static void netTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTzTime(TZ_POSIX, "pool.ntp.org", "time.nist.gov");
  uint32_t nextWx = 0, nextAq = 5000, backWx = 0, backAq = 0;
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      uint32_t now = millis();
      if ((int32_t)(now - nextWx) >= 0) {
        if (fetchWeather()) { backWx = 0; nextWx = millis() + 15UL * 60000; }
        else { backWx = backWx ? min(backWx * 2, 15u) : 1; nextWx = millis() + backWx * 60000UL; }
      }
      if ((int32_t)(millis() - nextAq) >= 0) {
        if (fetchAir()) { backAq = 0; nextAq = millis() + 30UL * 60000; }
        else { backAq = backAq ? min(backAq * 2, 15u) : 1; nextAq = millis() + backAq * 60000UL; }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


// ---------- 7. UI ----------
LV_FONT_DECLARE(font_th_16);
LV_FONT_DECLARE(font_th_22);
LV_FONT_DECLARE(font_th_30);
LV_FONT_DECLARE(font_clock);
LV_FONT_DECLARE(font_temp);

static const char *wmo(int c) {
  switch (c) {
    case 0: return "ท้องฟ้าแจ่มใส"; case 1: return "ฟ้าโปร่ง"; case 2: return "มีเมฆบางส่วน"; case 3: return "เมฆมาก";
    case 45: case 48: return "หมอก";
    case 51: case 53: case 55: return "ฝนปรอย";
    case 56: case 57: return "ฝนปรอยเยือกแข็ง";
    case 61: return "ฝนเล็กน้อย"; case 63: return "ฝนตก"; case 65: return "ฝนตกหนัก";
    case 66: case 67: return "ฝนเยือกแข็ง";
    case 71: case 73: case 75: case 77: return "หิมะตก";
    case 80: case 81: case 82: return "ฝนตกเป็นช่วง";
    case 85: case 86: return "หิมะตกเป็นช่วง";
    case 95: return "พายุฟ้าคะนอง"; case 96: case 99: return "พายุฟ้าคะนอง ลูกเห็บ";
  }
  return "รอข้อมูล";
}
static const char *aqiCat(int a, uint32_t &col) {
  if (a <= 50)  { col = 0x38D996; return "ดี"; }
  if (a <= 100) { col = 0xF4D35E; return "ปานกลาง"; }
  if (a <= 150) { col = 0xFF9F43; return "เสี่ยงในกลุ่มเปราะบาง"; }
  if (a <= 200) { col = 0xFF5D73; return "ไม่ดีต่อสุขภาพ"; }
  if (a <= 300) { col = 0xB084F5; return "ไม่ดีต่อสุขภาพมาก"; }
  col = 0xFF4D6D; return "อันตราย";
}
static const char *moonNameTh(float p) {
  if (p < 0.03f || p > 0.97f) return "จันทร์ดับ";
  if (p < 0.22f) return "ข้างขึ้นเสี้ยว";
  if (p < 0.28f) return "ครึ่งดวงข้างขึ้น";
  if (p < 0.47f) return "ข้างขึ้นเกือบเต็ม";
  if (p < 0.53f) return "จันทร์เต็มดวง";
  if (p < 0.72f) return "ข้างแรมเกือบเต็ม";
  if (p < 0.78f) return "ครึ่งดวงข้างแรม";
  return "ข้างแรมเสี้ยว";
}
static const char *TH_DAY[7] = {"อาทิตย์", "จันทร์", "อังคาร", "พุธ", "พฤหัสบดี", "ศุกร์", "เสาร์"};
static const char *TH_MON[12] = {"มกราคม", "กุมภาพันธ์", "มีนาคม", "เมษายน", "พฤษภาคม", "มิถุนายน",
                                 "กรกฎาคม", "สิงหาคม", "กันยายน", "ตุลาคม", "พฤศจิกายน", "ธันวาคม"};

#define NSC 4
static lv_obj_t *sc[NSC];
static lv_obj_t *dots[NSC];
static int cur = 0;

// Home
static lv_obj_t *l_time, *l_sec, *l_date, *l_ctemp, *l_ccond, *l_chilo, *l_cstat, *ico_home;
static lv_obj_t *l_homeSun, *l_homeSet, *l_homeDay, *l_haqi, *l_homeAir;
// Weather
static lv_obj_t *l_wtemp, *l_wcond, *l_wfeel, *l_whilo, *l_wdet, *ico_wx;
// Sun and moon
static lv_obj_t *l_rise, *l_set, *l_day, *l_moon, *l_moonp, *arc_moon, *bar_day;
// Air quality
static lv_obj_t *l_aqi, *l_aqicat, *l_pm, *l_pm10, *l_uv, *bar_aqi;
static lv_obj_t *l_hpm;

// Theme
static const uint32_t C_BG = 0x0A0E1A;
static const uint32_t C_CARD = 0x141B2D;
static const uint32_t C_CARD2 = 0x1A2338;
static const uint32_t C_BORDER = 0x28344F;
static const uint32_t C_TRACK = 0x2B3750;
static const uint32_t C_TEXT = 0xF5F7FB;
static const uint32_t C_MUTED = 0x8B97AD;
static const uint32_t C_AMBER = 0xFFB86B;
static const uint32_t C_TEAL = 0x4FD1C5;
static const uint32_t C_VIOLET = 0xB9A4FF;
static const uint32_t C_GREEN = 0x38D996;

static lv_obj_t *mk(lv_obj_t *p, const lv_font_t *f, uint32_t col,
                    lv_align_t al, int x, int y, const char *t = "") {
  lv_obj_t *l = lv_label_create(p);
  lv_obj_set_style_text_font(l, f, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
  lv_obj_align(l, al, x, y);
  lv_label_set_text(l, t);
  return l;
}

static lv_obj_t *mkCard(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color = C_CARD) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_radius(o, 26, 0);
  lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(o, 1, 0);
  lv_obj_set_style_border_color(o, lv_color_hex(C_BORDER), 0);
  lv_obj_set_style_shadow_width(o, 0, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  return o;
}

// shape helper used by the icons
static lv_obj_t *shape(lv_obj_t *p, int x, int y, int w, int h, uint32_t col,
                       int r = LV_RADIUS_CIRCLE, lv_opa_t op = LV_OPA_COVER) {
  lv_obj_t *o = lv_obj_create(p);
  lv_obj_remove_style_all(o);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_radius(o, r, 0);
  lv_obj_set_style_bg_color(o, lv_color_hex(col), 0);
  lv_obj_set_style_bg_opa(o, op, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static void mkHeader(lv_obj_t *p, const char *title, uint32_t col) {
  shape(p, 24, 20, 5, 26, col, 3);
  mk(p, &font_th_22, col, LV_ALIGN_TOP_LEFT, 40, 18, title);
  mk(p, &font_th_22, C_MUTED, LV_ALIGN_TOP_RIGHT, -24, 18, CITY_NAME);
}

// ---- weather icons (drawn with LVGL shapes, coordinates in a 100x100 box) ----
static int icSz = 100;
#define K(v) ((int)(v) * icSz / 100)

static void cloudAt(lv_obj_t *p, int x, int y, int w, uint32_t c) {
  shape(p, K(x), K(y + w * 30 / 100), K(w), K(w * 38 / 100), c);
  shape(p, K(x + w * 12 / 100), K(y + w * 10 / 100), K(w * 42 / 100), K(w * 42 / 100), c);
  shape(p, K(x + w * 34 / 100), K(y), K(w * 52 / 100), K(w * 52 / 100), c);
}
static lv_obj_t *mkIcon(lv_obj_t *p, int x, int y, int sz) {
  lv_obj_t *o = lv_obj_create(p);
  lv_obj_remove_style_all(o);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, sz, sz);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_user_data(o, (void *)(intptr_t)(sz << 16));
  return o;
}
static void setIcon(lv_obj_t *box, int code, bool day, uint32_t bg) {
  intptr_t old = (intptr_t)lv_obj_get_user_data(box);
  int sz = (int)(old >> 16);
  intptr_t key = ((intptr_t)sz << 16) | (code * 2 + (day ? 1 : 0) + 1);
  if (old == key) return;
  lv_obj_set_user_data(box, (void *)key);
  lv_obj_clean(box);
  icSz = sz;
  const uint32_t SUN = 0xFFB86B, WHITE = 0xDDE6F5, GREY = 0x7F8CA3, LIGHT = 0xC9D3E3, DROP = 0x4FD1C5;
  switch (code) {
    case 0: case 1:
      if (day) {
        shape(box, K(6), K(6), K(88), K(88), SUN, LV_RADIUS_CIRCLE, LV_OPA_30);
        shape(box, K(22), K(22), K(56), K(56), SUN);
      } else {
        shape(box, K(22), K(18), K(56), K(56), 0xE6EAFF);
        shape(box, K(40), K(8), K(52), K(52), bg);
      }
      break;
    case 2:
      if (day) shape(box, K(4), K(4), K(52), K(52), SUN);
      else { shape(box, K(6), K(6), K(46), K(46), 0xE6EAFF); shape(box, K(20), K(0), K(44), K(44), bg); }
      cloudAt(box, 22, 36, 74, WHITE);
      break;
    case 3:
      cloudAt(box, 2, 14, 62, GREY);
      cloudAt(box, 24, 32, 72, LIGHT);
      break;
    case 45: case 48:
      cloudAt(box, 14, 6, 72, 0x9AA8BF);
      for (int i = 0; i < 3; i++) shape(box, K(i % 2 ? 24 : 12), K(58 + i * 13), K(64), K(7), 0x9AA8BF, LV_RADIUS_CIRCLE, LV_OPA_80);
      break;
    case 71: case 73: case 75: case 77: case 85: case 86:
      cloudAt(box, 14, 6, 72, LIGHT);
      for (int i = 0; i < 3; i++) shape(box, K(26 + i * 20), K(68 + (i % 2) * 10), K(10), K(10), 0xFFFFFF);
      break;
    case 95: case 96: case 99: {
      cloudAt(box, 14, 4, 72, GREY);
      lv_obj_t *b = lv_label_create(box);
      lv_obj_set_style_text_font(b, &lv_font_montserrat_28, 0);
      lv_obj_set_style_text_color(b, lv_color_hex(SUN), 0);
      lv_label_set_text(b, LV_SYMBOL_CHARGE);
      lv_obj_set_pos(b, K(42), K(56));
      break;
    }
    default:  // drizzle / rain / showers
      cloudAt(box, 14, 6, 72, 0xAEBBD0);
      for (int i = 0; i < 3; i++) shape(box, K(30 + i * 20), K(66 + (i % 2) * 8), K(7), K(18), DROP, 4);
      break;
  }
}

static void show(int i);

static lv_obj_t *mkScene(lv_obj_t *root) {
  lv_obj_t *o = lv_obj_create(root);
  lv_obj_remove_style_all(o);
  lv_obj_set_size(o, SW, SH);
  lv_obj_set_style_bg_color(o, lv_color_hex(C_BG), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(o, [](lv_event_t *e) {
    lv_indev_t *in = lv_indev_get_act();
    if (!in) return;
    lv_point_t v; lv_indev_get_point(in, &v);
    // Tap left/right side to switch pages.
    if (v.x < 400) show(cur - 1);
    else show(cur + 1);
  }, LV_EVENT_CLICKED, nullptr);
  return o;
}

static void show(int i) {
  cur = (i + NSC) % NSC;
  for (int k = 0; k < NSC; k++) {
    if (k == cur) lv_obj_clear_flag(sc[k], LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(sc[k], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(dots[k], lv_color_hex(k == cur ? C_AMBER : 0x34415C), 0);
    lv_obj_set_style_width(dots[k], k == cur ? 24 : 8, 0);
  }
}

static void buildUI() {
  lv_obj_t *root = lv_scr_act();
  lv_obj_set_style_bg_color(root, lv_color_hex(C_BG), 0);
  lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);
  const lv_font_t *F16 = &font_th_16;
  const lv_font_t *F22 = &font_th_22;
  const lv_font_t *F30 = &font_th_30;
  const lv_font_t *FT = &font_temp;
  for (int i = 0; i < NSC; i++) sc[i] = mkScene(root);

  // 0 — HOME: big clock + three summary cards
  mkHeader(sc[0], "หน้าหลัก", C_AMBER);
  l_time = mk(sc[0], &font_clock, C_TEXT, LV_ALIGN_TOP_MID, -22, 34, "--:--");
  l_sec = mk(sc[0], F30, C_AMBER, LV_ALIGN_TOP_LEFT, 664, 126, "");
  l_date = mk(sc[0], F30, C_MUTED, LV_ALIGN_TOP_MID, 0, 178, "กำลังซิงค์เวลา...");

  lv_obj_t *cW = mkCard(sc[0], 24, 250, 240, 160, C_CARD);
  ico_home = mkIcon(cW, 14, 14, 72);
  l_ctemp = mk(cW, FT, C_TEXT, LV_ALIGN_TOP_LEFT, 96, 8, "--");
  l_ccond = mk(cW, F22, C_TEAL, LV_ALIGN_TOP_LEFT, 18, 96, "รอข้อมูล...");
  l_chilo = mk(cW, F16, C_MUTED, LV_ALIGN_TOP_LEFT, 18, 128, "");

  lv_obj_t *cS = mkCard(sc[0], 280, 250, 240, 160, C_CARD2);
  mk(cS, F16, C_AMBER, LV_ALIGN_TOP_LEFT, 18, 14, "ดวงอาทิตย์");
  l_homeSun = mk(cS, F22, C_TEXT, LV_ALIGN_TOP_LEFT, 18, 46, "ขึ้น  --:--");
  l_homeSet = mk(cS, F22, C_AMBER, LV_ALIGN_TOP_LEFT, 18, 84, "ตก   --:--");
  l_homeDay = mk(cS, F16, C_MUTED, LV_ALIGN_TOP_LEFT, 18, 126, "");

  lv_obj_t *cA = mkCard(sc[0], 536, 250, 240, 160, C_CARD);
  mk(cA, F16, C_GREEN, LV_ALIGN_TOP_LEFT, 18, 14, "คุณภาพอากาศ (AQI)");
  l_haqi = mk(cA, FT, C_GREEN, LV_ALIGN_TOP_LEFT, 18, 34, "--");
  l_hpm = mk(cA, F16, C_MUTED, LV_ALIGN_TOP_LEFT, 18, 94, "PM2.5  --");
  l_homeAir = mk(cA, F22, C_TEXT, LV_ALIGN_TOP_LEFT, 18, 120, "รอข้อมูล...");

  l_cstat = mk(sc[0], F16, C_MUTED, LV_ALIGN_BOTTOM_MID, 0, -36, "กำลังเชื่อมต่อ Wi-Fi...");

  // 1 — WEATHER
  mkHeader(sc[1], "สภาพอากาศ", C_TEAL);
  lv_obj_t *weatherMain = mkCard(sc[1], 24, 65, 340, 350);
  ico_wx = mkIcon(weatherMain, 20, 22, 100);
  l_wtemp = mk(weatherMain, FT, C_TEXT, LV_ALIGN_TOP_LEFT, 136, 28, "--");
  l_wcond = mk(weatherMain, F30, C_TEAL, LV_ALIGN_TOP_LEFT, 22, 150, "รอข้อมูลสภาพอากาศ...");
  l_wfeel = mk(weatherMain, F22, C_AMBER, LV_ALIGN_TOP_LEFT, 22, 208, "รู้สึกเหมือน --");
  l_whilo = mk(weatherMain, F22, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 250, "สูงสุด --   ต่ำสุด --");
  mk(weatherMain, F16, C_MUTED, LV_ALIGN_BOTTOM_LEFT, 22, -18, "พยากรณ์วันนี้ • Open-Meteo");
  lv_obj_t *weatherDetails = mkCard(sc[1], 386, 65, 390, 350, C_CARD2);
  mk(weatherDetails, F16, C_TEAL, LV_ALIGN_TOP_LEFT, 22, 20, "รายละเอียด");
  l_wdet = mk(weatherDetails, F22, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 56,
              "ความชื้น  --%\nเมฆปกคลุม  --%\nลม  --\nความกดอากาศ  --\nรังสี UV  --");
  lv_obj_set_style_text_line_space(l_wdet, 14, 0);

  // 2 — SUN & MOON
  mkHeader(sc[2], "ดวงอาทิตย์และดวงจันทร์", C_AMBER);
  lv_obj_t *sunCard = mkCard(sc[2], 24, 65, 390, 350);
  mk(sunCard, F16, C_MUTED, LV_ALIGN_TOP_LEFT, 22, 20, "ช่วงเวลากลางวัน");
  l_rise = mk(sunCard, F30, C_AMBER, LV_ALIGN_TOP_LEFT, 22, 62, "ขึ้น  --:--");
  l_set = mk(sunCard, F30, C_TEAL, LV_ALIGN_TOP_LEFT, 22, 118, "ตก   --:--");
  l_day = mk(sunCard, F22, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 184, "ระยะเวลากลางวัน --");
  bar_day = lv_bar_create(sunCard);
  lv_obj_set_size(bar_day, 346, 14); lv_obj_align(bar_day, LV_ALIGN_TOP_LEFT, 22, 240);
  lv_bar_set_range(bar_day, 0, 100);
  lv_obj_set_style_radius(bar_day, 7, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_day, lv_color_hex(C_TRACK), LV_PART_MAIN);
  lv_obj_set_style_radius(bar_day, 7, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bar_day, lv_color_hex(C_AMBER), LV_PART_INDICATOR);
  mk(sunCard, F16, C_MUTED, LV_ALIGN_BOTTOM_LEFT, 22, -18, "ความคืบหน้าของวัน");
  lv_obj_t *moonCard = mkCard(sc[2], 434, 65, 342, 350, C_CARD2);
  mk(moonCard, F16, C_VIOLET, LV_ALIGN_TOP_LEFT, 22, 20, "ดวงจันทร์");
  arc_moon = lv_arc_create(moonCard);
  lv_obj_set_size(arc_moon, 180, 180); lv_obj_align(arc_moon, LV_ALIGN_TOP_MID, 0, 52);
  lv_arc_set_rotation(arc_moon, 270); lv_arc_set_bg_angles(arc_moon, 0, 360);
  lv_arc_set_range(arc_moon, 0, 100);
  lv_obj_remove_style(arc_moon, nullptr, LV_PART_KNOB);
  lv_obj_clear_flag(arc_moon, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(arc_moon, 12, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc_moon, 12, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(arc_moon, lv_color_hex(C_TRACK), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc_moon, lv_color_hex(C_VIOLET), LV_PART_INDICATOR);
  l_moonp = mk(moonCard, F30, C_TEXT, LV_ALIGN_TOP_MID, 0, 120, "--%");
  l_moon = mk(moonCard, F22, C_VIOLET, LV_ALIGN_BOTTOM_MID, 0, -28, "ดวงจันทร์");

  // 3 — AIR QUALITY
  mkHeader(sc[3], "คุณภาพอากาศ", C_GREEN);
  lv_obj_t *aqCard = mkCard(sc[3], 24, 65, 350, 350);
  mk(aqCard, F16, C_MUTED, LV_ALIGN_TOP_LEFT, 22, 20, "ดัชนีคุณภาพอากาศ (US AQI)");
  l_aqi = mk(aqCard, FT, C_GREEN, LV_ALIGN_TOP_LEFT, 22, 56, "--");
  l_aqicat = mk(aqCard, F30, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 160, "รอข้อมูลคุณภาพอากาศ...");
  bar_aqi = lv_bar_create(aqCard);
  lv_obj_set_size(bar_aqi, 306, 14); lv_obj_align(bar_aqi, LV_ALIGN_TOP_LEFT, 22, 230);
  lv_bar_set_range(bar_aqi, 0, 300);
  lv_obj_set_style_radius(bar_aqi, 7, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_aqi, lv_color_hex(C_TRACK), LV_PART_MAIN);
  lv_obj_set_style_radius(bar_aqi, 7, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bar_aqi, lv_color_hex(C_GREEN), LV_PART_INDICATOR);
  mk(aqCard, F16, C_MUTED, LV_ALIGN_BOTTOM_LEFT, 22, -18, "ยิ่งต่ำยิ่งดี");
  lv_obj_t *aqDetails = mkCard(sc[3], 396, 65, 380, 350, C_CARD2);
  mk(aqDetails, F16, C_GREEN, LV_ALIGN_TOP_LEFT, 22, 20, "สุขภาพและสิ่งแวดล้อม");
  l_pm = mk(aqDetails, F30, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 70, "PM2.5  -- µg/m³");
  l_pm10 = mk(aqDetails, F30, C_TEXT, LV_ALIGN_TOP_LEFT, 22, 136, "PM10  -- \xC2\xB5g/m\xC2\xB3");
  l_uv = mk(aqDetails, F30, C_AMBER, LV_ALIGN_TOP_LEFT, 22, 202, "รังสี UV  --");
  mk(aqDetails, F16, C_MUTED, LV_ALIGN_BOTTOM_LEFT, 22, -22, "ควรหลีกเลี่ยงกิจกรรมกลางแจ้งนานๆ เมื่อ AQI สูง");

  // Page indicators
  for (int i = 0; i < NSC; i++) {
    dots[i] = lv_obj_create(root);
    lv_obj_remove_style_all(dots[i]);
    lv_obj_set_size(dots[i], 8, 8);
    lv_obj_set_style_radius(dots[i], LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(dots[i], LV_OPA_COVER, 0);
    lv_obj_align(dots[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 24 - 12, -10);
    lv_obj_clear_flag(dots[i], LV_OBJ_FLAG_CLICKABLE);
  }
  show(0);
}

static void fmtT(char *b, size_t n, time_t t, const char *f = "%H:%M") {
  if (t <= 0) { snprintf(b, n, "--:--"); return; }
  struct tm m; localtime_r(&t, &m); strftime(b, n, f, &m);
}

static void refreshUI(lv_timer_t *) {
  char b[160];
  struct tm t;
  bool tv = getLocalTime(&t, 0);
  sunmoonRecompute();
  time_t now = time(nullptr);
  bool day = (g.sunrise == 0) || (now >= g.sunrise && now < g.sunset);

  if (tv) {
    strftime(b, sizeof b, "%H:%M", &t); lv_label_set_text(l_time, b);
    strftime(b, sizeof b, "%S", &t); lv_label_set_text(l_sec, b);
    snprintf(b, sizeof b, "วัน%sที่ %d %s %d", TH_DAY[t.tm_wday], t.tm_mday, TH_MON[t.tm_mon], t.tm_year + 1900 + 543);
    lv_label_set_text(l_date, b);
  } else {
    lv_label_set_text(l_time, "--:--");
    lv_label_set_text(l_sec, "");
    lv_label_set_text(l_date, "รอเวลาจากเครือข่าย...");
  }

  if (g.wxValid) {
    snprintf(b, sizeof b, "%.0f\xC2\xB0", dTemp(g.tempC)); lv_label_set_text(l_ctemp, b);
    lv_label_set_text(l_ccond, wmo(g.code));
    snprintf(b, sizeof b, "สูง %.0f\xC2\xB0  ต่ำ %.0f\xC2\xB0", dTemp(g.hiC), dTemp(g.loC)); lv_label_set_text(l_chilo, b);
    snprintf(b, sizeof b, "%.0f\xC2\xB0%s", dTemp(g.tempC), U_T); lv_label_set_text(l_wtemp, b);
    lv_label_set_text(l_wcond, wmo(g.code));
    snprintf(b, sizeof b, "รู้สึกเหมือน %.1f\xC2\xB0%s", dTemp(g.feelsC), U_T); lv_label_set_text(l_wfeel, b);
    snprintf(b, sizeof b, "สูงสุด %.0f\xC2\xB0   ต่ำสุด %.0f\xC2\xB0", dTemp(g.hiC), dTemp(g.loC)); lv_label_set_text(l_whilo, b);
    snprintf(b, sizeof b, "ความชื้น  %d%%\nเมฆปกคลุม  %d%%\nลม  %.0f %s\nความกดอากาศ  %.*f %s\nรังสี UV  %.1f",
             g.hum, g.cloud, dWind(g.windKph), U_W, USE_IMPERIAL ? 2 : 0, dPress(g.pressHpa), U_P, g.uv);
    lv_label_set_text(l_wdet, b);
    setIcon(ico_home, g.code, day, C_CARD);
    setIcon(ico_wx, g.code, day, C_CARD);
  } else {
    lv_label_set_text(l_ctemp, "--");
    lv_label_set_text(l_ccond, "รอข้อมูล...");
    lv_label_set_text(l_wtemp, "--");
  }

  bool nxt = now > g.sunset;
  time_t r = nxt ? g.nextRise : g.sunrise;
  time_t s = nxt ? g.nextSet : g.sunset;
  char riseBuf[24], setBuf[24];
  fmtT(riseBuf, sizeof riseBuf, r); fmtT(setBuf, sizeof setBuf, s);
  snprintf(b, sizeof b, "ขึ้น  %s", riseBuf); lv_label_set_text(l_homeSun, b);
  snprintf(b, sizeof b, "ตก   %s", setBuf); lv_label_set_text(l_homeSet, b);
  snprintf(b, sizeof b, "ตก   %s", setBuf); lv_label_set_text(l_set, b);
  snprintf(b, sizeof b, "ขึ้น  %s", riseBuf); lv_label_set_text(l_rise, b);
  if (tv && r > 0 && s > r) {
    long dl = s - r;
    snprintf(b, sizeof b, "%s  |  กลางวัน %ld ชม. %02ld น.", nxt ? "พรุ่งนี้" : "วันนี้", dl / 3600, (dl % 3600) / 60);
    lv_label_set_text(l_day, b);
    snprintf(b, sizeof b, "กลางวัน %ld ชม. %02ld น.", dl / 3600, (dl % 3600) / 60);
    lv_label_set_text(l_homeDay, b);
    int pct = (now <= r) ? 0 : (now >= s ? 100 : (int)((now - r) * 100 / (s - r)));
    lv_bar_set_value(bar_day, pct, LV_ANIM_OFF);
  } else {
    lv_label_set_text(l_day, "ระยะเวลากลางวัน --");
    lv_label_set_text(l_homeDay, "");
    lv_bar_set_value(bar_day, 0, LV_ANIM_OFF);
  }
  lv_arc_set_value(arc_moon, (int)g.moonIllum);
  snprintf(b, sizeof b, "%.0f%%", g.moonIllum); lv_label_set_text(l_moonp, b);
  lv_label_set_text(l_moon, moonNameTh(g.moonPhase));

  if (g.aqValid) {
    uint32_t col; const char *cat = aqiCat(g.aqi, col);
    snprintf(b, sizeof b, "%d", g.aqi);
    lv_label_set_text(l_aqi, b); lv_label_set_text(l_haqi, b);
    lv_obj_set_style_text_color(l_aqi, lv_color_hex(col), 0);
    lv_obj_set_style_text_color(l_haqi, lv_color_hex(col), 0);
    lv_label_set_text(l_aqicat, cat);
    lv_obj_set_style_bg_color(bar_aqi, lv_color_hex(col), LV_PART_INDICATOR);
    lv_bar_set_value(bar_aqi, min(g.aqi, 300), LV_ANIM_OFF);
    lv_label_set_text(l_homeAir, cat);
    snprintf(b, sizeof b, "PM2.5  %d \xC2\xB5g/m\xC2\xB3", g.pm25); lv_label_set_text(l_pm, b);
    lv_obj_set_style_text_color(l_pm, lv_color_hex(col), 0);
    snprintf(b, sizeof b, "PM10  %d \xC2\xB5g/m\xC2\xB3", g.pm10); lv_label_set_text(l_pm10, b);
    snprintf(b, sizeof b, "PM2.5  %d \xC2\xB5g/m\xC2\xB3", g.pm25); lv_label_set_text(l_hpm, b);
  } else {
    lv_label_set_text(l_aqi, "--"); lv_label_set_text(l_haqi, "--");
    lv_label_set_text(l_aqicat, "รอข้อมูลคุณภาพอากาศ...");
    lv_label_set_text(l_homeAir, "รอข้อมูล...");
    lv_label_set_text(l_pm, "PM2.5  -- \xC2\xB5g/m\xC2\xB3");
    lv_label_set_text(l_pm10, "PM10  -- \xC2\xB5g/m\xC2\xB3");
    lv_label_set_text(l_hpm, "PM2.5  --");
  }
  snprintf(b, sizeof b, "Wi-Fi %s   |   สภาพอากาศ %s   |   คุณภาพอากาศ %s",
           WiFi.status() == WL_CONNECTED ? "เชื่อมต่อแล้ว" : "กำลังเชื่อมต่อ",
           g.wxValid ? "ล่าสุด" : "รอข้อมูล", g.aqValid ? "ล่าสุด" : "รอข้อมูล");
  lv_label_set_text(l_cstat, b);
  snprintf(b, sizeof b, "รังสี UV  %.1f", g.uv); lv_label_set_text(l_uv, b);
}
// ---------- 8. setup / loop ----------
void setup() {
  Serial.begin(115200);
  lcd.init(); lcd.setRotation(2); lcd.setBrightness(128);

  pinMode(TOUCH_RST, OUTPUT); pinMode(TOUCH_INT, OUTPUT);
  digitalWrite(TOUCH_INT, LOW); digitalWrite(TOUCH_RST, LOW); delay(20);
  digitalWrite(TOUCH_RST, HIGH); delay(20);
  digitalWrite(TOUCH_INT, LOW); delay(50);
  pinMode(TOUCH_INT, INPUT);
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  gtInit();

  lv_init();
  lvbuf = (lv_color_t *)heap_caps_malloc(SW * SH / 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_disp_draw_buf_init(&draw_buf, lvbuf, NULL, SW * SH / 10);
  static lv_disp_drv_t dd; lv_disp_drv_init(&dd);
  dd.hor_res = SW; dd.ver_res = SH; dd.flush_cb = my_disp_flush; dd.draw_buf = &draw_buf;
  lv_disp_drv_register(&dd);
  static lv_indev_drv_t id; lv_indev_drv_init(&id);
  id.type = LV_INDEV_TYPE_POINTER; id.read_cb = my_touch_read;
  lv_indev_drv_register(&id);

  buildUI();
  lv_timer_create(refreshUI, 500, nullptr);
  xTaskCreate(netTask, "net", 12288, nullptr, 1, nullptr);
}

void loop() {
  lv_timer_handler();
  delay(5);
}