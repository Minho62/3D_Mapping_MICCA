// 2024-04-10
#include <LiquidCrystal_I2C.h>

#define tx_enable 2
#define SN74LV_TX \
  digitalWrite(tx_enable, HIGH); \
  delayMicroseconds(10);
#define SN74LV_RX \
  digitalWrite(tx_enable, LOW); \
  delayMicroseconds(10);

float velocity = 5;
int ori_angle_min = 180;
int ori_angle_max = 180;

enum SN  // State nums
{
  ERROR,
  FAIL_LIDAR,
  FAIL_IMU,
  FAIL_USB,
  FAIL_LIDAR_IMU_USB,
  FAIL_LIDAR_IMU,
  FAIL_LIDAR_USB,
  FAIL_IMU_USB,
  BOOT,
  GOOD,
};

enum PM  // Print Menu
{
  MAIN,
  MEASUREMENT,
  TUNNEL,
  CAVE,
  PROCESSING,
  SETTING,
  SETTING_ANGLE_MAX,
  SETTING_ANGLE_MIN,
  SELECT_USB,
};

void send_data(Stream &serialport, byte buf_0, byte buf_1, byte buf_2);

class str {
  private:
    char *pstring;
    int str_len = 0;
  public:
    str(int val);
    str(int val, int space_buffer);
    ~str();
    str(const char c[]);
    int len(void);
    const char* string(void);
};

str::str(int val) {
  int temp_val = val;
  int cnt = 1;
  while (temp_val != 0) {
    temp_val = temp_val / 10;
    ++cnt;
  }
  str_len = cnt;

  pstring = new char[str_len + 1];
  sprintf(pstring, "%d  ", val);
  pstring[str_len] = NULL;
}

str::str(int val, int space_buffer) {
  int temp_val = val;
  int cnt = 1;
  while (temp_val != 0) {
    temp_val = temp_val / 10;
    ++cnt;
  }
  str_len = cnt;

  pstring = new char[str_len + 1 + space_buffer];
  sprintf(pstring, "%d  ", val);
  pstring[str_len + space_buffer] = NULL;
}

int str::len(void) {
  return str_len;
}

const char* str::string(void) {
  return pstring;
}

str::~str() {
  delete[] pstring;
}

// 객체이름을 lcd로 선언 I2C 주소값을 0x27로 설정, 16 문자 2줄 디스플레이
// 만약 lcd가 출력이 안된다면, 주소값을 0x27 혹은 0x3F로 수정
class CONTROL {
  private:
    bool jetson_state = false;
    bool imu_state = false;
    bool lidar_state = false;
    bool USB_state = false;

    bool toogle = true;
    int col;
    int row;

    int bat = 100;
    int stat = 4;
    int page = MAIN;
    int cursor = 3;
    int proc_g = 0;

    int angle_min = 180;
    int angle_max = 180;

    int js_x = 1;
    int js_y = 1;

    int cursor_cnt = 0;
    int capture_cnt = 0;

    unsigned long auto_time;
    LiquidCrystal_I2C *lcd;
  public:
    CONTROL(void);
    void print_menu(void);
    void print_state(int battery, int state);
    void print_str(const char c[]);
    void set_cursor(int _col, int _row);
    void sector_clear(char start_row, char start_col, char cnt);
    int cur_range(int page, int select);
    void get_angle(int& A_min, int& A_max);
    void angle_print(int max_cursor, int min_cursor);
    void control(Stream &serialport, int joystick_x, int joystick_y, int joystick_btn, int press_btn);
    void serial_data_state(int instruction, int value, int state);
    bool jetson_state_return(void);
};

CONTROL::CONTROL(void) {
  col = 0;
  row = 0;
  LiquidCrystal_I2C lcd_init(0x27, 20, 4);

  lcd = &lcd_init;
  lcd->init(); // lcd 시작
  lcd->backlight(); // lcd 백라이트 on
  set_cursor(col, row);
}

bool CONTROL::jetson_state_return(void) {
  return jetson_state;
}

void CONTROL::print_state(int battery, int state) {
  /*
    state
    enum SN  // State nums
    {
    ERROR,
    FAIL_LIDAR,
    FAIL_IMU,
    FAIL_USB,
    FAIL_LIDAR_IMU_USB,
    FAIL_LIDAR_IMU,
    BOOT,
    GOOD,
    FAIL_USB,
    };
  */
  if (bat != battery) {
    sector_clear(8, 0, 12);
    bat = battery;
  }
  str bat_s(bat);

  set_cursor(0, 0);
  print_str("BATTERY:");

  set_cursor(20 - (bat_s.len() + 1), 0);
  print_str(bat_s.string());
  print_str("%");

  if (stat != state && state != -1) {
    sector_clear(6, 1, 14);
    stat = state;
  }
  else {
    return;
  }
  set_cursor(0, 1);
  print_str("STATE:");
  switch (stat) {
    case FAIL_LIDAR:
      set_cursor(20 - 11, 1);
      print_str("FAIL(LIDAR)");
      break;
    case FAIL_IMU:
      set_cursor(20 - 9, 1);
      print_str("FAIL(IMU)");
      break;
    case FAIL_USB:
      set_cursor(20 - 9, 1);
      print_str("FAIL(USB)");
      break;
    case FAIL_LIDAR_IMU_USB:
      set_cursor(20 - 13, 1);
      print_str("FAIL(L, I, U)");
      break;
    case FAIL_LIDAR_IMU:
      set_cursor(20 - 10, 1);
      print_str("FAIL(L, I)");
      break;
    case FAIL_LIDAR_USB:
      set_cursor(20 - 10, 1);
      print_str("FAIL(L, U)");
      break;
    case FAIL_IMU_USB:
      set_cursor(20 - 10, 1);
      print_str("FAIL(I, U)");
      break;
    case BOOT:
      set_cursor(20 - 7, 1);
      print_str("BOOTING");
      break;
    case GOOD:
      set_cursor(20 - 4, 1);
      print_str("GOOD");
      break;
    default:
      set_cursor(20 - 5, 1);
      print_str("ERROR");
  }
}

int CONTROL::cur_range(int page, int select) {
  if (page == MAIN || page == MEASUREMENT || page == TUNNEL || page == CAVE) {
    if (select < 2)
      return 2;
    else if (select > 3)
      return 3;
  } else if (page == PROCESSING) {
    return -1;
  }
}

void CONTROL::angle_print(int max_cursor, int min_cursor) {
  str angle_max_s(angle_max, 2);
  str angle_min_s(angle_min, 2);

  if (max_cursor != -1) {
    set_cursor(11, max_cursor);
    print_str(angle_max_s.string());
  }
  if (min_cursor != -1) {
    set_cursor(11, min_cursor);
    print_str(angle_min_s.string());
  }
}

void CONTROL::get_angle(int& A_min, int& A_max) {
  A_min = angle_min;
  A_max = angle_max;
}

void CONTROL::control(Stream &serialport, int joystick_x, int joystick_y, int joystick_btn, int press_btn) {
  if (page == MAIN) {
    if (joystick_y < 1 && cursor < 3)
      cursor++;
    else if (joystick_y > 1 && cursor > 2)
      cursor--;

    if (joystick_x < 1 || joystick_btn || !press_btn) {
      if (cursor == 2)
        page = MEASUREMENT;
      else if (cursor == 3)
        page = SETTING;
    }
  } else if (page == MEASUREMENT) {
    if (joystick_y < 1 && cursor < 3)
      cursor++;
    else if (joystick_y > 1 && cursor > 2)
      cursor--;

    if ((joystick_x < 1 || joystick_btn || !press_btn) && (jetson_state && imu_state && lidar_state)) {
      if (cursor == 2) {
        // SEND TUNNEL 측정 시작
        send_data(serialport, 0x02, 0x00, 0x02);
        page = TUNNEL;
        auto_time = millis();
        angle_min = 90;
        angle_max = 90;
      } else if (cursor == 3) {
        // SEND CAVE 측정 시작
        send_data(serialport, 0x02, 0x00, 0x05);
        page = CAVE;
        velocity = 1;
        ori_angle_min = angle_min;
        ori_angle_max = angle_max;
        angle_min = 180;
        angle_max = 180;
      }
    } else if (joystick_x > 1) {
      page = MAIN;
    }
  } else if (page == TUNNEL) {
    if (joystick_x > 1) {
      send_data(serialport, 0x02, 0x00, 0x01);
      // SEND TUNNEL 측정 중지
      send_data(serialport, 0x03, 0x00, 0x01);
      // SEND USB 연결 유지
      page = MEASUREMENT;
    }
    if (joystick_x < 1 || joystick_btn || !press_btn || (millis() > (auto_time + 10000))) {
      // SEND TUNNEL 측정 중지, PROCESSING
      send_data(serialport, 0x02, 0x00, 0x03);
      page = PROCESSING;
    }
  } else if (page == CAVE) {
    if (joystick_x > 1) {
      send_data(serialport, 0x02, 0x00, 0x04);
      // SEND CAVE 측정 중지
      send_data(serialport, 0x03, 0x00, 0x01);
      // SEND USB 연결 유지
      page = MEASUREMENT;
      //#capture_cnt = 0;
    }
    if (joystick_btn || !press_btn) {
      //cave측정 시작
      if (capture_cnt == 0)
      {
        send_data(serialport, 0x02, 0x00, 0x07);
        
        delay(5000);//채터링 방지
        angle_min = ori_angle_min;
        angle_max = ori_angle_max;
        
        capture_cnt++;

      }
      else if (capture_cnt >= 1)
      { 
        delay(10);//채터링 방지
        //    if (joystick_x < 1 || joystick_btn) {
        send_data(serialport, 0x02, 0x00, 0x04);
        // SEND CAVE 측정 중지
        send_data(serialport, 0x03, 0x00, 0x01);
        // SEND USB 연결 유지
        page = SELECT_USB;
        velocity = 5;
        capture_cnt = 0;
      }
    }
    else if (joystick_x < 1) {
      //    else if(!press_btn) {
      // SEND CAVE 측정 중지, PROCESSING
      send_data(serialport, 0x02, 0x00, 0x06);
      page = PROCESSING;
    }
  } else if (page == PROCESSING) {
    if (joystick_x > 1) {
      page = MAIN;
    }
  } else if (page == SETTING) {
    if (joystick_y < 1 && cursor < 5)
      cursor++;
    else if (joystick_y > 1 && cursor > 2)
      cursor--;

    if (joystick_x < 1 || joystick_btn || !press_btn) {
      if (cursor == 2) {
        page = SETTING_ANGLE_MAX;
      }
      else if (cursor == 3) {
        page = SETTING_ANGLE_MIN;
      }
      else if (cursor == 4) {
        // SEND USB DIS CON
        send_data(serialport, 0x03, 0x00, 0x02);
      }
      else if (cursor == 5) {
        // SHUTDOWN
        send_data(serialport, 0x98, 0x43, 0x17);
      }
    }

    if (joystick_x > 1) {
      if (cursor > 3) {
        cursor = 3;
      }
      page = MAIN;
    }
  } else if (page == SETTING_ANGLE_MIN) {
    if (joystick_y < 1 && angle_min > 0)
      angle_min--;
    else if (joystick_y > 1 && angle_min < angle_max)
      angle_min++;

    if (joystick_x > 1 || joystick_btn || !press_btn) {
      page = SETTING;
    }
  } else if (page == SETTING_ANGLE_MAX) {
    if (joystick_y < 1 && angle_min < angle_max)
      angle_max--;
    else if (joystick_y > 1 && angle_max < 360)
      angle_max++;

    if (joystick_x > 1 || joystick_btn || !press_btn) {
      page = SETTING;
    }
  } else if (page == SELECT_USB) {
    if (joystick_y < 1 && cursor < 3)
      cursor++;
    else if (joystick_y > 1 && cursor > 2)
      cursor--;

    if (joystick_x < 1 || joystick_btn || !press_btn) {
      if (cursor == 2) {
        // SEND USB DIS CON
        send_data(serialport, 0x03, 0x00, 0x02);
        page = MAIN;
      }
      else if (cursor == 3) {
        // SEND USB CON
        send_data(serialport, 0x03, 0x00, 0x01);
        page = MAIN;
      }
    }
  }
}

void CONTROL::print_menu(void) {
  static int pg = 0;
  if (pg != page) {
    sector_clear(0, 2, 20);
    sector_clear(0, 3, 20);
    pg = page;
  }

  if (pg == MAIN) {  // main
    set_cursor(0, 2);
    print_str("MEASUREMENT");
    set_cursor(0, 3);
    print_str("SETTING");
  } else if (pg == MEASUREMENT) {  // 측정(MEASUREMENT)
    set_cursor(0, 2);
    print_str("TUNNEL");
    set_cursor(0, 3);
    print_str("CAVE");
  } else if (pg == TUNNEL) {  // 측정(MEASUREMENT) - TUNNEL
    set_cursor(0, 2);
    print_str("Press enter");
    set_cursor(0, 3);
    print_str("button to stop");
  } else if (pg == CAVE) {  // 측정(MEASUREMENT) - CAVE
    set_cursor(0, 2);
    print_str("Press enter");
    set_cursor(0, 3);
    print_str("button to stop");
  } else if (pg == PROCESSING) {  // PROCESSING
    set_cursor(5, 2);
    print_str("PROCESSING");
    for (int i = 0; i <= proc_g; i++) {
      set_cursor(i, 3);
      lcd->write(0xFF);
    }

    if (proc_g >= 19) {
      proc_g = 0;
      page = SELECT_USB;
      send_data(Serial, 0x02, 0x00, 0x01);
      angle_min = 180;
      angle_max = 180;
    }
  } else if (pg == SETTING) {  // SETTING
    if (cursor < 4) {
      set_cursor(0, 2);
      print_str("ANGLE MAX:");
      set_cursor(0, 3);
      print_str("ANGLE MIN:");
      set_cursor(0, 2);
      angle_print(2, 3);
    }
    else if (cursor < 5) {
      set_cursor(0, 2);
      print_str("ANGLE MIN:");
      set_cursor(0, 3);
      print_str("USB EJECT");
      angle_print(-1, 2);
      sector_clear(9, 3, 6);
    }
    else {
      set_cursor(0, 2);
      print_str("USB EJECT");
      set_cursor(0, 3);
      print_str("SHUT DOWN");
      sector_clear(9, 2, 6);
      sector_clear(9, 3, 6);
    }
  } else if (pg == SETTING_ANGLE_MAX || pg == SETTING_ANGLE_MIN) {
    set_cursor(0, 2);
    print_str("ANGLE MAX:");
    set_cursor(0, 3);
    print_str("ANGLE MIN:");
    set_cursor(0, 2);

    angle_print(2, 3);

  } else if (pg == SELECT_USB) {  // SELECT_USB
    set_cursor(0, 2);
    print_str("USB DISCON");
    set_cursor(0, 3);
    print_str("USB CON STAY");
    set_cursor(0, 2);
  }

  //  cursor = cur_range(pg, cursor);
  cursor_cnt++;
  if (toogle && (pg == MAIN || pg == MEASUREMENT || pg == SETTING || pg == SETTING_ANGLE_MAX || pg == SETTING_ANGLE_MIN || pg == SELECT_USB) && (cursor_cnt > 2)) {
    if (cursor < 4) {
      set_cursor(20 - 4, cursor);
      print_str("<---");
    }
    else if (cursor < 5) {
      set_cursor(20 - 4, cursor - 1);
      print_str("<---");
    }
    else {
      set_cursor(20 - 4, cursor - 2);
      print_str("<---");
    }
    cursor_cnt = 0;
    toogle = !toogle;
  } else if (!toogle && !(pg == SETTING_ANGLE_MAX) && !(pg == SETTING_ANGLE_MIN) && (cursor_cnt > 2)) {
    sector_clear(20 - 4, 2, 4);
    sector_clear(20 - 4, 3, 4);
    cursor_cnt = 0;
    toogle = !toogle;
  }
}

void CONTROL::sector_clear(char start_col, char start_row, char cnt) {
  for (char i = start_col; i < start_col + cnt; i++) {
    set_cursor(i, start_row);
    lcd->write(' ');
  }
}

void CONTROL::set_cursor(int _col, int _row) {
  if ((col != _col) || (row != _row)) {
    col = _col;
    row = _row;
    lcd->setCursor(col, row);
  }
}

void CONTROL::print_str(const char c[]) {
  for (int i = 0; c[i] != NULL; i++) {
    set_cursor(col, row);
    lcd->write(c[i]);
    col++;
  }
}

void CONTROL::serial_data_state(int instruction, int value, int state) {
  if (instruction == 0x01) {
    if (value == 0x01 && state == 0x02) {
      jetson_state = true;
    } else if (value == 0x02 && state == 0x02) {
      lidar_state = true;
    } else if (value == 0x03 && state == 0x02) {
      imu_state = true;
    } else if (value == 0x04 && state == 0x02) {
      USB_state = true;
    } else if (value == 0x03 && state == 0x01) {
      imu_state = false;
    } else if (value == 0x02 && state == 0x01) {
      lidar_state = false;
    } else if (value == 0x04 && state == 0x01) {
      USB_state = false;
    }

    if (jetson_state && imu_state && lidar_state && USB_state) {
      print_state(bat, GOOD);  // GOOD
    }
    else if (jetson_state && !imu_state && lidar_state && USB_state) {
      print_state(bat, FAIL_IMU);  // FAIL(IMU)
    }
    else if (jetson_state && imu_state && !lidar_state && USB_state) {
      print_state(bat, FAIL_LIDAR);  // FAIL(LIDAR)
    }
    else if (jetson_state && imu_state && lidar_state && !USB_state) {
      print_state(bat, FAIL_USB);  // FAIL(USB)
    }
    else if (jetson_state && !imu_state && !lidar_state && USB_state) {
      print_state(bat, FAIL_LIDAR_IMU);  // FAIL(L, I)
    }
    else if (jetson_state && imu_state && !lidar_state && !USB_state) {
      print_state(bat, FAIL_LIDAR_USB);  // FAIL(L, U)
    }
    else if (jetson_state && !imu_state && lidar_state && !USB_state) {
      print_state(bat, FAIL_IMU_USB);  // FAIL(I, U)
    }
    else if (jetson_state && !imu_state && !lidar_state && !USB_state) {
      print_state(bat, FAIL_LIDAR_IMU_USB);  // FAIL(L, I, U)
    }
  }
  else if (instruction == 0x02) {
    proc_g = value;
  }
}

byte gbpTxBuffer[128];
byte gbpParameter[128];

unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size)
{
  /*
    리턴 값 : 16bit CRC 값
    crc_accum : ‘0’으로 설정
    data_blk_ptr : Packet array pointer
    data_blk_size : CRC를 제외한 패킷의 byte 수
    data_blk_size = Header(3) + Reserved(1) + Packet ID(1) + Length(2) + Length - CRC(2) = 3 + 1 + 1 + 2 + Length - 2 = 5 + Length;
    Packet Length = (LEN_H « 8 ) + LEN_L; //Little-endian
  */
  unsigned short i, j;
  unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

void TxPacket_xm430(Stream &serialport, byte bID, byte blnstruction, byte bParameterLength)  //ID값,Instruction,parameter 길이
{
  byte bCount, bPacketLength;
  unsigned short bCRC;

  gbpTxBuffer[0] = 0xff;        //패킷의 시작을 알리는 신호
  gbpTxBuffer[1] = 0xff;        //패킷의 시작을 알리는 신호
  gbpTxBuffer[2] = 0xfd;        //패킷의 시작을 알리는 신호
  gbpTxBuffer[3] = 0x00;        //Reserved(Header와 동일한 기능)
  gbpTxBuffer[4] = bID;         //ID
  gbpTxBuffer[5] = ((bParameterLength + 3) & 0xff);       //패킷의 길이  (Parameter0(1) + ParameterN(N) Parameter 개수(N) Instruction(1) + 2) 하위 비트
  gbpTxBuffer[6] = (((bParameterLength + 3) >> 8) & 0xff);  //패킷의 길이  (Parameter0(1) + ParameterN(N) Parameter 개수(N) Instruction(1) + 2) 상위 비트
  gbpTxBuffer[7] = blnstruction;    //Dynamixel에게 수행하라고 지시하는 명령.

  for (bCount = 8; bCount < (bParameterLength + 8); bCount++)   //Put gbpParameter Value in gbpTxBuffer
  {
    gbpTxBuffer[bCount] = gbpParameter[bCount - 8];
  }

  //CRC
  //Packet이 통신 중에 파손되었는지를 점검하기 위한 필드 (16bit CRC)
  //하위 바이트와 상위 바이트를 Instruction Packet에서 나누어서 보냄.
  //CRC 계산 범위: Instruction Packet의 Header (FF FF FD 00)를 포함하여, CRC 필드 이전까지.
  bPacketLength = bParameterLength + 8;     //Header(3) + Reserved(1) + Packet ID(1) + Length(2) + 패킷의 길이 - CRC(2) = 패킷의 길이 + 5 = (bParameterLength+3) + 5 = bParameterLength + 8
  bCRC = update_crc(0, gbpTxBuffer, bPacketLength);
  gbpTxBuffer[bCount] = (bCRC & 0xFF);
  gbpTxBuffer[bCount + 1] = (bCRC >> 8) & 0xFF;

  SN74LV_TX
  for (bCount = 0; bCount < (bPacketLength + 2); bCount++) {  //uart통신 Packet 전송
    serialport.write(gbpTxBuffer[bCount]);
    serialport.flush();
  }
  SN74LV_RX
}

void xm430_position(Stream &serialport, unsigned char ID_number, float p_number)      //모터 위치값,모터 ID값
{
  unsigned int position = 11.375 * p_number;  //Change 0~360 to 0~4095

  gbpParameter[0] = 0x74; //goal position address_L
  gbpParameter[1] = (0x74 >> 8); //goal position address_H
  gbpParameter[2] = (unsigned char)(position); //Writing Data  , goal position
  gbpParameter[3] = (unsigned char)(position >> 8); //goal position
  gbpParameter[4] = (unsigned char)(position >> 16); //goal position
  gbpParameter[5] = (unsigned char)(position >> 24); //goal position

  TxPacket_xm430(serialport, ID_number, 0x03, 0x06);  // , 0x03명령, 길이
}

void send_data(Stream &serialport, byte buf_0, byte buf_1, byte buf_2)
{
  gbpTxBuffer[0] = 0xff;
  gbpTxBuffer[1] = 0xff;
  gbpTxBuffer[2] = buf_0;
  gbpTxBuffer[3] = buf_1;
  gbpTxBuffer[4] = buf_2;

  char checksum = 0;
  for (int i = 2; i < 5; i++) {
    checksum += gbpTxBuffer[i];
  }
  gbpTxBuffer[5] = ~checksum;

  for (int i = 0; i < 6; i++) {
    serialport.write(gbpTxBuffer[i]);
    serialport.flush();
  }
}

void xm430_Operating_mode(Stream &serialport, unsigned char ID_number, unsigned char Operating_mode)  //Operating mode 설정 함수
{
  //Operating Mode 0~16(기본값: 3)
  //0: 전류제어 모드
  //1: 속도제어 모드
  //3: 위치제어 모드
  //4: 확장 위치제어 모드(Multi-turn)
  //5: 전류기반 위치제어 모드
  //16:PWM 제어 모드 (Voltage Control Mode)

  xm430_Torque(serialport, 0xFE, 0);
  gbpParameter[0] = 0x0B;         //Operating mode address_L
  gbpParameter[1] = (0x0B >> 8);  //Operating mode address_H
  gbpParameter[2] = Operating_mode;

  TxPacket_xm430(serialport, ID_number, 0x03, 0x03);
  xm430_Torque(serialport, 0xFE, 1);
}

void xm430_Torque(Stream &serialport, unsigned char ID_number, unsigned char Torque)  //TORQUE 설정 함수
{
  //1이면 ON
  //0이면 OFF(EEPROM할 때 0으로 설정해야함)
  gbpParameter[0] = 0x40;         //TORQUE address_L
  gbpParameter[1] = (0x40 >> 8);  //TORQUE address_H
  gbpParameter[2] = Torque;

  TxPacket_xm430(serialport, ID_number, 0x03, 0x03);
}

void xm430_profile_velocity(Stream &serialport, unsigned char ID_number, float velocity) {
  unsigned int velocity_int = 4.364 * velocity; //veloctiy: 0.0 ~ 77.0[rev/min] -> 0 ~ 336 (velocity unit: 0.229)

  gbpParameter[0] = 0x70;                                 //profile_velocity address_L
  gbpParameter[1] = (0x70 >> 8);                          //profile_velocity address_H
  gbpParameter[2] = (unsigned char)(velocity_int);        //Writing Data  , profile_velocity
  gbpParameter[3] = (unsigned char)(velocity_int >> 8);   //Writing Data  , profile_velocity
  gbpParameter[4] = (unsigned char)(velocity_int >> 16);  //Writing Data  , profile_velocity
  gbpParameter[5] = (unsigned char)(velocity_int >> 24);  //Writing Data  , profile_velocity

  TxPacket_xm430(serialport, ID_number, 0x03, 0x06);
}

void xm430_profile_acceleration(Stream &serialport, unsigned char ID_number, int32_t acceleration) {
  acceleration = 0.0046 * acceleration; //acceleration: 0 ~ 6000[rev/min^2] -> 28 (acceleration unit: 214.577)

  gbpParameter[0] = 0x6C;                                 //profile_acceleration address_L
  gbpParameter[1] = (0x6C >> 8);                          //profile_acceleration address_H
  gbpParameter[2] = (unsigned char)(acceleration);        //Writing Data  , profile_acceleration
  gbpParameter[3] = (unsigned char)(acceleration >> 8);   //Writing Data  , profile_acceleration
  gbpParameter[4] = (unsigned char)(acceleration >> 16);  //Writing Data  , profile_acceleration
  gbpParameter[5] = (unsigned char)(acceleration >> 24);  //Writing Data  , profile_acceleration

  TxPacket_xm430(serialport, ID_number, 0x03, 0x06);
}

void xm430_init(Stream &serialport, int operating_mode, float velocity, int32_t acceleration)
{
  // veloctiy: 0.0 ~ 77.0[rev/min] -> 0 ~ 336 (velocity unit: 0.229)
  // acceleration: 0 ~ 6000[rev/min^2] -> 28 (acceleration unit: 214.577)

  // Operating Mode(11) : 위치제어 모드, value : 3
  // Drive Mode(10) : Profile Configuration([0] Velocity-based Profile: 속도를 기준으로 Profile 생성), value : 0
  // Profile Velocity(112) : Drive Mode(10)에서 Velocity-based Profile이 선택된 경우, Profile Velocity(112)는 Profile의 최대 속도를 설정
  //                          단위: 0.229 [rev/min], 4 byte
  // Profile Acceleration(108) : Drive Mode(10)에서 Velocity-based Profile이 선택된 경우, Profile Acceleration(108)은 Profile의 가속도를 설정
  //                          단위: 214.577 [rev/min^2], 4 byte
  xm430_Torque(serialport, 0xFE, 0);

  xm430_Operating_mode(serialport, 0xfe, operating_mode);
  xm430_profile_velocity(serialport, 0xfe, velocity);
  xm430_profile_acceleration(serialport, 0xfe, acceleration);

  xm430_Torque(serialport, 0xFE, 1);
}

void setup()
{
  CONTROL CTL;
  delay(500);

  Serial.begin(1000000);         // SBC
  Serial.setTimeout(20);

  const int BUFFER_SIZE = 6;
  char buf[BUFFER_SIZE + 1];

  while (!CTL.jetson_state_return()) {
    if (Serial.available() > 0) {
      int rlen = Serial.readBytes(buf, BUFFER_SIZE);
      char checksum = 0;
      for (int i = 2; i < rlen - 1; i++) {
        checksum += buf[i];
      }
      if (~checksum == buf[rlen - 1]) {
        CTL.serial_data_state(buf[2], buf[3], buf[4]);
      }
    }
  }
  Serial1.begin(57600);       // XM430-W210-T
  Serial1.setTimeout(20);
  Serial2.begin(57600);       // XM430-W210-T
  Serial2.setTimeout(20);

  const int bat_amount = A0;
  const int js_x = A1;
  const int js_y = A2;
  const int js_btn = 10;
  const int stop_btn = 12;
  const int press_btn = 11;

  const int32_t acceleration = 1000;

  pinMode(js_btn, INPUT);
  pinMode(press_btn, INPUT);
  pinMode(tx_enable, OUTPUT);

  const int motor_id = 0xFE;
  int angle_min, angle_max, motor_move_time;
  bool motor_toggle = true;
  xm430_init(Serial1, 3, velocity, acceleration);

  long time_lcd = millis();
  long time_motor = millis();

  CTL.print_state(map(analogRead(bat_amount), 0, 1023, 0, 100), BOOT);
  CTL.print_menu();

  long _cnt = 0;
  while (true) {
    if (time_lcd + 80 < millis()) {
      CTL.control(Serial, map(analogRead(js_x), 0, 1023, 0, 3), map(analogRead(js_y), 0, 1023, 2, -1), digitalRead(js_btn), digitalRead(press_btn));
      //      CTL.control(Serial, map(analogRead(js_x), 0, 1023, 0, 3), map(analogRead(js_y), 0, 1023, 2, -1), digitalRead(js_btn), 1);
      CTL.print_state(map(analogRead(bat_amount), 0, 1023, 0, 100), -1);
      CTL.print_menu();
      time_lcd = millis();
      //      Serial.print("200ms TIMER: ");
      Serial2.print("abc");
      Serial2.println(_cnt);
      //      _cnt++;
    }

    if (time_motor + motor_move_time < millis()) {
      CTL.get_angle(angle_min, angle_max);
      motor_move_time = ((velocity * 64 * 4.364) / (0.0046 * acceleration)) + ((64 * (angle_max - angle_min)) / (velocity * 4.364)) + 800; // 600ms -> 정지 시간
      if (motor_toggle) {
        xm430_position(Serial1, motor_id, angle_min);
      }
      else {
        xm430_position(Serial1, motor_id, angle_max);
      }
      motor_toggle = !motor_toggle;

      time_motor = millis();
    }

    if (Serial.available() > 0) {
      int rlen = Serial.readBytes(buf, BUFFER_SIZE);
      char checksum = 0;
      for (int i = 2; i < rlen - 1; i++) {
        checksum += buf[i];
      }
      if (~checksum == buf[rlen - 1]) {
        CTL.serial_data_state(buf[2], buf[3], buf[4]);
      }
    }
    if (!digitalRead(stop_btn)) {
      send_data(Serial, 0x98, 0x43, 0x17);
      CTL.sector_clear(0, 0, 20);
      CTL.sector_clear(0, 1, 20);
      CTL.sector_clear(0, 2, 20);
      CTL.sector_clear(0, 3, 20);
      CTL.set_cursor(5, 1);
      CTL.print_str("EMERGENCY");
      CTL.set_cursor(8, 2);
      CTL.print_str("STOP");
      break;
    }
    if (map(analogRead(bat_amount), 0, 1023, 0, 100) < 70) {
      send_data(Serial, 0x98, 0x43, 0x17);
      CTL.sector_clear(0, 0, 20);
      CTL.sector_clear(0, 1, 20);
      CTL.sector_clear(0, 2, 20);
      CTL.sector_clear(0, 3, 20);
      CTL.set_cursor(8, 1);
      CTL.print_str("LOW");
      CTL.set_cursor(6, 2);
      CTL.print_str("BATTERY!");
      break;
    }

  }
  while (1) {
    send_data(Serial, 0x98, 0x43, 0x17);
  }
}

void loop()
{
}
