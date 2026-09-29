# LDS-006: thay curoa "không chính hãng" làm tốc độ quay tăng lên nhưng board gốc không hề chỉnh PWM, nên đây là bộ điều khiển PWM tự thích ứng (dùng curoa gì cũng được, kể cả một cọng thun)

> **English summary.** The LDS-006 lidar (from Ecovacs Deebot robot vacuums) reports its own
> head speed in every data frame. After replacing the worn drive belt with a non-original one
> (a 3D-printed TPU belt), the measured head speed went **up**, but the stock motor drive
> never adjusted its PWM, so the sensor rejected every revolution (all frames became `0xFB`
> "speed error"). This project moves motor control onto an **ESP32-S3** that reads the speed
> straight from the lidar's own data line and closes a **PID loop** on it. The PWM adapts
> to whatever belt you use: an original one, a printed one, an O-ring, even a rubber band.
> It also has a live scan map in the browser, PID tuning, and OTA updates over WiFi.

---

## Mục lục

1. [Câu chuyện: vì sao có dự án này](#1-câu-chuyện-vì-sao-có-dự-án-này)
2. [Nó hoạt động như thế nào](#2-nó-hoạt-động-như-thế-nào)
3. [Linh kiện cần có](#3-linh-kiện-cần-có)
4. [Nối dây](#4-nối-dây)
5. [Build và nạp firmware](#5-build-và-nạp-firmware)
6. [Sử dụng](#6-sử-dụng)
7. [Tinh chỉnh PID cho curoa của bạn](#7-tinh-chỉnh-pid-cho-curoa-của-bạn)
8. [Giao thức lệnh qua USB / WebSocket](#8-giao-thức-lệnh-qua-usb--websocket)
9. [Xử lý sự cố](#9-xử-lý-sự-cố)
10. [Cấu trúc mã nguồn](#10-cấu-trúc-mã-nguồn)
11. [Tham khảo](#11-tham-khảo)

---

## 1. Câu chuyện: vì sao có dự án này

Curoa zin của LDS-006 lâu ngày bị giãn, trượt, rồi đứt. Tôi thay bằng một sợi curoa in 3D
bằng nhựa dẻo TPU. Đầu quét quay lại được, nhưng lidar **không trả về dữ liệu đo nào**:

- Mỗi frame của LDS-006 có một trường **tốc độ đầu quét** (byte 2–3) do chính cảm biến đo
  bằng một photo-interrupter trong đầu quay.
- Với sợi curoa mới, trường tốc độ này **tăng lên rõ rệt** so với khi dùng curoa zin (đường kính
  hiệu dụng và độ bám khác nhau, nên cùng một điện áp motor sẽ cho tốc độ đầu quét khác).
- Mạch điều khiển motor trên board gốc **không hề điều chỉnh PWM** để bù lại. Tốc độ nằm
  ngoài cửa sổ mà cảm biến chấp nhận, nên **mọi frame đều bị đánh dấu `0xFB` (lỗi tốc độ)**,
  khoảng cách `0x7777`, độ phản xạ 0. Lidar quay nhưng mù.

Cửa sổ tốc độ mà cảm biến chấp nhận khá hẹp. Trên con lidar của tôi:

| Tốc độ đặt (giá trị trường speed) | Kết quả |
|---|---|
| < 21 500 | gần như toàn bộ frame là `0xFB` |
| **21 500 – 23 750** | **≥ 96,5 % frame hợp lệ** |
| 22 700 (tâm cửa sổ) | 100 % frame hợp lệ khi đã ổn định |
| > 23 750 | gần như toàn bộ frame là `0xFB` |

Vì vậy tôi tách motor khỏi board lidar và để một con ESP32-S3 lái nó bằng PID, **lấy phản hồi
tốc độ từ chính luồng dữ liệu của lidar**. Không cần encoder, không cần sửa cơ khí. Curoa
trơn hơn, chặt hơn, lỏng hơn, to hơn hay nhỏ hơn thì PID sẽ tự tìm duty PWM phù hợp để giữ
đầu quét ở tâm cửa sổ tốc độ. Nên bạn có thể dùng **bất kỳ thứ gì làm curoa, kể cả một cọng thun**.

---

## 2. Nó hoạt động như thế nào

```
                      ┌──────────── dữ liệu quét + tốc độ đầu quét ────────────┐
                      │                                                          ▼
 ┌─────────┐  PWM  ┌──┴──────┐   quay    ┌───────────────┐   UART 115200   ┌──────────────┐
 │ ESP32-S3│──────►│ MOSFET  │─────────► │ motor + curoa │ ─ ─ ─ ─ ─ ─ ─ ► │ LDS-006 TX   │
 │  (PID)  │ GPIO8 │ + diode │           │   (bất kỳ)    │                 │ (frame 22 B) │
 └────▲────┘       └─────────┘           └───────────────┘                 └──────┬───────┘
      │ GPIO18 (UART1 RX)                                                         │
      └───────────────────────────────────────────────────────────────────────────┘
```

### 2.1 Khung dữ liệu LDS-006

UART 115200 8N1, mỗi frame 22 byte:

| Byte | Ý nghĩa |
|---|---|
| 0 | `0xFA`: byte đồng bộ |
| 1 | chỉ số `0xA0`–`0xF9`. Góc = `(index − 0xA0) × 4`, nên 90 frame cho một vòng. **`0xFB` = lỗi tốc độ**: cảm biến từ chối tốc độ hiện tại |
| 2–3 | **tốc độ đầu quét**, uint16 little-endian (lớn hơn = nhanh hơn) |
| 4–19 | 4 × (khoảng cách uint16 LE, độ phản xạ uint16 LE) |
| 20–21 | checksum = tổng các byte 0–19 (uint16 LE) |

Mã "không có phản hồi" cho từng điểm đo là `0x7777`, `0x8888` và `0x9999`. Điểm có độ phản xạ ≤ 10
được coi là không hợp lệ.

Ngay cả frame `0xFB` vẫn mang trường tốc độ, nên bộ điều khiển luôn có phản hồi, kể cả khi
tốc độ đang nằm ngoài cửa sổ.

### 2.2 Chuỗi khởi động (chế độ AUTO)

Firmware luôn khởi động **từ cao xuống thấp**: đẩy mạnh cho đầu quét quay, rồi hạ dần về tốc độ đặt.

| Trạng thái | Việc làm |
|---|---|
| **KICK** | Chạy vòng hở ở duty 720/799, cứ 3 s tăng thêm 20 (tối đa 799) cho đến khi có frame. Lidar im lặng khi đầu quét chưa quay |
| **SETTLE** | Đợi 1 s để bộ lọc trung vị báo được tốc độ lúc kick |
| **RAMP** | Bật PID *không giật* (bumpless) từ duty đang kick, rồi hạ tốc độ đặt 2500 đơn vị/s từ tốc độ hiện tại xuống mục tiêu. Nếu nhảy thẳng xuống một duty cố định thì tốc độ sẽ tụt quá đà, ra khỏi cửa sổ |
| **PID** | Giữ tốc độ đặt (mặc định 22 700). Nếu mất frame quá 1 s (kẹt, trượt curoa) thì quay lại KICK |
| **REST** | Kick 25 s mà vẫn không có frame: tắt motor 30 s rồi thử lại, tránh nung motor khi có lỗi dây |

### 2.3 Lọc tín hiệu tốc độ

- **Trung vị 5 mẫu.** Trường tốc độ có một xung "chỉ mục" mỗi vòng (khoảng +33 % rồi −20 %).
  Xung này là bình thường, và bộ lọc trung vị loại nó đi.
- **Loại mẫu vô lý.** Nhiễu điện có thể làm hỏng một frame mà checksum cộng đơn giản vẫn không
  phát hiện được. Mẫu lệch quá 40 % so với giá trị đã lọc sẽ bị bỏ. Nếu 5 mẫu liên tiếp đều lệch
  thì coi đó là thay đổi thật (ví dụ curoa trượt) và bộ lọc khởi động lại.

### 2.4 PID

- PID dạng vị trí với **tích phân được kẹp trong [min, max] duty** (chống bão hòa tích phân).
- Khâu vi phân tính **trên giá trị đo** (không giật khi đổi tốc độ đặt).
- **Giới hạn tốc độ thay đổi duty** (slew) trên mỗi lần cập nhật.

| Tham số | Mặc định |
|---|---|
| Kp | 0.0015 |
| Ki | 0.006 |
| Kd | 0 |
| Duty min–max (thang 0..799) | 580–760 |
| Slew | 2 / lần cập nhật |
| Tốc độ đặt | 22 700 |
| Tần số PWM | 2 kHz |

Các giá trị này được tinh chỉnh trên phần cứng của tôi. **Hãy tinh chỉnh lại cho curoa và mạch
lái của bạn** (xem [mục 7](#7-tinh-chỉnh-pid-cho-curoa-của-bạn)).

### 2.5 Kiến trúc firmware

- **Core 1** chạy một task điều khiển duy nhất, sở hữu toàn bộ trạng thái (không cần khóa):
  đọc UART1 (lidar), UART2 (lệnh của robot), USB và hàng đợi lệnh từ web, chạy PID, xuất PWM.
- **Core 0** chạy WiFi (SmartConfig), mDNS, HTTP/WebSocket server và OTA.
- **PWM** dùng MCPWM, xung nhịp 80 MHz. Duty luôn ở thang 0..799 (tương thích bản Arduino Uno),
  còn độ phân giải thật thì mịn hơn nhiều (40 000 tick ở 2 kHz).
- **Log hệ thống** (`ESP_LOG`) đi ra UART0 (không nối). Cổng USB chỉ mang giao thức lệnh, nên
  có thể nói chuyện với nó bằng bất kỳ serial terminal nào.
- ESP32-S3 **chỉ nghe** (RX only) trên cả hai đường UART. Nó không bao giờ lái đường dữ liệu
  của lidar hay của robot, nên có thể gắn song song vào robot mà không ảnh hưởng gì.

---

## 3. Linh kiện cần có

| Linh kiện | Ghi chú |
|---|---|
| Lidar LDS-006 | Từ robot Ecovacs Deebot |
| Board ESP32-S3 | Đã thử với ESP32-S3 8 MB flash, cổng USB native (USB-Serial/JTAG). Cần **≥ 8 MB flash** cho bảng phân vùng OTA |
| MOSFET kênh N **logic-level** | Ví dụ **AO3400A**, IRLZ44N, IRLML6344. Chân GPIO chỉ ra 3,3 V |
| Diode Schottky | Ví dụ SS14 / 1N5819, làm diode dập ngược (flyback) song song với motor |
| Điện trở 100–220 Ω | Nối tiếp chân gate |
| Điện trở 10 kΩ | Kéo gate xuống GND, để motor tắt khi ESP đang reset |
| Điện trở 220 Ω – 1 kΩ | Nối tiếp đường TX của lidar vào ESP |
| Tụ 100 nF | Hàn trực tiếp giữa hai cực motor |
| Tụ 470 µF | Tụ lọc nguồn motor |
| Nguồn 5 V | Cho lidar và motor. Motor nên có nguồn riêng hoặc dây riêng về nguồn |
| (Tùy chọn) Bộ chuyển USB-UART | Ví dụ USB2LDS / CH340 / CP2102, để gửi `startlds$` từ PC khi không có robot |

> ⚠️ **Không nên dùng module IRF520N với ESP32.** IRF520N không phải MOSFET logic-level: với gate
> 3,3 V nó chỉ mở một phần, nên ngay cả ở 100 % duty motor cũng không đủ lực. Nếu buộc phải dùng
> nó, hãy thêm một buffer 5 V (ví dụ 74HCT1G125) trước gate.

---

## 4. Nối dây

### 4.1 Bảng chân mặc định

| Chức năng | GPIO ESP32-S3 | Đổi ở đâu |
|---|---|---|
| **Lidar TX → ESP** (UART1 RX) | **GPIO18** | `menuconfig`, hoặc lệnh `U <gpio>` lúc đang chạy (lưu vào NVS) |
| **Robot/PC TX → ESP** (UART2 RX, nghe `startlds$`/`stoplds$`) | **GPIO17** | `CONFIG_LDS_ROBOT_RX_GPIO` trong `sdkconfig.defaults` (−1 = không dùng) |
| **PWM → gate MOSFET** | **GPIO8** | `CONFIG_LDS_PWM_GPIO` trong `sdkconfig.defaults` |
| **Nút bấm** | **GPIO0** (nút BOOT sẵn có) | `CONFIG_LDS_BUTTON_GPIO` |
| GND | GND | Chung GND giữa lidar, ESP và nguồn motor |

Tránh các chân: GPIO0/3/45/46 (strapping), 19/20 (USB), 26–32 (flash/PSRAM), 43/44 (UART0 console).

### 4.2 Sơ đồ

Bước đầu tiên: **tháo hai dây motor ra khỏi board lidar.** Motor giờ do ESP32 lái qua MOSFET.
Board lidar vẫn được cấp 5 V như bình thường để đo và gửi dữ liệu.

```
                         Nguồn 5 V motor
                              │
                     ┌────────┼─────────┬──────────┐
                     │        │         │          │
                  470µF    Schottky   100nF     (+) MOTOR
                     │     (vạch ↑)     │          │
                     │        │         │       (−) MOTOR
                     │        └─────────┴──────────┤
                     │                             │
                     │                          Drain
 ESP32-S3            │                     ┌───────────┐
  GPIO8 ──[100–220Ω]─┼────────────────Gate │  AO3400A  │
                     │           │         │ (N-MOSFET)│
                     │        [10kΩ]       └───────────┘
                     │           │            Source
                     │           │               │
  GND ───────────────┴───────────┴───────────────┴──── GND nguồn motor  (★ điểm nối sao)


  LDS-006 (board lidar, đã tháo motor)
    5V  ─────────────────────────── 5 V
    GND ─────────────────────────── GND (★)
    TX  ──[220Ω–1kΩ]──┬──────────── ESP32 GPIO18   (UART1 RX)
                      └──────────── RX của robot / bộ USB-UART (nếu có)
    RX  ◄───────────────┬────────── TX của robot / bộ USB-UART  (gửi startlds$ / stoplds$)
                        └────────── ESP32 GPIO17   (UART2 RX, chỉ nghe, tùy chọn)
```

- Diode Schottky mắc **ngược** song song với motor: vạch (cathode) về phía +5 V, anode về phía drain.
- Hàn tụ 100 nF **ngay tại chân motor**. Xoắn đôi hai dây motor.
- **Nối đất hình sao**: dây GND của motor đi thẳng về nguồn, không đi chung đường với GND của
  lidar hay ESP. Nhiễu từ motor là kẻ thù số một của đường UART lidar (xem [mục 9](#9-xử-lý-sự-cố)).
- ESP32-S3 được cấp nguồn qua cổng USB, hoặc qua chân 5 V của board từ cùng nguồn (nhớ chung GND).

### 4.3 Lệnh `startlds$`: ai gửi?

LDS-006 chỉ phát dữ liệu sau khi **nhận lệnh `startlds$` trên chân RX của nó**. ESP32 trong dự án
này **chỉ nghe**, không gửi lệnh này. Có hai cách dùng:

| Cách dùng | Ai gửi `startlds$` | ESP làm gì |
|---|---|---|
| **Lắp lại vào robot** | Board chính của robot, như bình thường | Nghe được `startlds$` trên GPIO17 thì tự vào AUTO và quay motor. Nghe `stoplds$` thì dừng motor |
| **Dùng với PC / máy tính nhúng** | Chương trình của bạn, qua một bộ USB-UART nối vào RX lidar | Như trên nếu nối GPIO17 vào đường đó. Hoặc bật motor bằng tay (nút BOOT, lệnh `M 1`, nút "Bật" trên web) |

> Vì sao ESP không tự gửi `startlds$`? Một phiên bản trước từng chuyển GPIO17 sang TX để tự gửi.
> Nhưng khi dùng chung đường với bộ USB2LDS (chip CH9102), chân TX của bộ chuyển giữ đường ở mức
> cao mà không có điện trở nối tiếp, nên đường dây không bao giờ đổi theo. Tính năng đó đã được
> bỏ. Để ESP gửi được lệnh, chỉ nên có **một** thiết bị lái chân RX của lidar.

Lệnh cuối cùng của robot được giữ trong RTC memory qua các lần soft reset (ví dụ reboot sau OTA),
nên motor tự quay lại mà không phải đợi robot gửi lại `startlds$`.

---

## 5. Build và nạp firmware

Cần **ESP-IDF v5.5.x** (đã thử với v5.5.2). Thành phần `espressif/mdns` được IDF Component
Manager tự tải về trong lần build đầu.

```bash
git clone https://github.com/kenhtaymay/ESP_LDS_006.git
cd ESP_LDS_006
idf.py set-target esp32s3      # chỉ cần lần đầu
idf.py build
idf.py -p <PORT> flash monitor # PORT: COMx trên Windows, /dev/ttyACM0 trên Linux
```

- Nếu muốn đổi chân GPIO hoặc hostname, chạy `idf.py menuconfig` → **LDS-006 motor controller**,
  hoặc sửa `sdkconfig.defaults` rồi xóa `sdkconfig` trước khi build lại.
- `idf.py monitor` chỉ hiển thị giao thức lệnh (dòng `READY`, `T ...`, `V ...`). Log hệ thống đi ra UART0.
- Build sạch lần đầu biên dịch khoảng 1 000 file, mất vài phút.

### Cập nhật firmware qua WiFi (OTA)

Firmware có hai slot ứng dụng 3 MB và bootloader tự động **rollback** nếu bản mới không khởi động được.

- **Trên web:** mục *Cập nhật firmware (OTA)*, chọn `build/lds_motor_idf.bin`, bấm *Tải lên & khởi động lại*.
- **Từ dòng lệnh:**
  ```bash
  python tools/ota_upload.py <ip-của-esp>
  ```
- **Lưu ý:** `idf.py flash` qua USB ghi lại otadata (khởi động slot `ota_0`), còn `idf.py app-flash`
  thì không đổi slot đang chạy.

---

## 6. Sử dụng

### 6.1 Lần đầu tiên

1. **Nạp firmware xong, motor sẽ không tự quay.** AUTO mặc định tắt, để chân PWM chưa được kiểm tra
   không bị lái ngay khi khởi động.
2. **Kết nối WiFi bằng SmartConfig:**
   - Firmware nhận WiFi theo giao thức SmartConfig **ESPTouch (v1)** hoặc **AirKiss**
     (`SC_TYPE_ESPTOUCH_AIRKISS` trong `main/wifi.c`). Dùng bất kỳ app nào hỗ trợ một trong hai
     giao thức này. (App ESPTouch gốc của Espressif không còn cài được trên nhiều máy Android đời mới.)
   - Điện thoại cần ở cùng mạng WiFi **2,4 GHz**. Nhập mật khẩu WiFi trong app rồi gửi.
   - ESP tự vào SmartConfig khi chưa có WiFi, hoặc khi không kết nối được trong 30 s.
   - Thông tin WiFi được lưu vào flash.
3. **Tìm địa chỉ IP:**
   - Mở serial terminal trên cổng USB của ESP (baud rate không quan trọng với USB native) và đọc dòng
     `I wifi=connected ip=192.168.x.y hostname=lds006.local`.
   - Hoặc truy cập `http://lds006.local` (mDNS; tùy mạng và hệ điều hành có thể không phân giải được).
   - Hoặc xem danh sách thiết bị trên router.
4. Mở `http://<ip>` trong trình duyệt.
5. Làm cho lidar phát dữ liệu (robot hoặc PC gửi `startlds$`), rồi bấm **Tự động (G)** hoặc **Motor: Bật**.
6. Theo dõi motor đi qua KICK → SETTLE → RAMP → PID, và tỉ lệ frame hợp lệ lên gần 100 %.
7. Bấm **Áp dụng & lưu** (lệnh `W`) để lưu cấu hình. Nếu đang ở chế độ AUTO lúc lưu, lần khởi động
   sau motor sẽ tự vào AUTO.

### 6.2 Giao diện web

| Mục | Nội dung |
|---|---|
| **Bản đồ quét thời gian thực** | 360 điểm/vòng qua WebSocket, có chỉnh phạm vi và xoay |
| **Điều khiển motor** | Tự động (G), Dừng (X), tốc độ đặt (S), Motor Bật/Tắt (M), duty vòng hở (P) |
| **Hiệu chỉnh PID** | Kp, Ki, Kd, Min, Max, Slew, PWM Hz. *Áp dụng* / *Áp dụng & lưu* / *Nạp lại* |
| **Motor / Lidar** | Trạng thái, tốc độ đã lọc, duty, % frame hợp lệ, frame/s, vòng/s, số lần khởi động lại, lỗi checksum |
| **WiFi / Hệ thống / Phần cứng** | SSID, IP, RSSI, bảng chân GPIO. Nút *Quên WiFi & SmartConfig* |
| **Cập nhật firmware (OTA)** | Tải file `.bin` lên |

Trạng thái được đẩy lên trang khoảng 4 lần/giây.

### 6.3 Nút BOOT (GPIO0)

| Thao tác | Tác dụng |
|---|---|
| Nhấn ngắn (< 1 s) | Bật/tắt motor (giống lệnh `M`). Bật thì chạy chuỗi AUTO mà không cần đợi `startlds$` |
| Giữ 3 s | Cảnh báo trên USB: "tiếp tục giữ để xóa WiFi" |
| Giữ 8 s | Quên WiFi và vào SmartConfig |

Nếu nút đang được giữ ngay lúc khởi động (ví dụ khi vào chế độ nạp), nó sẽ bị bỏ qua cho đến khi
được nhả ra một lần. Nhờ vậy giữ BOOT qua một lần restart không xóa nhầm WiFi.

### 6.4 Dùng trong robot

Nối GPIO17 vào đường **robot TX → lidar RX**. Robot gửi `startlds$` thì ESP tự vào AUTO và quay
motor, robot gửi `stoplds$` thì motor dừng. Khi không cắm USB, dữ liệu gửi ra USB bị bỏ đi, nên
vòng điều khiển không bao giờ phải chờ.

---

## 7. Tinh chỉnh PID cho curoa của bạn

Mỗi sợi curoa (và mỗi mạch lái MOSFET) cần duty khác nhau cho cùng một tốc độ. Quy trình gợi ý:

1. **Tìm vùng duty.** Trên web, bấm *Chạy vòng hở* với duty từ cao xuống thấp (ví dụ 780, 760,
   740, …). Ghi lại tốc độ đã lọc ở mỗi mức, và mức duty mà motor bắt đầu đứng (stall).
2. **Đặt giới hạn duty** (`L <min> <max>`):
   - `max` cao hơn một chút so với duty cho tốc độ ~23 750.
   - `min` thấp hơn một chút so với duty cho ~21 500, và có thể thấp hơn cả điểm stall khi đang
     chạy (motor ấm cần ít duty hơn; nếu có stall thì firmware tự kick lại).
3. **Chạy AUTO** và quan sát. Tốc độ dao động quanh setpoint thì giảm Ki / Kp. Tốc độ tiến về
   setpoint quá chậm thì tăng Ki.
4. Curoa trơn hoặc hay trượt (như dây thun) thì nên giảm **Slew** để duty thay đổi mượt hơn.
5. Nếu tỉ lệ frame hợp lệ không đạt ~100 % ở 22 700, thử dò **tốc độ đặt** trong khoảng 21 500 –
   23 750 để tìm tâm cửa sổ của con lidar của bạn.
6. Bấm **Áp dụng & lưu**.

---

## 8. Giao thức lệnh qua USB / WebSocket

Mỗi lệnh một dòng. Gửi qua cổng USB (USB-Serial/JTAG) hoặc qua WebSocket `/ws` của trang web.

| Lệnh | Ý nghĩa |
|---|---|
| `G` | Chế độ AUTO: KICK → SETTLE → RAMP → PID |
| `M [1\|0]` | Motor bật (AUTO, không đợi `startlds$`) / tắt. Không có tham số thì đảo trạng thái |
| `X` | Dừng motor, thoát AUTO |
| `S <speed>` | Tốc độ đặt (ở HOST mode thì đồng thời bật PID) |
| `P <duty>` | HOST mode: duty vòng hở 0..799 |
| `K <kp> <ki> <kd>` | Hệ số PID |
| `L <min> <max>` | Giới hạn duty cho PID |
| `R <steps>` | Giới hạn thay đổi duty mỗi lần cập nhật |
| `Q <hz>` | Tần số PWM (1221 Hz – 800 kHz, mặc định 2000; lưu bằng `W`) |
| `W` | Lưu cấu hình (và cờ tự vào AUTO khi khởi động) vào NVS |
| `U <gpio>` | Chuyển chân RX của lidar sang GPIO khác (lưu ngay) |
| `D` | Chẩn đoán chân RX lidar: mức, số cạnh, số byte |
| `C` | Quên WiFi, vào SmartConfig |
| `?` | In trạng thái đầy đủ (dòng `STAT ...`) |
| `F <speed>`, `A` | Bỏ qua / keepalive (tương thích công cụ PC cũ) |

**Dữ liệu ra (USB):**

```
READY lds_motor_idf v2 top=799
T <ms> <setpoint> <filtered> <duty> <state>     # 10 Hz
V valid=<n> errors=<n> pct=<p>                  # ~1 Hz, tỉ lệ frame hợp lệ
I wifi=connected ip=... hostname=lds006.local   # khi WiFi thay đổi
OK ... / ERR ... / STAT ... / DIAG ...
```

**HOST mode:** các lệnh `P`/`S` chuyển firmware sang chế độ để máy tính điều khiển. Máy tính phải
gửi một lệnh bất kỳ (ví dụ `A`) ít nhất mỗi 3 s, nếu không motor sẽ dừng (an toàn khi mất kết nối).

---

## 9. Xử lý sự cố

### Motor quay nhưng không có frame nào (trạng thái KICK rồi REST)

- Lidar đã nhận `startlds$` chưa? (xem [4.3](#43-lệnh-startlds-ai-gửi))
- Gửi `D` để chẩn đoán chân RX:
  - `signal present`: có tín hiệu. Nếu vẫn không có frame thì kiểm tra baud rate hoặc nhiễu.
  - `idle high: no data on this pin`: dây nối đúng chân nhưng lidar không phát, hoặc nhầm chân.
  - `stuck low`: nhầm dây, thiếu GND chung, hoặc chân đang bị thiết bị khác kéo xuống.
- Kiểm tra lại bằng `?`: `raw_bytes` tăng mà `frames` không tăng nghĩa là dữ liệu đến nhưng bị hỏng.

### Nhiều frame lỗi checksum / tỉ lệ hợp lệ thấp khi motor chạy

Đây là **nhiễu từ motor** lọt vào đường UART của lidar. Tôi đã đo số frame hỏng checksum ở duty
720 với dây nối chưa xử lý nhiễu:

| Tần số PWM | Frame hỏng |
|---|---|
| 20 kHz | 72 % |
| 10 kHz | 48 % |
| 5 kHz | 32 % |
| 2 kHz | 22 % |
| DC (duty 799) | 17 % |

Cách khắc phục, **ưu tiên phần cứng trước**:

1. Diode Schottky dập ngược song song với motor.
2. Tụ 100 nF hàn ngay tại chân motor.
3. Nối đất hình sao: GND motor đi riêng về nguồn.
4. Tụ 470 µF trên nguồn motor.
5. Xoắn đôi dây motor, để xa dây tín hiệu lidar.
6. Điện trở 100–220 Ω nối tiếp gate.
7. Hạ tần số PWM bằng `Q <hz>` (mặc định đã là 2 kHz; motor có thể kêu rít nhẹ).

Về phía phần mềm, firmware đã loại bỏ mẫu tốc độ vô lý (xem [2.3](#23-lọc-tín-hiệu-tốc-độ))
để PID không bị nhiễu kéo lệch.

### Motor không đủ lực ở 100 % duty

MOSFET không phải loại logic-level (ví dụ IRF520N). Thay bằng AO3400A hoặc thêm buffer 5 V cho gate.

### Tốc độ dao động, tỉ lệ hợp lệ lên xuống

Giảm Kp/Ki hoặc Slew. Kiểm tra curoa có trượt không. Với curoa trơn, thêm chút lực căng.

### Không vào được trang web

- Đọc IP qua cổng USB (dòng `I wifi=...`).
- mDNS `lds006.local` không phải mạng nào cũng hỗ trợ, nên hãy dùng IP.
- ESP32 chỉ hỗ trợ WiFi 2,4 GHz.

---

## 10. Cấu trúc mã nguồn

```
ESP_LDS_006/
├── CMakeLists.txt          # dự án ESP-IDF
├── sdkconfig.defaults      # target esp32s3, 8 MB flash, chân GPIO, OTA rollback, WebSocket
├── partitions.csv          # nvs + otadata + 2 × 3 MB app (OTA)
├── dependencies.lock       # espressif/mdns
├── main/
│   ├── main.c              # task điều khiển, giao thức lệnh, UART, nút BOOT, NVS
│   ├── controller.c/.h     # máy trạng thái KICK/SETTLE/RAMP/PID/REST, PID, bộ lọc trung vị
│   ├── lidar.c/.h          # bộ phân tích frame 22 byte, kiểm tra checksum, tự đồng bộ lại
│   ├── motor_pwm.c/.h      # MCPWM, duty thang 0..799, đổi tần số lúc chạy
│   ├── scan.c/.h           # bản đồ 360 điểm cho web
│   ├── wifi.c/.h           # WiFi STA + SmartConfig + mDNS
│   ├── web.c/.h            # HTTP, WebSocket (bản đồ + trạng thái JSON), OTA
│   ├── web/index.html      # giao diện web (nhúng vào firmware)
│   ├── Kconfig.projbuild   # menu cấu hình chân GPIO, hostname
│   └── idf_component.yml
└── tools/
    └── ota_upload.py       # nạp firmware qua WiFi
```

---

## 11. Tham khảo

- Bài phân tích ngược giao thức LDS-006 trên jentsch.io:
  <https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/>
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/)
