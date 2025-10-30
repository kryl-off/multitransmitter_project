Наработки Андрея, legacy код и подключение проекта

+12v и плюс датчика дыма в NC реле
-12v на общую землю

ESP32 отправляет UDP-сообщения на заданный адрес при изменении состояния датчиков и каждые 5 секунд для поддержания связи

Пакет-пинг (keep-alive)
Отправляется каждые 5 секунд для проверки соединения.
Содержит два байта:
01 FF
где
0x01 – маркер начала пакета
0xFF – код «ping».

Пакет события (sensor event)
Отправляется при изменении состояния кнопки или аналогового сенсора.
Формат (в байтах):

0x01 – стартовый байт
type – код события (1 байт)
sensor_id – номер сенсора (1 байт)
timestamp – метка времени UNIX +3 часа, 4 байта (в формате little-endian)

Общий размер пакета – 6 байт (без заголовка UDP).




чтобы запустить - изменить в прошивке esp32 переменные с именем и паролем wifi сети на актуальные, адрес на актуальный адрес принимающего компьютера

прошивка:

Открой Arduino IDE 2.x (обновлённая).
USB-C кабель с передачей данных

Настройка IDE под ESP32
1) Arduino IDE → Preferences → Additional Boards Manager URLs и добавь:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

2) Tools → Board → Boards Manager… → найди “esp32 by Espressif Systems” → Install (последняя версия 3.x).

Библиотеки:
1) Sketch → Include Library → Manage Libraries… → установи GyverNTP (by AlexGyver).
2) Остальные (WiFi, WiFiUdp) идут в составе ядра ESP32.

