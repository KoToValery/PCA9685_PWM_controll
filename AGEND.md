# LED Индикация - Логика и Архитектура

## Общ преглед

Адонът използва RGB LED само по време на стартовата диагностика. След като стартовият резултат е показан, Home Assistant поема управлението на RGB LED чрез MQTT entity:

```text
light.pca_rgb_led
```

Runtime hardware проблемите продължават да се следят от `pca9539_worker()` и се публикуват като MQTT feedback binary sensors. HAOS автоматизациите могат да използват тези sensors, за да изберат цвят, мигане или друга логика за RGB LED.

## Поведение при старт

При стартиране:

1. RGB LED показва diagnostic цвета.
2. Изпълняват се initialization проверките.
3. Ако има warning/error при инициализация, RGB LED мига с error цвета за `STARTUP_RESULT_BLINK_DURATION`.
4. Ако инициализацията е успешна, RGB LED мига с OK цвета за `STARTUP_RESULT_BLINK_DURATION`.
5. RGB LED се изключва.
6. `system_status` се задава на `OK`.
7. Активира се HAOS контролът върху RGB LED. Ако HAOS е изпратил команда по време на старта, последното заявено състояние се прилага тогава.

След тази точка няма runtime worker, който периодично да променя RGB LED.

## Работен режим

Home Assistant управлява RGB LED каналите 12-14 чрез:

```text
homeassistant/light/pca_rgb_led/set
homeassistant/light/pca_rgb_led/state
```

Entity-то използва Home Assistant MQTT light JSON schema с RGB color mode и brightness.

## Отговорности на компонентите

### `show_startup_result()`

- Изчаква минималното време за стартова индикация.
- Показва red/green стартов резултат.
- Изключва RGB LED.
- Задава `system_status = OK`.

### `handle_rgb_led_command()`

- Обработва HAOS MQTT JSON light команди.
- Поддържа `state`, `brightness` и `color.r/g/b`.
- Прилага active-low PWM стойности към RGB каналите 12-14.
- Публикува retained light state.

### `pca9539_worker()`

- Чете hardware feedback всяка секунда.
- Публикува feedback topics за dashboard и автоматизации.
- Обновява `any_problem_realtime`.
- Не управлява RGB LED директно.

### `sys_led_worker()`

- Управлява само CH15 system LED.
- Не управлява RGB LED.

## Резюме

| Сценарий | Собственик на RGB LED | Поведение |
| --- | --- | --- |
| Стартова диагностика | Адонът | Diagnostic цвят, после red/green резултат, после off |
| Нормална работа | Home Assistant | Управлява се от `light.pca_rgb_led` |
| Runtime hardware проблем | Home Assistant automation | Автоматизацията решава цвят/ефект според feedback sensors |
| Shutdown | Адонът | RGB каналите се изключват |

Ключово: след старта адонът вече не презаписва периодично RGB LED състоянието. Това предотвратява конфликт с HAOS автоматизации.
