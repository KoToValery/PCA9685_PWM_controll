# LED Индикация - Логика и Архитектура

## Общ преглед

Системата използва **два независими източника** за състояние:

| Източник | Назначение | Потребител |
|----------|-----------|------------|
| `system_status` | Фаза на системата (DIAGNOSTIC → OK) | Стартови фази |
| `any_problem_realtime` | Реални hardware проблеми от feedback | Dashboard + LED индикация |

---

## LED Режими

### 1. Диагностика (Startup)

**Кога:** По време на `hardware_diagnostic()`

**Поведение:**
- Син LED постоянно свети
- Проверките на релета/сигнали работят на 1 секунда
- **Важно:** `verified_apply_switch()` НЕ обновява `system_status` по време на диагностика

**След край на диагностика:**
```
Ако има грешки:    Червен за 5 сек → ИЗГАСВА
Ако няма грешки:   Зелен за 5 сек  → ИЗГАСВА

След това: system_status = OK (винаги!)
```

**Защо `system_status = OK` винаги?**
- Стартовият статус е само визуална индикация
- След старта runtime проблемите се детектират от `pca9539_worker`
- Това позволява авто-възстановяване: ако проблемът изчезне, LED показва зелено

---

### 2. Работен режим (Runtime)

**Кога:** След приключване на диагностиката

**Активиране:** На всеки `led_indicator_interval` секунди (по подразбиране 30с)

**Логика на LED индикатора:**
```python
if any_problem_realtime == True:
    # Има реален hardware проблем
    МИГА ЧЕРВЕНО за 5 секунди
else:
    # Всичко е наред според feedback
    СВЕТИ ЗЕЛЕНО за 5 секунди

ИЗГАСВА за останалите 25 секунди
```

**Важно:** В работен режим LED гледа **САМО** `any_problem_realtime`, **НЕ** `system_status`!

---

## Архитектура на компонентите

### `hardware_diagnostic()`
- Сетва `system_status = DIAGNOSTIC` в началото
- Показва син LED
- Тества всички релета и сигнали
- Показва червен/зелен за 5 сек
- **Винаги сетва `system_status = OK` накрая**
- Не пипа `any_problem_realtime`

### `pca9539_worker()`
- Работи на всяка 1 секунда
- Чете feedback от PCA9539
- Публикува индивидуални topics (TOPIC_FEEDBACK_RELAY1 и т.н.)
- Изчислява `any_problem` от всички проверки
- **Сетва `any_problem_realtime = any_problem`**
- **НЕ управлява LED директно!**

### `led_indicator_worker()`
- Работи на интервали (`led_indicator_interval`)
- Чете **само** `any_problem_realtime`
- **Игнорира** `system_status`
- Мига червено (проблем) или зелено (OK) за 5 сек
- Изгася за останалото време

### `sys_led_worker()`
- Управлява **само** CH15 (системен LED)
- Мига всяка секунда
- **НЕ управлява RGB LED!**

---

## Зависимости между компонентите

```
hardware_diagnostic()
    ├── Сетва system_status = DIAGNOSTIC (начало)
    ├── Показва син LED
    ├── Тества компоненти
    ├── Показва червен/зелен за 5 сек
    └── Сетва system_status = OK (край)
        └── Стартира led_indicator_worker()

pca9539_worker() ──► any_problem_realtime ◄── led_indicator_worker()
     │                                          (мига според проблем)
     │
     └──► Публикува TOPIC_FEEDBACK_* ◄── Dashboard (Home Assistant)
```

---

## Конфигурация

### `config.yaml`

```yaml
led_indicator_interval: 30  # секунди между LED активации (5-300)
```

### `run.py` константи

```python
LED_INDICATOR_ON_DURATION = 5  # секунди за които LED свети
```

---

## Често срещани проблеми

### Проблем: LED показва зелено, но dashboard показва Problem

**Диагноза:** `led_indicator_worker` вероятно чете `system_status` вместо `any_problem_realtime`

**Решение:** Провери че `led_indicator_worker` използва:
```python
with any_problem_lock:
    has_problem = any_problem_realtime  # ✅ Правилно
    
# НЕ:
if system_status == "ERROR":  # ❌ Грешно - винаги е OK след старта
```

### Проблем: LED мига червено по време на диагностика

**Диагноза:** `verified_apply_switch` сетва `system_status = ERROR`

**Решение:** Увери се че се използва `update_status=False` по време на диагностика:
```python
verified_apply_switch(ch, True, name, update_status=False)  # ✅
```

---

## Поток на данни при проблем с реле

1. Реле 1 се включва/изключва
2. `pca9539_worker()` проверява feedback → открива несъответствие
3. Публикува `TOPIC_FEEDBACK_RELAY1 = "Problem"`
4. `any_problem_realtime = True`
5. Dashboard показва Problem
6. При следващия цикъл (30 сек), `led_indicator_worker()` вижда `any_problem_realtime = True`
7. LED мига червено за 5 сек

---

## Резюме

| Сценарий | Dashboard | LED индикация |
|----------|-----------|---------------|
| Стартова диагностика | Няма данни | Син → [Червен/Зелен 5с] → Изгасва |
| Нормална работа | OK | Зелен 5с на всеки 30с |
| Проблем с реле | Problem | Червен 5с на всеки 30с |
| Проблем изчезне | OK | Зелен 5с на всеки 30с |

**Ключова концепция:** След старта `system_status` винаги е `OK`. Всички runtime проблеми се отразяват в `any_problem_realtime` и се показват чрез периодичния LED индикатор.
