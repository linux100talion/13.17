package main

import (
	"log"
	"os/exec"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

func main() {
	node := &gomavlib.Node{
		Endpoints: []gomavlib.EndpointConf{
			// Т.к. MAVProxy делает udpout, мы должны СЛУШАТЬ этот порт как Сервер
			gomavlib.EndpointUDPServer{Address: "127.0.0.1:14550"},
		},
		Dialect:     common.Dialect,
		OutVersion:  gomavlib.V2,
		OutSystemID: 255,
	}

	err := node.Initialize()
	if err != nil {
		log.Fatal("Ошибка инициализации ноды MAVLink: ", err)
	}
	defer node.Close()

	var pressStartTime time.Time
	isPressing := false
	isArmed := true 

	log.Println("Служба мониторинга MAVLink запущенаяяя, ожидание пакетов...")

	for evt := range node.Events() {
		// Используем type switch для корректной маршрутизации событий
		switch e := evt.(type) {
		
		case *gomavlib.EventChannelOpen:
			log.Println("MAVLink канал открыт:", e.Channel)

			// ПИНАЕМ ПОЛЕТНИК, ЧТОБЫ ОН НАЧАЛ СЛАТЬ ДАННЫЕ О СТИКАХ ПУЛЬТА
            // TargetSystem: 1 (ID дрона), TargetComponent: 1 (ID автопилота)
            // ReqStreamId: 3 (MAV_DATA_STREAM_RC_CHANNELS)
            // ReqMessageRate: 10 (10 раз в секунду)
            // StartStop: 1 (Начать передачу)
            node.WriteMessageAll(&common.MessageRequestDataStream{
                TargetSystem:    1,
                TargetComponent: 1,
                ReqStreamId:     3,
                ReqMessageRate:  10,
                StartStop:       1,
            })
            log.Println("==> Запрос на поток RC_CHANNELS отправлен!")


		case *gomavlib.EventChannelClose:
			log.Println("MAVLink канал закрыт:", e.Channel)

		case *gomavlib.EventParseError:
			// Раскомментируй строку ниже, если захочешь дебажить битые пакеты
			// log.Println("Ошибка парсинга пакета:", e.Error)

case *gomavlib.EventFrame:
            // Пакет успешно распарсен, проверяем тип сообщения
            switch msg := e.Message().(type) {

            case *common.MessageHeartbeat:
                // ВАЖНО: Слушаем Heartbeat только от автопилота (Component ID = 1)
                // Игнорируем Heartbeat от Mission Planner, MAVProxy и прочих
                if e.ComponentID() == 1 {
                    // Приводим к uint32 для безопасности сравнения битовых масок
                    isArmed = (uint32(msg.BaseMode) & 128) != 0
                }

            case *common.MessageRcChannels:
                pwm := msg.Chan10Raw

                if pwm > 1500 { // Кнопка зажата
                    if !isPressing {
                        isPressing = true
                        pressStartTime = time.Now()
                        // Выводим сообщение ОДИН раз при нажатии и сразу показываем текущий статус
                        log.Printf("==> Кнопка ЗАЖАТА! Текущий статус ARMED: %v. Таймер пошел...\n", isArmed)
                    } else if time.Since(pressStartTime) >= 3*time.Second {
                        if !isArmed {
                            log.Println("Дрон задизармлен! Выполняем shutdown -h now...")
                            err := exec.Command("shutdown", "-h", "now").Run()
                            if err != nil {
                                log.Printf("Ошибка вызова shutdown: %v", err)
                            }
                            time.Sleep(10 * time.Second) 
                        } else {
                            log.Println("Отказ выключения: дрон ARMED")
                            pressStartTime = time.Now() // Сброс таймера, проверим еще через 5 сек
                        }
                    }
                } else { // Кнопка отпущена
                    if isPressing {
                        log.Println("==> Кнопка ОТПУЩЕНА. Сброс таймера.")
                    }
                    isPressing = false
                }
            }
		}
	}
}
