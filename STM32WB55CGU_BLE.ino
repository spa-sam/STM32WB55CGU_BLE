#include <STM32duinoBLE.h>
#include "lock_resource.h"
#define LED_PIN PE4

HCISharedMemTransportClass HCISharedMemTransport;
BLELocalDevice BLEObj(&HCISharedMemTransport);
BLELocalDevice& BLE = BLEObj;

BLEService tempService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic tempCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 10);

extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};

    /* This prevents concurrent access to RCC registers by CPU2 (M0+) */
    hsem_lock(CFG_HW_RCC_SEMID, HSEM_LOCK_DEFAULT_RETRY);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* This prevents the CPU2 (M0+) to disable the HSI48 oscillator */
    hsem_lock(CFG_HW_CLK48_CONFIG_SEMID, HSEM_LOCK_DEFAULT_RETRY);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI
        | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2
        | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the peripherals clocks
    */
    /* RNG needs to be configured like in M0 core, i.e. with HSI48 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS | RCC_PERIPHCLK_RFWAKEUP
        | RCC_PERIPHCLK_CLK48SEL | RCC_PERIPHCLK_USB
        | RCC_PERIPHCLK_RNG;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
    PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
    PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
    LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40);
    LL_PWR_SMPS_Enable();

    /* Select HSI as system clock source after Wake Up from Stop mode */
    LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);

    hsem_unlock(CFG_HW_RCC_SEMID);
}


void setup() {
    SystemClock_Config();
    pinMode(LED_PIN, OUTPUT);

    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Starting...");

    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1) {
            blinkLEDNonBlocking();
        }
    }



    BLE.setLocalName("STM32WB55 Temp Sensor");
    BLE.setAdvertisedService(tempService);
    tempService.addCharacteristic(tempCharacteristic);
    BLE.addService(tempService);

    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    tempCharacteristic.setEventHandler(BLERead, tempCharacteristicRead);

    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
}

unsigned long previousMillis = 0;
const long interval = 200;
bool ledState = LOW;

unsigned long lastTempUpdate = 0;
const long tempUpdateInterval = 1000;

void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_PIN, HIGH);

        while (central.connected()) {
            if (tempCharacteristic.written()) {
                String receivedTemp = tempCharacteristic.value();
                float setTemperature = receivedTemp.toFloat();
                Serial.print("Received new temperature setting: ");
                Serial.println(setTemperature);
            }

            updateTemperature();
            blinkLEDNonBlocking();
            BLE.poll();
        }

        digitalWrite(LED_PIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }

    BLE.poll();
    blinkLEDNonBlocking();
    updateTemperature();
}

void blinkLEDNonBlocking() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}

void updateTemperature() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastTempUpdate >= tempUpdateInterval) {
        lastTempUpdate = currentMillis;

        float temperature = random(2000, 3000) / 100.0;
        tempCharacteristic.writeValue(String(temperature, 2));

        Serial.print("Temperature updated: ");
        Serial.println(temperature);

        if (BLE.connected()) {
            tempCharacteristic.writeValue(String(temperature, 2));
            Serial.println("Temperature sent to connected device");
        }
    }
}

void blePeripheralConnectHandler(BLEDevice central) {
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
}

void tempCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
    float temperature = random(2000, 3000) / 100.0;
    tempCharacteristic.writeValue(String(temperature, 2));
    Serial.print("Temperature read: ");
    Serial.println(temperature);
}

