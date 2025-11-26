#include "task_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// 引入组件和配置
#include "board_config.h"  // 引脚定义
#include "robot_bus.hpp"   // 数据总线
#include "spi_bus.h"	   // 驱动组件
#include "icm42688.h"	   // 驱动组件
#include "i2c_bus.h"
#include "qmc5883l.h"

static const char* TAG = "TASK_SENSOR";

// --- 模块内部私有对象 (Static 限制作用域) ---
// 这些对象现在只属于 Sensor 任务，别的任务看不见，很安全
static SPIBus spi_bus(SPI2_HOST);
static ICM42688 imu(&spi_bus, PIN_SPI_CS_IMU);
static SemaphoreHandle_t sem_imu_drdy = NULL;

static I2CBus i2c_bus(I2C_NUM_0);  // 新增 I2C 总线
static QMC5883L mag(&i2c_bus);	   // 新增 磁力计

// --- 中断回调 (ISR) ---
static void IRAM_ATTR imu_isr_handler(void* arg)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sem_imu_drdy, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
	portYIELD_FROM_ISR();
}

// --- 任务实体 ---
static void task_sensor_entry(void* arg)
{
  auto& bus = RobotBus::instance();
  ImuData data = {};
  MagData mag_data = {};
  bool use_interrupt = false;
  int tick_counter = 0;
  int print_count = 0;

  // 1. 初始化硬件
  spi_bus.begin(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK);
  i2c_bus.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  vTaskDelay(pdMS_TO_TICKS(100));  // 上电缓冲

  if (imu.begin() != ESP_OK)
  {
	ESP_LOGE(TAG, "IMU Init Failed! Task Suspended.");
	vTaskDelete(NULL);
  }

  if (mag.begin() != ESP_OK)
  {
	ESP_LOGW(TAG, "Mag Init Failed! (Check wiring)");
  }
  else
  {
	// 可选：设置校准数据
	// mag.setCalibration(...);
  }

  // 2. 配置中断
  sem_imu_drdy = xSemaphoreCreateBinary();
  if (imu.enableInterrupt(PIN_IMU_INT, imu_isr_handler) == ESP_OK)
  {
	use_interrupt = true;
	ESP_LOGI(TAG, "Mode: Hardware Interrupt (Low Latency)");
  }
  else
  {
	ESP_LOGW(TAG, "Mode: Polling (Fallback)");
  }

  // 3. 循环
  while (true)
  {
	bool data_ready = false;

	if (use_interrupt)
	{
	  if (xSemaphoreTake(sem_imu_drdy, pdMS_TO_TICKS(20)) == pdTRUE)
	  {
		data_ready = true;
	  }
	}
	else
	{
	  if (imu.isDataReady())
	  {
		data_ready = true;
	  }
	  else
	  {
		vTaskDelay(pdMS_TO_TICKS(1));
	  }
	}

	if (data_ready)
	{
	  imu.getAccel(&data.ax, &data.ay, &data.az);
	  imu.getGyro(&data.gx, &data.gy, &data.gz);
	  data.timestamp_us = esp_timer_get_time();
	  bus.imu.publish(data);

	  // 心跳打印
	  print_count++;
	  if (print_count % 200 == 0)
	  {
		ESP_LOGI(TAG, "IMU Data: Acc(%.2f, %.2f, %.2f)", data.ax, data.ay, data.az);
	  }

	  // --- B. 分频读取磁力计 (50Hz) ---
	  // 200Hz / 4 = 50Hz
	  if (tick_counter % 4 == 0)
	  {
		// 使用 isDataReady 避免 I2C 读空
		if (mag.isDataReady())
		{
			
		  if (mag.readData(&mag_data.x, &mag_data.y, &mag_data.z) == ESP_OK)
		  {
			mag_data.timestamp_us = esp_timer_get_time();
			bus.mag.publish(mag_data);
		  }
		}
	  }

	  tick_counter++;
	}
  }
}

// --- 对外接口实现 ---
void start_sensor_task()
{
  // 可以在这里做一些模块级的预检查
  // 创建任务，绑定到 Core 1
  xTaskCreatePinnedToCore(task_sensor_entry, "Sensor", 4096, NULL, 5, NULL, 1);
  ESP_LOGI(TAG, "Task Started");
}