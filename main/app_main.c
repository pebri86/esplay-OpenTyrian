
#include "opentyr.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void system_reboot_to_firmware(void)
{
	const esp_partition_t *part;
	part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
	// If no factory partition found, use first ota one
	if (part == NULL)
		part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

	if (part != NULL) {
		esp_ota_set_boot_partition(part);
	}
	esp_restart();
}

void tyrianTask(void *pvParameters)
{
//    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
//    spi_lcd_init();

    char *argv[]={"opentyrian", NULL};
    main(1, argv);
    system_reboot_to_firmware();
}


//extern "C"
void app_main(void)
{
	xTaskCreatePinnedToCore(&tyrianTask, "tyrianTask", 34000, NULL, 5, NULL, 0);
}
