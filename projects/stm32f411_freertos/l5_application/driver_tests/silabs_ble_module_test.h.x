#pragma once

void silabs_ble_module_test__create_tasks(unsigned long priority);
void silabs_ble_module_test__task(void *parameter);
void silabs_ble_module_test__rx_buffer_filler_task(void *parameter);