/*
 * can_device_manager.h
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_DEVICE_MANAGER_H_
#define APPLICATION_CAN_CAN_DEVICE_MANAGER_H_

#include "can_device.h"

namespace CANDeviceManager
{
	void init(CAN_HandleTypeDef* hcan, uint8_t slave_id, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	void addDevice(CANDevice* device);
	void sendMessages();
	void tick(int cycle /* ms */);
	void Receive_IT();
	bool isTimeout();
	void userSendMessages();
	void userReceiveMessagesCallback(uint8_t slave_id, uint8_t device_id, uint8_t message_id, uint32_t DLC, uint8_t* data);
}

#endif /* APPLICATION_CAN_CAN_DEVICE_MANAGER_H_ */
