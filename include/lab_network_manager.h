
#ifndef _LAB_NETWORK_MANAGER_H_
#define _LAB_NETWORK_MANAGER_H_

#include "aws_demo.h"

#include "types/iot_network_types.h"

typedef struct labContext
{
    networkConnectedCallback_t networkConnectedCallback;
    networkDisconnectedCallback_t networkDisconnectedCallback;
} labContext_t;

#define configENABLED_NETWORK   ( AWSIOT_NETWORK_TYPE_WIFI | AWSIOT_NETWORK_TYPE_BLE )

int network_initialize(labContext_t *pContext);

#endif