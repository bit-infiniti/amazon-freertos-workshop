/*The config header is always included first.*/

#include "iot_config.h"

/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/* Platform layer includes. */
#include "platform/iot_clock.h"
#include "platform/iot_threads.h"

#include "aws_demo.h"
#include "types/iot_network_types.h"
#include "esp_log.h"
#include "iot_network_manager_private.h"
#include "iot_init.h"
#include "lab_network_manager.h"


/* Semaphore used to wait for a network to be available. */
static IotSemaphore_t networkSemaphore;

/* Variable used to indicate the connected network. */
static uint32_t connectedNetwork = AWSIOT_NETWORK_TYPE_NONE;

static IotNetworkManagerSubscription_t subscription = IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER;

/*-----------------------------------------------------------*/

static uint32_t _getConnectedNetworkForDemo()
{
    uint32_t ret = (AwsIotNetworkManager_GetConnectedNetworks() & configENABLED_NETWORK);

    if ((ret & AWSIOT_NETWORK_TYPE_WIFI) == AWSIOT_NETWORK_TYPE_WIFI)
    {
        ret = AWSIOT_NETWORK_TYPE_WIFI;
    }
    else if ((ret & AWSIOT_NETWORK_TYPE_BLE) == AWSIOT_NETWORK_TYPE_BLE)
    {
        ret = AWSIOT_NETWORK_TYPE_BLE;
    }
    else if ((ret & AWSIOT_NETWORK_TYPE_ETH) == AWSIOT_NETWORK_TYPE_ETH)
    {
        ret = AWSIOT_NETWORK_TYPE_ETH;
    }
    else
    {
        ret = AWSIOT_NETWORK_TYPE_NONE;
    }

    return ret;
}

/*-----------------------------------------------------------*/

static uint32_t _waitForDemoNetworkConnection()
{
    IotSemaphore_Wait(&networkSemaphore);

    return _getConnectedNetworkForDemo();
}

/*-----------------------------------------------------------*/

static void _onNetworkStateChangeCallback(uint32_t network,
                                          AwsIotNetworkState_t state,
                                          void *pContext)
{
    const IotNetworkInterface_t *pNetworkInterface = NULL;
    void *pConnectionParams = NULL, *pCredentials = NULL;
    uint32_t disconnectedNetworks = AWSIOT_NETWORK_TYPE_NONE;

    labContext_t *pLabContext = (labContext_t *)pContext;

    if ((state == eNetworkStateEnabled) && (connectedNetwork == AWSIOT_NETWORK_TYPE_NONE))
    {
        connectedNetwork = network;
        IotSemaphore_Post(&networkSemaphore);

        /* Disable the disconnected networks to save power and reclaim any unused memory. */
        disconnectedNetworks = configENABLED_NETWORKS & (~connectedNetwork);

        if (disconnectedNetworks != AWSIOT_NETWORK_TYPE_NONE)
        {
            AwsIotNetworkManager_DisableNetwork(disconnectedNetworks);
        }

        if (pLabContext->networkConnectedCallback != NULL)
        {
            pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface(network);
            pConnectionParams = AwsIotNetworkManager_GetConnectionParams(network);
            pCredentials = AwsIotNetworkManager_GetCredentials(network),

            pLabContext->networkConnectedCallback(true,
                                                   clientcredentialIOT_THING_NAME,
                                                   pConnectionParams,
                                                   pCredentials,
                                                   pNetworkInterface);
        }
    }
    else if ((state == eNetworkStateDisabled) && (connectedNetwork == network))
    {
        if (pLabContext->networkDisconnectedCallback != NULL)
        {
            pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface(network);
            pLabContext->networkDisconnectedCallback(pNetworkInterface);
        }

        /* Re-enable all the networks for the demo for reconnection. */
        disconnectedNetworks = configENABLED_NETWORKS & (~connectedNetwork);

        if (disconnectedNetworks != AWSIOT_NETWORK_TYPE_NONE)
        {
            AwsIotNetworkManager_EnableNetwork(disconnectedNetworks);
        }

        connectedNetwork = _getConnectedNetworkForDemo(pLabContext);

        if (connectedNetwork != AWSIOT_NETWORK_TYPE_NONE)
        {
            if (pLabContext->networkConnectedCallback != NULL)
            {
                pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface(connectedNetwork);
                pConnectionParams = AwsIotNetworkManager_GetConnectionParams(connectedNetwork);
                pCredentials = AwsIotNetworkManager_GetCredentials(connectedNetwork);

                pLabContext->networkConnectedCallback(true,
                                                       clientcredentialIOT_THING_NAME,
                                                       pConnectionParams,
                                                       pCredentials,
                                                       pNetworkInterface);
            }
        }
    }
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the common libraries, Mqtt library and network manager.
 *
 * @return `EXIT_SUCCESS` if all libraries were successfully initialized;
 * `EXIT_FAILURE` otherwise.
 */
int network_initialize(labContext_t *pContext)
{
    int status = EXIT_SUCCESS;
    bool commonLibrariesInitialized = false;
    bool semaphoreCreated = false;

    /* Initialize common libraries required by network manager and demo. */
    if (IotSdk_Init() == true)
    {
        commonLibrariesInitialized = true;
    }
    else
    {
        IotLogInfo("Failed to initialize the common library.");
        status = EXIT_FAILURE;
    }

    if (status == EXIT_SUCCESS)
    {
        if (AwsIotNetworkManager_Init() != pdTRUE)
        {
            IotLogError("Failed to initialize network manager library.");
            status = EXIT_FAILURE;
        }
    }

    if (status == EXIT_SUCCESS)
    {
        /* Create semaphore to signal that a network is available for the demo. */
        if (IotSemaphore_Create(&networkSemaphore, 0, 1) != true)
        {
            IotLogError("Failed to create semaphore to wait for a network connection.");
            status = EXIT_FAILURE;
        }
        else
        {
            semaphoreCreated = true;
        }
    }

    if (status == EXIT_SUCCESS)
    {
        /* Subscribe for network state change from Network Manager. */
        if (AwsIotNetworkManager_SubscribeForStateChange(configENABLED_NETWORKS,
                                                         _onNetworkStateChangeCallback,
                                                         pContext,
                                                         &subscription) != pdTRUE)
        {
            IotLogError("Failed to subscribe network state change callback.");
            status = EXIT_FAILURE;
        }
    }

    /* Initialize all the  networks configured for the device. */
    if (status == EXIT_SUCCESS)
    {
        if (AwsIotNetworkManager_EnableNetwork(configENABLED_NETWORKS) != configENABLED_NETWORKS)
        {
            IotLogError("Failed to intialize all the networks configured for the device.");
            status = EXIT_FAILURE;
        }
    }

    if (status == EXIT_SUCCESS)
    {
        /* Wait for network configured for the demo to be initialized. */
        connectedNetwork = _getConnectedNetworkForDemo();

        if (connectedNetwork == AWSIOT_NETWORK_TYPE_NONE)
        {
            /* Network not yet initialized. Block for a network to be intialized. */
            IotLogInfo("No networks connected for the demo. Waiting for a network connection. ");
            connectedNetwork = _waitForDemoNetworkConnection();
        }
    }

    if (status == EXIT_FAILURE)
    {
        if (semaphoreCreated == true)
        {
            IotSemaphore_Destroy(&networkSemaphore);
        }

        if (commonLibrariesInitialized == true)
        {
            IotSdk_Cleanup();
        }
    }

    return status;
}