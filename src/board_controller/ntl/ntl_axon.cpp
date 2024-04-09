#include <string>

#include "custom_cast.h"
#include "ntl_axon.h"
#include "timestamp.h"


#define START_BYTE 0x0A

static void ntl_adapter_1_on_scan_found (
    simpleble_adapter_t adapter, simpleble_peripheral_t peripheral)
{
    ((NTLAxon *)(board))->adapter_1_on_scan_found (adapter, peripheral);
}

static void ntl_read_notifications (simpleble_uuid_t service, simpleble_uuid_t characteristic,
    uint8_t *data, size_t size, void *board)
{
    ((NTLAxon *)(board))->read_data (service, characteristic, data, size, 0);
}

NTLAxon::NTLAxon (struct BrainFlowInputParams params)
    : BLELibBoard ((int)BoardIds::NTL_AXON_BLE_BOARD, params)
{
    ntlAxonAdapter = NULL;
    ntlAxonPeripheral = NULL;
    initialized = false;
    is_streaming = false;
}

NTLAxon::~NTLAxon ()
{
    skip_logs = true;
    release_session ();
}

int NTLAxon::prepare_session ()
{
    if (initialized)
    {
        safe_logger ("Session already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }
    if (params.timeout < 1)
    {
        params.timeout = 5;
    }
    // count on computer adapters
    size_t num_adapters = simpleble_adapter_get_count ();
    if (num_adapters == 0)
    {
        safe_logger ("No BLE adapters found");
        return (int)BrainFlowExitCodes::UNABLE_TO_OPEN_PORT_ERROR;
    }
    // get a handle on the first adapter on the computer
    ntlAxonAdapter = simpleble_adapter_get_handle (0);
    {
        safe_logger (spdlog::level::err, "Adapter is NULL");
        return (int)BrainFlowExitCodes::UNABLE_TO_OPEN_PORT_ERROR;
    }
    // set up the callback for grabbing the ntl peripheral
    simpleble_adapter_set_callback_on_scan_found (
        ntlAxonAdapter, ::ntl_adapter_1_on_scan_found, (void *)this);

    if (!simpleble_adapter_is_bluetooth_enabled ())
    {
        safe_logger ("Bluetooth appears disabled");
        // dont throw an exception because of this
        // https://github.com/OpenBluetoothToolbox/SimpleBLE/issues/115
    }
    // start scanning
    simpleble_adapter_scan_start (ntlAxonAdapter);
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    std::unique_lock<std::mutex> lk (m);
    auto sec = std::chrono::seconds (1);
    // keep scanning till its found or time runs out
    if (cv.wait_for (lk, params.timeout * sec, [this] { return this->ntlAxonPeripheral != NULL; }))
    {
        safe_logger (spdlog::level::info, "Found NTL Bluetooth Board");
        res = (int)BrainFlowExitCodes::STATUS_OK;
    }
    simpleble_adapter_scan_stop (ntlAxonAdapter);
    // attempt connection
    if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        // brainalive run connection 3 times. they say for safety.
        for (int i = 0; i < 3; i++)
        {
            if (simpleble_peripheral_connect (ntlAxonPeripheral) == SIMPLEBLE_SUCCESS)
            {
                safe_logger (spdlog::level::info, "Connected to NTL Bluetooth Board.");
                res = (int)BrainFlowExitCodes::STATUS_OK;
                break;
            }
            else
            {
                safe_logger (spdlog::level::err, "Failed to connect to NTL Bluetooth Board.");
                res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
            }
#ifdef _WIN32
            Sleep (1000);
#else
            sleep (1);
#endif
        }
    }
    else
    {
        // https://github.com/OpenBluetoothToolbox/SimpleBLE/issues/26#issuecomment-955606799
#ifdef __linux__
        usleep (1000000);
#endif
    }

    bool control_characteristics_found = false;
    // get the bluetooth stuff set up
    if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        // one service
        // two characteristics
        simpleble_service_t service;
        if (simpleble_peripheral_services_get (ntlAxonPeripheral, 0, &service) != SIMPLEBLE_SUCCESS)
        {
            safe_logger (spdlog::level::err, "failed to get service");
            res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
        }
        if (service.characteristic_count != 2)
        {
            safe_logger (spdlog::level::err, "incorrect characteristic count");
            res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
        }
        // TODO
        // check to see which characteristic is which
        // notify
        // set settings

        if ((res == (int)BrainFlowExitCodes::STATUS_OK) && (control_characteristics_found))
        {
            initialized = true;
        }
        else
        {
            release_session ();
        }
        return res;
    }
}