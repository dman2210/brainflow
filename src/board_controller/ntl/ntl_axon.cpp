#include <string>

#include "custom_cast.h"
#include "ntl_axon.h"
#include "timestamp.h"


#define START_BYTE 0x0A
#define write_characteristic_uuid "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define notify_characteristic_uuid "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

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
        if (strcmp (service.characteristics[0].uuid.value, write_characteristic_uuid))
        {
            write_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                service.uuid, service.characteristics[0].uuid);
            control_characteristics_found = true;
            safe_logger (spdlog::level::info, "found control characteristic");
            if (strcmp (service.characteristics[1].uuid.value, notify_characteristic_uuid) == 0)
            {
                if (simpleble_peripheral_notify (ntlAxonPeripheral, service.uuid,
                        service.characteristics[1].uuid, ::ntlAxon_read_notifications,
                        (void *)this) == SIMPLEBLE_SUCCESS)
                {
                    notified_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                        service.uuid, service.characteristics[1].uuid);
                }
                else
                {
                    safe_logger (spdlog::level::err, "failed to notify characteristic");
                    res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
                }
            }
        }
        else if (strcmp (service.characteristics[1].uuid.value, write_characteristic_uuid))
        {
            write_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                service.uuid, service.characteristics[1].uuid);
            control_characteristics_found = true;
            safe_logger (spdlog::level::info, "found control characteristic");
            if (strcmp (service.characteristics[0].uuid.value, notify_characteristic_uuid) == 0)
            {
                if (simpleble_peripheral_notify (ntlAxonPeripheral, service.uuid,
                        service.characteristics[0].uuid, ::ntlAxon_read_notifications,
                        (void *)this) == SIMPLEBLE_SUCCESS)
                {
                    notified_characteristics = std::pair<simpleble_uuid_t, simpleble_uuid_t> (
                        service.uuid, service.characteristics[0].uuid);
                }
            }
            else
            {
                safe_logger (spdlog::level::err, "failed to notify characteristic");
                res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
            }
        }
        else
        {
            safe_logger (spdlog::level::err, "failed to find control characteristic");
            res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
        }
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

int NTLAxon::start_stream (int buffer_size, const char *streamer_params)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    int res = prepare_for_acquisition (buffer_size, streamer_params);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        // TODO get command for start
        res = send_command ("start");
    }
    if (res == (int)BrainFlowExitCodes::STATUS_OK)
    {
        safe_logger (spdlog::level::debug, "Start command Send 0x8000000d");
        is_streaming = true;
    }

    return res;
}

int NTLAxon::stop_stream ()
{
    if (ntlAxonPeripheral == NULL)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    if (is_streaming)
    {
        res = send_command ("stop");
    }
    else
    {
        res = (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }
    is_streaming = false;
    return res;
}

int NTLAxon::release_session ()
{
    if (initialized)
    {
        // looped previously to prevent a crash
        for (int i = 0; i < 2; i++)
        {
            stop_stream ();
            // need to wait for notifications to stop triggered before unsubscribing, otherwise
            // macos fails inside simpleble with timeout
#ifdef _WIN32
            Sleep (2000);
#else
            sleep (2);
#endif
            if (simpleble_peripheral_unsubscribe (ntlAxonPeripheral, notified_characteristics.first,
                    notified_characteristics.second) != SIMPLEBLE_SUCCESS)
            {
                safe_logger (spdlog::level::err, "failed to unsubscribe for {} {}",
                    notified_characteristics.first.value, notified_characteristics.second.value);
            }
            else
            {
                break;
            }
        }
        free_packages ();
        initialized = false;
    }
    if (ntlAxonPeripheral != NULL)
    {
        bool is_connected = false;
        if (simpleble_peripheral_is_connected (ntlAxonPeripheral, &is_connected) ==
            SIMPLEBLE_SUCCESS)
        {
            if (is_connected)
            {
                simpleble_peripheral_disconnect (ntlAxonPeripheral);
            }
        }
        simpleble_peripheral_release_handle (ntlAxonPeripheral);
        ntlAxonPeripheral = NULL;
    }
    if (ntlAxonAdapter != NULL)
    {
        simpleble_adapter_release_handle (ntlAxonAdapter);
        ntlAxonAdapter = NULL;
    }
    return (int)BrainFlowExitCodes::STATUS_OK;
}

static void ntlAxon_read_notifications (simpleble_uuid_t service, simpleble_uuid_t characteristic,
    uint8_t *data, size_t size, void *board)
{
    ((NTLAxon *)(board))->read_data (service, characteristic, data, size, 0);
}