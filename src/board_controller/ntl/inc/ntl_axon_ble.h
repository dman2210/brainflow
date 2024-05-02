#include <condition_variable>
#include <mutex>
#include <thread>

#include "ble_lib_board.h"
#include "board.h"
#include "board_controller.h"

class NTLAxonBLEBoard : public BLELibBoard
{
public:
    NTLAxonBLEBoard (struct BrainFlowInputParams params);
    ~NTLAxonBLEBoard ();

    int prepare_session ();
    int start_stream (int buffer_size, const char *streamer_params);
    int stop_stream ();
    int release_session ();
    int sendCommand (std::string config, std::string &response);
    int sendCommand (std::string config);

    void adapter_1_on_scan_found (simpleble_adapter_t adapter, simpleble_peripheral_t peripheral);
    void read_data (simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data,
        size_t size, int channel_num);

    int config_board (std::string commandString);
    int config_board (std::string commandString, std::string &response);

protected:
    volatile simpleble_adapter_t ntlAxonAdapter;
    volatile simpleble_peripheral_t ntlAxonPeripheral;
    static void ntlAxon_read_notifications (simpleble_uuid_t service,
        simpleble_uuid_t characteristic, uint8_t *data, size_t size, void *board);
    bool initialized;
    bool is_streaming;
    std::mutex m;
    std::condition_variable cv;
    std::pair<simpleble_uuid_t, simpleble_uuid_t> notified_characteristics;
    std::pair<simpleble_uuid_t, simpleble_uuid_t> write_characteristics;
};