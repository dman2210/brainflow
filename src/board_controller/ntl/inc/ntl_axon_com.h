#include "board.h"
#include "board_controller.h"
#include "serial.h"

class NTLAxonComBoard : public Board
{
protected:
    volatile bool keep_alive;
    bool initialized;
    bool is_streaming;
    std::thread streaming_thread;

    Serial *serial;

    int open_port ();
    int status_check ();
    int set_port_settings ();
    void read_thread ();
    int send_to_board (const char *msg);
    int send_to_board (const char *msg, std::string &response);
    std::string read_serial_response ();


public:
int config_board (std::string command, std::string &response);
    NTLAxonComBoard (struct BrainFlowInputParams params);
    NTLAxonComBoard ();
    int prepare_session ();
    int start_stream (int buffer_size, const char *streamer_params);
    int stop_stream ();
    int release_session ();
    int sendCommand (std::string config, std::string &response);
};