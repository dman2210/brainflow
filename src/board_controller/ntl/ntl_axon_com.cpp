#include <chrono>
#include <string.h>
#include <thread>

#include "brainflow_boards.h"
#include "custom_cast.h"
#include "ntl_axon_com.h"
#include "serial.h"
#include "timestamp.h"

#ifdef _WIN32
#include "windows_registry.h"
#endif

#define BOARDID 57
#define START_BYTE 0x0A
#define STOP_BYTE 0x0B
// commands
// i for init (com only)
// p for initialization packet (com only)
// b for begin streaming
// h for halt streaming

NTLAxonComBoard::NTLAxonComBoard (struct BrainFlowInputParams params)
    : Board ((int)(BoardIds::NTL_AXON_COM_BOARD), params)
{
    serial = NULL;
    is_streaming = false;
    keep_alive = false;
    initialized = false;
}

NTLAxonComBoard::~NTLAxonComBoard ()
{
    skip_logs = true;
    release_session ();
}

int NTLAxonComBoard::open_port ()
{
    if (serial->is_port_open ())
    {
        safe_logger (spdlog::level::err, "port {} already open", serial->get_port_name ());
        return (int)BrainFlowExitCodes::PORT_ALREADY_OPEN_ERROR;
    }
    safe_logger (spdlog::level::info, "opening port {}", serial->get_port_name ());
    int res = serial->open_serial_port ();
    if (res < 0)
    {
        safe_logger (spdlog::level::err,
            "Make sure you provided correct port name and have permissions to open it(run with "
            "sudo/admin). Also, close all other apps using this port.");
        return (int)BrainFlowExitCodes::UNABLE_TO_OPEN_PORT_ERROR;
    }
    safe_logger (spdlog::level::trace, "port {} is open", serial->get_port_name ());
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int NTLAxonComBoard::config_board (std::string command, std::string &response)
{
    return sendCommand (command, response);
}

int NTLAxonComBoard::sendCommand (std::string command, std::string &response)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    if (is_streaming)
    {
        safe_logger (spdlog::level::warn,
            "You are changing board params during streaming, it may lead to sync mismatch between "
            "data acquisition thread and device");
        res = send_to_board (command.c_str ());
    }
    else
    {
        // read response if streaming is not running? comment from openbci_serial_board.cpp
        res = send_to_board (command.c_str (), response);
    }

    return res;
}

int NTLAxonComBoard::send_to_board (const char *msg)
{
    int length = (int)strlen (msg);
    safe_logger (spdlog::level::debug, "sending {} to the board", msg);
    int res = serial->send_to_serial_port ((const void *)msg, length);
    if (res != length)
    {
        return (int)BrainFlowExitCodes::BOARD_WRITE_ERROR;
    }

    return (int)BrainFlowExitCodes::STATUS_OK;
}

int NTLAxonComBoard::send_to_board (const char *msg, std::string &response)
{
    int length = (int)strlen (msg);
    safe_logger (spdlog::level::debug, "sending {} to the board", msg);
    int res = serial->send_to_serial_port ((const void *)msg, length);
    if (res != length)
    {
        response = "";
        return (int)BrainFlowExitCodes::BOARD_WRITE_ERROR;
    }
    response = read_serial_response ();

    return (int)BrainFlowExitCodes::STATUS_OK;
}

std::string NTLAxonComBoard::read_serial_response ()
{
    constexpr int max_tmp_size = 4096;
    unsigned char tmp_array[max_tmp_size];
    unsigned char tmp;
    int tmp_id = 0;
    while (serial->read_from_serial_port (&tmp, 1) == 1)
    {
        if (tmp_id < max_tmp_size)
        {
            tmp_array[tmp_id] = tmp;
            tmp_id++;
        }
        else
        {
            serial->flush_buffer ();
            break;
        }
    }
    tmp_id = (tmp_id == max_tmp_size) ? tmp_id - 1 : tmp_id;
    tmp_array[tmp_id] = '\0';

    return std::string ((const char *)tmp_array);
}

int NTLAxonComBoard::set_port_settings ()
{
    int res = serial->set_serial_port_settings (1000, false);
    if (res < 0)
    {
        safe_logger (spdlog::level::err, "Unable to set port settings, res is {}", res);
        return (int)BrainFlowExitCodes::SET_PORT_ERROR;
    }
    safe_logger (spdlog::level::trace, "set port settings");
#ifdef __APPLE__
    int set_latency_res = serial->set_custom_latency (1);
    safe_logger (spdlog::level::info, "set_latency_res is: {}", set_latency_res);
#endif
    return 0;
}

int NTLAxonComBoard::status_check ()
{
    unsigned char buf[1];
    int count = 0;
    int max_empty_seq = 5;
    int num_empty_attempts = 0;
    std::string resp = "";

    for (int i = 0; i < 500; i++)
    {
        int res = serial->read_from_serial_port (buf, 1);
        if (res > 0)
        {
            resp += buf[0];
            num_empty_attempts = 0;
            // board is ready if there are '$$$'
            if (buf[0] == '$')
            {
                count++;
            }
            else
            {
                count = 0;
            }
            if (count == 3)
            {
                return (int)BrainFlowExitCodes::STATUS_OK;
            }
        }
        else
        {
            num_empty_attempts++;
            if (num_empty_attempts > max_empty_seq)
            {
                safe_logger (spdlog::level::err, "board doesnt send welcome characters! Msg: {}",
                    resp.c_str ());
                return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
            }
        }
    }
    return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
}

int NTLAxonComBoard::prepare_session ()
{
    if (initialized)
    {
        safe_logger (spdlog::level::info, "Session already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }
    if (params.serial_port.empty ())
    {
        safe_logger (spdlog::level::err, "serial port is empty");
        return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
    }
#ifdef _WIN32
    LONG res = set_ftdi_latency_in_registry (1, params.serial_port);
    if (res != ERROR_SUCCESS)
    {
        safe_logger (spdlog::level::warn,
            "failed to adjust latency param in ftdi driver automatically, reboot or dongle "
            "reconnection may be needed.");
    }
#endif
    serial = Serial::create (params.serial_port.c_str (), this);
    int port_open = open_port ();
    if (port_open != (int)BrainFlowExitCodes::STATUS_OK)
    {
        send_to_board ("c");
        delete serial;
        serial = NULL;
        return port_open;
    }

    int set_settings = set_port_settings ();
    if (set_settings != (int)BrainFlowExitCodes::STATUS_OK)
    {
        send_to_board ("c");
        delete serial;
        serial = NULL;
        return set_settings;
    }

    initialized = true;

    int send_res = 0;
    send_res = send_to_board ("c");
    std::this_thread::sleep_for (std::chrono::milliseconds (1500));
    send_res = send_to_board ("p");
    std::this_thread::sleep_for (std::chrono::milliseconds (2000));
    if (send_res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        safe_logger (spdlog::level::err, "Failed to send initializtion command");
    }
    std::string packetResponse = "";
    packetResponse = read_serial_response ();
    using timep_t = decltype (std::chrono::steady_clock::now ());
    timep_t start = std::chrono::steady_clock::now ();
    timep_t end = std::chrono::steady_clock::now ();
    uint8_t *responseData = nullptr;
    int lastPrintedSecond = 0;
    while (std::chrono::duration_cast<std::chrono::seconds> (end - start).count () < 10 &&
        packetResponse.size () == 0)
    {
        packetResponse += read_serial_response ();
        int diff = std::chrono::duration_cast<std::chrono::seconds> (end - start).count ();
        if (diff % 3 == 0)
        {
            send_res = send_to_board ("p");
            if (send_res != (int)BrainFlowExitCodes::STATUS_OK)
            {
                safe_logger (spdlog::level::err, "Failed to send initializtion command");
            }
        }
        if (diff > lastPrintedSecond)
        {
            lastPrintedSecond = diff;
            safe_logger (spdlog::level::debug, "Waiting for initialization packet this changes");
            if (packetResponse.size () != 0)
            {
                safe_logger (spdlog::level::debug, "Received packet: {}", packetResponse.c_str ());
            }
        }
        // TODO log response
        end = std::chrono::steady_clock::now ();
    }
    if (packetResponse.size () == 0)
    {
        safe_logger (spdlog::level::err, "Empty initialization packet.");
        res = (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
        send_to_board ("c");
        return res;
    }
    safe_logger (spdlog::level::debug, "Initialization packet: {}", packetResponse.c_str ());
    if (packetResponse.find (START_BYTE) != std::string::npos)
    {
        int eegCount = NTLAxonComBoard::countSubstring (packetResponse, "2") * 8;
        int totalSize = eegCount + 4;
        std::vector<int> eeg_channelsV;
        for (int i = 0; i < eegCount; i++)
        {
            eeg_channelsV.push_back (i);
        }
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["eeg_channels"] =
            eeg_channelsV;
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["eeg_names"] = {};

        boards_struct.brainflow_boards_json["boards"][std::to_string (56)]["default"]["num_rows"] =
            totalSize;
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["status_channel"] =
            totalSize - 3;
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["battery_channel"] =
            totalSize - 2;
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["timestamp_channel"] =
            totalSize - 1;
        boards_struct
            .brainflow_boards_json["boards"][std::to_string (56)]["default"]["marker_channel"] =
            totalSize;

        board_descr["default"]["eeg_channels"] = eeg_channelsV;
        board_descr["default"]["eeg_names"] = {};
        board_descr["default"]["num_rows"] = totalSize;
        board_descr["default"]["status_channel"] = totalSize - 3;
        board_descr["default"]["battery_channel"] = totalSize - 2;
        board_descr["default"]["timestamp_channel"] = totalSize - 1;
        board_descr["default"]["marker_channel"] = totalSize;
    }
    else
    {
        safe_logger (
            spdlog::level::err, "Board config error: Initialization packet improperly formatted.");
        safe_logger (spdlog::level::trace, "read {}", packetResponse.c_str ());
        send_to_board ("c");
        delete serial;
        serial = NULL;
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int NTLAxonComBoard::start_stream (int buffer_size, const char *streamer_params)
{
    if (is_streaming)
    {
        safe_logger (spdlog::level::err, "Streaming thread already running");
        return (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }

    int res = prepare_for_acquisition (buffer_size, streamer_params);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }
    int send_res = send_to_board ("b");
    if (send_res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return send_res;
    }
    std::this_thread::sleep_for (std::chrono::milliseconds (1200));
    send_res = send_to_board ("b");
    if (send_res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return send_res;
    }
    keep_alive = true;
    streaming_thread = std::thread ([this] { this->read_thread (); });
    is_streaming = true;
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int NTLAxonComBoard::stop_stream ()
{
    if (is_streaming)
    {
        keep_alive = false;
        is_streaming = false;
        if (streaming_thread.joinable ())
        {
            streaming_thread.join ();
        }
        return send_to_board ("h");
    }
    else
    {
        return (int)BrainFlowExitCodes::STREAM_THREAD_IS_NOT_RUNNING;
    }
}

int NTLAxonComBoard::release_session ()
{
    if (initialized)
    {
        if (is_streaming)
        {
            stop_stream ();
        }
        free_packages ();
        initialized = false;
    }
    if (serial)
    {
        int send_res = send_to_board ("c");
        serial->close_serial_port ();
        delete serial;
        serial = NULL;
    }
    return (int)BrainFlowExitCodes::STATUS_OK;
}

// returns count of non-overlapping occurrences of 'sub' in 'str'
int NTLAxonComBoard::countSubstring (const std::string &str, const std::string &sub)
{
    if (sub.length () == 0)
        return 0;
    int count = 0;
    for (size_t offset = str.find (sub); offset != std::string::npos;
         offset = str.find (sub, offset + sub.length ()))
    {
        ++count;
    }
    return count;
}

void NTLAxonComBoard::read_thread ()
{
    const int totalRows =
        boards_struct.brainflow_boards_json["boards"][std::to_string (56)]["default"]["num_rows"];
        //todo store extra rows data in variable or something
    const int eegRows = totalRows - 4;
    const int totalIncomingRows = (eegRows * 3) + 4;
    int res;
    unsigned char *b = new unsigned char[totalIncomingRows];
    double *package = new double[totalRows];
    for (int i = 0; i < totalRows; i++)
    {
        package[i] = 0.0;
    }
    std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];

    while (keep_alive)
    {
        res = serial->read_from_serial_port (b, totalIncomingRows);
        if (res != totalIncomingRows)
        {
            if (res < 0)
            {
                safe_logger (spdlog::level::warn, "unable to read {} bytes", totalIncomingRows);
                continue;
            }
        }
        if (b[0] != START_BYTE)
        {
            safe_logger (spdlog::level::warn, "Invalid start byte");
            continue;
        }
        int remaining_bytes = totalIncomingRows;
        int pos = 0;
        while ((remaining_bytes > 0) && (keep_alive))
        {
            res = serial->read_from_serial_port (b + pos, remaining_bytes);
            remaining_bytes -= res;
            pos += res;
        }
        if (!keep_alive)
        {
            break;
        }
        if ((b[totalIncomingRows - 1] != STOP_BYTE))
        {
            safe_logger (spdlog::level::warn, "Wrong end byte {}", b[totalIncomingRows - 1]);
            continue;
        }
        package[board_descr["default"]["package_num_channel"].get<int> ()] = (double)b[2];
        for (int i = 0; i < eeg_channels.size (); i += 3)
        {
            package[i] = ((double)cast_24bit_to_int32 (b + 3 * i));
        }
        // status
        package[eegRows] = ((double)cast_16bit_to_int32 (b + totalIncomingRows - 3));
        // battery level
        package[eegRows + 1] = ((double)b[totalIncomingRows - 1]);
        // timestamp
        package[eegRows + 2] = get_timestamp ();
        push_package (&(package[0]));
    }
}

// void NTLAxonComBoard::print_settings ()
// {
//     safe_logger(spdlog::level::debug, "eeg_channels: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["eeg_channels"].dump());
//     safe_logger(spdlog::level::debug, "eeg_names: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["eeg_names"].dump());
//     safe_logger(spdlog::level::debug, "num_rows: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["num_rows"].dump());
//     safe_logger(spdlog::level::debug, "status_channel: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["status_channel"].dump());
//     safe_logger(spdlog::level::debug, "battery_channel: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["battery_channel"].dump());
//     safe_logger(spdlog::level::debug, "timestamp_channel: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["timestamp_channel"].dump());
//     safe_logger(spdlog::level::debug, "marker_channel: {}", boards_struct.brainflow_boards_json["boards"][std::to_string(56)]["default"]["marker_channel"].dump());

//     safe_logger(spdlog::level::debug, "eeg_channels: {}", board_descr["default"]["eeg_channels"].dump());
//     safe_logger(spdlog::level::debug, "eeg_names: {}", board_descr["default"]["eeg_names"].dump());
//     safe_logger(spdlog::level::debug, "num_rows: {}", board_descr["default"]["num_rows"].dump());
//     safe_logger(spdlog::level::debug, "status_channel: {}", board_descr["default"]["status_channel"].dump());
//     safe_logger(spdlog::level::debug, "battery_channel: {}", board_descr["default"]["battery_channel"].dump());
//     safe_logger(spdlog::level::debug, "timestamp_channel: {}", board_descr["default"]["timestamp_channel"].dump());
//     safe_logger(spdlog::level::debug, "marker_channel: {}", board_descr["default"]["marker_channel"].dump());
// }