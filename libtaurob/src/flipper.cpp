/**************************************************************************//**
 *
 * @file flipper_frames.cpp
 * @author taurob GmbH
 * @brief Library for communication with taurob flipper
 *
 *
 *  Copyright (c) 2017 taurob GmbH. All rights reserved.
 *  Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/

#include "libtaurob_flipper/libtaurob_flipper.h"

#include <string>
#include <stdlib.h>

using namespace boost;

Flipper::Flipper(std::string host_ip, int host_port)
    : host_ip(host_ip) 
    , host_port(host_port)
    , sending_thread_running(false)
    , latest_watchdog_time(boost::posix_time::time_from_string("1970-01-01 00:00:00.000"))
    , watchdog_checker_thread_running(false)
    , watchdog_has_expired(false)
{ 
    watchdog_checker_thread_running = true;
    watchdog_checker_thread = boost::thread(boost::bind(&Flipper::Watchdog_checker, this));
}

Flipper::~Flipper()
{
    Stop();
}

void Flipper::On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)
{
    FlipperStatusFrame status_frame;

    if (!flipperFrames.Deserialize_status_frame(data, len, &status_frame))
    {
        return;
    }

    current_status_locker.lock();
    current_status.angle = ((double)status_frame.angle) / 1000;
    current_status.speed = status_frame.speed;
    current_status.current = ((double)status_frame.current) / 1000;
    current_status_locker.unlock();
}

void Flipper::Run()
{
    // bind to same port as host_port, since ecu will send responses there
    ecu_socket = new SocketUDPBinary(host_ip, host_port, host_port, FLIPPER_STATUS_FRAME_LENGTH);
    ecu_socket->start_listen(this);

    sending_thread_running = true;
    sending_thread = boost::thread(boost::bind(&Flipper::Sending_thread, this));
}

void Flipper::Stop()
{
    sending_thread_running = false;

    sending_thread.join();

    ecu_socket->stop_listen();
    delete ecu_socket;
    ecu_socket = 0;
}

void Flipper::Set(FlipperCommand command)
{
    current_status_locker.lock();
    bool flag_activate = !watchdog_has_expired;
    current_status_locker.unlock();
    bool flag_reset = command.flag_reset;
    if (flag_reset)
    {
        printf("sending reset flat to ECU");
    }

    const int16_t max_angle = 3142;
    const int16_t min_angle = -3141;

    int16_t angle = 1000 * command.angle;
    if (angle > max_angle)
    {
        angle = max_angle;
    }
    if (angle < min_angle)
    {
        angle = min_angle;
    }

    current_set_values_locker.lock();
    last_set_command_frame = command;
    last_set_command_frame.flag_reset = false;

    current_set_values.angle = angle;

    current_set_values.flags =
        (flag_activate ? 1 : 0) |
        (flag_reset    ? 1 : 0) << 1;

    current_set_values_locker.unlock();
}

FlipperCommand Flipper::GetCommandFrame()
{
    current_set_values_locker.lock();
    FlipperCommand fc = last_set_command_frame;
    current_set_values_locker.unlock();

    return fc;
}

FlipperStatus Flipper::Get()
{
    current_status_locker.lock();
    FlipperStatus fs = current_status;
    current_status_locker.unlock();
    return fs;
}

void Flipper::Sending_thread()
{
    unsigned char sendbuffer[FLIPPER_COMMAND_FRAME_LENGTH];

    printf("Flipper::Sending_thread\n");

    while (sending_thread_running)
    {
        current_set_values_locker.lock();
        flipperFrames.Serialize_command_frame(&current_set_values, sendbuffer, sizeof(sendbuffer));
        current_set_values_locker.unlock();

        ecu_socket->send(sendbuffer, sizeof(sendbuffer));

        boost::this_thread::sleep(boost::posix_time::milliseconds(SENDING_THREAD_PERIOD_MS));
    }

    printf("Flipper::Sending_thread: done\n");
}

void Flipper::Feed_watchdog()
{
    current_status_locker.lock();
    latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
    watchdog_has_expired = false;
    current_status_locker.unlock();
}

void Flipper::Watchdog_checker()
{
    while (watchdog_checker_thread_running)
    {
        current_status_locker.lock();

        auto ms = (boost::posix_time::microsec_clock::local_time() - latest_watchdog_time).total_milliseconds();
        if ((ms > WATCHDOG_MAX_TIME_MS) && (!(watchdog_has_expired)))
        {
            printf("Flipper: watchdog expired");
            watchdog_has_expired = true;
        }

        current_status_locker.unlock();

        boost::this_thread::sleep(boost::posix_time::milliseconds(WATCHDOG_CHECKER_THREAD_PERIOD_MS));
    }
}

bool Flipper::Is_ok(void)
{
    current_status_locker.lock();
    bool ok =  (!watchdog_has_expired);
    current_status_locker.unlock();

    return ok;
}

void Flipper::produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (ecu_socket->connection_timeout())
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "disconnected");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
}
