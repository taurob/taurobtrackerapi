/**************************************************************************//**
 *
 * @file libtaurob_flipper.h
 * @author taurob GmbH
 * @brief Library for communication with taurob flipper.
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
 
#ifndef LIBTAUROB_FLIPPER_H_
#define LIBTAUROB_FLIPPER_H_

#include <stdio.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

#include "flipper_frames.h"
#include "libtaurob_tools/SocketUDPBinary.h"

struct FlipperCommand
{
    /// Angle in radians.
    double angle;

    /// If set, the ECU will be reset.
    bool flag_reset;

    FlipperCommand()
        : angle(0)
        , flag_reset(false)
    {
    }
};

struct FlipperStatus
{
    uint8_t flags;
    uint32_t seconds;
    uint16_t microseconds;

    /// Angle in radians.
    double angle;

    /// Motor speed in rpm.
    double speed;

    /// Motor current in Amp.
    double current;

    FlipperStatus()
        : flags(0)
        , seconds(0)
        , microseconds(0)
        , angle(0)
        , speed(0)
        , current(0)
    {
    }
};

class Flipper: IUdpReceiverBinary
{
    public:
        Flipper(std::string host_ip, int host_port);
        ~Flipper();
		
        /** implementation for IUdpReceiverBinary */
        void On_data_received(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port);
		
        /** Starts communicating with the arm joint via UDP */
        void Run();

        /** Stops communicating with the arm joint via UDP  */
        void Stop();

        /** Control the flipper */
        void Set(FlipperCommand command);

        /** Returns the last set command frame */
        FlipperCommand GetCommandFrame();

        /** Get the flipper data */
        FlipperStatus Get();

        bool Is_ok(void);

        /** Safety feature: has to be called every 250 ms or the flipper will stop */
        void Feed_watchdog();

        void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    private:
        static const int SENDING_THREAD_PERIOD_MS = 20;

        static const int WATCHDOG_MAX_TIME_MS = 250;
        static const int WATCHDOG_CHECKER_THREAD_PERIOD_MS = 5;

        std::string host_ip;
        int host_port;
        SocketUDPBinary* ecu_socket;
        bool sending_thread_running;

        boost::mutex current_set_values_locker;
        FlipperCommandFrame current_set_values;
        FlipperCommand last_set_command_frame;

        boost::mutex current_status_locker;
        FlipperStatus current_status;

        boost::thread sending_thread;
	FlipperFrames flipperFrames;

        void Sending_thread();

        boost::posix_time::ptime latest_watchdog_time;
        boost::thread watchdog_checker_thread;
        bool watchdog_checker_thread_running;
        bool watchdog_has_expired;

        /** Checks if the watchdog has been fed well */
        void Watchdog_checker(void);
};

#endif
