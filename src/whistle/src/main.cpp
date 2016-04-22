#include <iostream>
#include <signal.h>

#include "Sound.h"
#include "Transform.h"
#include "nblogio.h"
#include "utilities.hpp"

const char * WHISTLE_LOG_PATH = "/home/nao/nbites/log/whistle";
const int WHISTLE_PORT = 30005;

using namespace nblog;

nbsound::Capture * capture = NULL;
nbsound::Transform * transform = NULL;

nblog::io::server_socket_t server = 0;
nblog::io::client_socket_t client = 0;

uint8_t WHISTLE_HEARD = false;

FILE * logFile;

void whistleExitEnd() {
    fflush(stdout);
    fclose(stdout);
    exit(0);
}

void whistleExit() {
    if (capture) {
        NBL_WARN("stopping capture...")
        if (!capture->stop()) {
        }
    }

    if (client) {
        NBL_WARN("close(client)...")
        close(client);
    }

    if (server) {
        NBL_WARN("close(server)...")
        close(server);
    }

    whistleExitEnd();
}

void handler(int signal) {
    printf("... handling signal ...\n");
    whistleExit();
}

long iteration = 0;

double sum(int start, int end) {
    NBL_ASSERT_GT(transform->get_freq_len(), end);
    double total = 0;
    for (int i = start; i < end; ++i) {
        total += transform->outputmag[i] * transform->outputmag[i];
    }

    return total;
}

const double WHISTLE_THRESHOLD = 5000000;

void callback(nbsound::Handler * cap, void * buffer, nbsound::parameter_t * params) {
//    printf("callback %ld\n", iteration);

    if (buffer && transform) {
        for (int i = 0; i < params->channels; ++i) {
//            printf("\ttransform %d\n", i);
            transform->transform(buffer, i);

            double summed = sum(1600, 1800);
//            NBL_PRINT("summed=\t%lf\n", summed);
            if (summed > WHISTLE_THRESHOLD) {
                NBL_WARN("WHISTLE: %lf\n", summed);
            }
        }
    }

    ++iteration;
}

int main(int argc, const char ** argv) {

    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    printf("...whistle...\nfreopen()....\n");
//    freopen(WHISTLE_LOG_PATH, "w", stdout);

    NBL_INFO("whistle::main() log file re-opened...");

    io::ioret ret = nblog::io::server_socket(server, WHISTLE_PORT, 1);
    if (ret) {
        NBL_ERROR("could not create server socket!\n");
        whistleExitEnd();
    }

    NBL_WHATIS(server);	

    nbsound::parameter_t params = {nbsound::NBS_S16_LE, 2, 32768, 48000};
    transform = new nbsound::Transform(params);
    capture = new nbsound::Capture(callback, params);

    printf("main: period is %lf seconds\n", nbsound::PERIOD(params));
    printf("main: sample freq is %lf seconds\n", nbsound::FREQUENCY(params));

    capture->init();
    std::cout << capture->print() << std::endl;
    pthread_t capture_thread;

    printf("main: capture created...\n");
    capture->start_new_thread(capture_thread, NULL);

    while(capture->is_active()) {
        io::ioret ret = io::poll_accept(server, client);
        if (ret) {
            NBL_ERROR("io::poll_accept() got error!");
            whistleExit();
        }

        io::config_socket(client, (io::sock_opt_mask) 0);
        io::send_exact(client, 1, &WHISTLE_HEARD, io::IO_MAX_DELAY());

        close(client);
    }

    whistleExit();
    return 0;
}
