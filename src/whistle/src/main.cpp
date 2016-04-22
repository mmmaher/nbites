#include <iostream>
#include <signal.h>

#include "Sound.h"
#include "Transform.h"
#include "nblogio.h"
#include "utilities.hpp"

const char * WHISTLE_LOG_PAH = "/home/nao/nbites/log/whistle";
const int WHISTLE_PORT = 30000;

using namespace nblog;

nbsound::Capture * capture = NULL;
nblog::io::server_socket_t server = 0;
nblog::io::client_socket_t client = 0;

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

void callback(nbsound::Handler * cap, void * buffer, nbsound::parameter_t * params) {
    printf("callback %ld\n", iteration);



    ++iteration;
}

int main(int argc, const char ** argv) {

    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    printf("...whistle...\nfreopen()....\n");
    freopen(WHISTLE_LOG_PAH, "w", stdout);

    NBL_INFO("whistle::main() log file re-opened...");

    io::ioret ret = nblog::io::server_socket(server, WHISTLE_PORT, 1);
    if (ret) {
        NBL_ERROR("could not create server socket!\n");
        whistleExitEnd();
    }

    nbsound::parameter_t params = {nbsound::NBS_S16_LE, 2, 32768, 48000};
    capture = new nbsound::Capture(callback, params);

    printf("main: period is %lf seconds\n", nbsound::PERIOD(params));
    printf("main: sample freq is %lf seconds\n", nbsound::FREQUENCY(params));

    capture->init();
    std::cout << capture->print() << std::endl;
    pthread_t capture_thread;

    printf("main: capture created...\n");
    capture->start_new_thread(capture_thread, NULL);

    while(capture->is_active()) {
        
    }

    whistleExit();
    return 0;
}
