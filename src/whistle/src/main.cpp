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

void handler(int signal) {

    printf("... handling signal ...\n");

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

long iteration = 0;

void callback(nbsound::Handler * cap, void * buffer, nbsound::parameter_t * params) {
    printf("callback %ld\n", iteration);



    ++iteration;
}

int main(int argc, const char ** argv) {
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

    capture->init();
    std::cout << capture->print() << std::endl;

    printf("main: period is %lf seconds\n", nbsound::PERIOD(params));
    printf("main: sample freq is %lf seconds\n", nbsound::FREQUENCY(params));

    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    std::string unused;
    std::getline(std::cin, unused);

    pthread_t thread;
    capture->start_new_thread(thread, NULL);
    printf("main: capture created...\n");

    while (capture->is_active()) {
        usleep(100);
    }
    printf("main: capture waited...\n");

    delete capture;
    nblog::log_main_destroy();

    printf("main: done.\n");

    std::string mode(argv[1]);

    if (mode == captureMode) {
        control::control_init();
        nblog::log_main_init();

        nbsound::parameter_t params = {nbsound::NBS_S16_LE, 2, 32768, 48000};
        capture = new nbsound::Capture(callback, params);

        capture->init();
        std::cout << capture->print() << std::endl;

        printf("main: period is %lf seconds\n", nbsound::PERIOD(params));
        printf("main: sample freq is %lf seconds\n", nbsound::FREQUENCY(params));

        signal(SIGINT, handler);
        signal(SIGTERM, handler);

        std::string unused;
        std::getline(std::cin, unused);

        pthread_t thread;
        capture->start_new_thread(thread, NULL);
        printf("main: capture created...\n");

        while (capture->is_active()) {
            usleep(100);
        }
        printf("main: capture waited...\n");
        
        delete capture;
        nblog::log_main_destroy();
        
        printf("main: done.\n");
    } else if (mode == transmitMode) {

        control::control_init();
        nblog::log_main_init();

        nbsound::parameter_t params = {nbsound::NBS_S16_LE, 2, 32768, 48000};
        capture = new nbsound::Capture(callback, params);

        capture->init();
        std::cout << capture->print() << std::endl;

        printf("main: period is %lf seconds\n", nbsound::PERIOD(params));
        printf("main: sample freq is %lf seconds\n", nbsound::FREQUENCY(params));

        signal(SIGINT, handler);
        signal(SIGTERM, handler);

        std::string unused;
        std::getline(std::cin, unused);

        pthread_t thread;
        capture->start_new_thread(thread, NULL);
        printf("main: capture created...\n");

        while (capture->is_active()) {
            usleep(100);
        }
        printf("main: capture waited...\n");

        delete capture;
        nblog::log_main_destroy();
        
        printf("main: done.\n");

    } else {
        printf("unknown mode: %s\n",
               argv[1]);
        return 1;
    }

    return 0;
}
