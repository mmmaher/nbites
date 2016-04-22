#include <iostream>
#include <signal.h>

#include "Sound.h"
#include "Transform.h"

#include "logcopy/logging.h"
#include "logcopy/control.h"

nbsound::Capture * capture = NULL;


#define GETFORMAT(v) (##v)

void handler(int signal) {
    printf("... handling signal ...\n");
    if (capture) {
        if (!capture->stop()) {
            exit(1);
        }
    }
}

int iteration = 0;
using nblog::SExpr;
void callback(nbsound::Handler * cap, void * buffer, nbsound::parameter_t * params) {
    printf("callback %i\n", iteration);
    
    SExpr ampSExpr("sound", "stand-alone", clock(), iteration, 0);
    ampSExpr.append(SExpr("channels", 2));
    ampSExpr.append(SExpr("format", "NBS_S16_LE") );
    ampSExpr.append(SExpr("frames", params->frames));
    ampSExpr.append(SExpr("rate", params->rate));
    
    std::vector<SExpr> ampContents = {ampSExpr};
    
    nblog::NBLog(NBL_IMAGE_BUFFER, "stand-alone-main", ampContents, buffer,
                 nbsound::APP_BUFFER_SIZE(*params));
    
    ++iteration;
}

const std::string captureMode("capture");
const std::string transmitMode("transmit");

int main(int argc, const char ** argv) {
    printf("sound stand-alone\n");

    if (argc < 2) {
        printf("stand-alone requires mode [capture, transmit]\n");
        return 0;
    }

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
