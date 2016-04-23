#include "GameStateModule.h"

#include <iostream>

#include "RoboCupGameControlData.h"
#include "Common.h"

#include "exactio.h"
#include <netinet/in.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <err.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/time.h>

namespace man{
    namespace gamestate{

        GameStateModule::GameStateModule(int team, int player) :
        portals::Module(),
        gameStateOutput(base()),
        gcResponseOutput(base()),
        team_number(team),
        player_number(player),
        last_button(false),
        last_initial(false),
        last_team(false),
        last_kickoff(false),
        response_status(GAMECONTROLLER_RETURN_MSG_ALIVE),
        keep_time(false),
        start_time(0),
        heard_whistle(false)
        {
            reset();
        }

        GameStateModule::~GameStateModule()
        {
        }

        void GameStateModule::run_()
        {
            latchInputs();
            update();

            portals::Message<messages::GameState> output(&latest_data);
            gameStateOutput.setMessage(output);

            portals::Message<messages::GCResponse> response(0);
            response.get()->set_status(response_status);
            gcResponseOutput.setMessage(response);
        }

        void GameStateModule::latchInputs()
        {
            commInput         .latch();
            buttonPressInput  .latch();
            initialStateInput .latch();
            switchTeamInput   .latch();
            switchKickOffInput.latch();
        }

        void GameStateModule::update()
        {
            int last, next;
            last = latest_data.state();
            // Check comm input last so we can reset button toggles.
            if (buttonPressInput.message().toggle() != last_button)
            {
                last_button = !last_button;
                if (!commInput.message().have_remote_gc()
                    || latest_data.state() == STATE_PLAYING)
                {
                    advanceState();
                }
            }
            if (initialStateInput.message().toggle() != last_initial)
            {
                last_initial = !last_initial;
                if (!commInput.message().have_remote_gc())
                    reset();
            }
            if (switchTeamInput.message().toggle() != last_team)
            {
                last_team = !last_team;
                if (!commInput.message().have_remote_gc()
                    && latest_data.state() == STATE_INITIAL)
                    switchTeam();
            }
            if (switchKickOffInput.message().toggle() != last_kickoff)
            {
                last_kickoff = !last_kickoff;
                if (!commInput.message().have_remote_gc()
                    && latest_data.state() == STATE_INITIAL)
                    switchKickOff();
            }
            if (commInput.message().have_remote_gc())
            {
                latest_data = commInput.message();
                if (latest_data.state() != STATE_PLAYING)
                {
                    keep_time = false;
                    start_time = 0;
                }
                else
                {
                    keep_time = true;
                    if (!start_time)
                    {
                        start_time = realtime_micro_time();
                    }
                }
                // Did GC get our message yet??
                for (int i = 0; i < latest_data.team_size(); ++i)
                {
                    messages::TeamInfo* team = latest_data.mutable_team(i);
                    if (team->team_number() == team_number)
                    {
                        messages::RobotInfo* player = team->mutable_player(player_number-1);
                        if (response_status == GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)
                        {
                            if(player->penalty())
                            {
                                response_status = GAMECONTROLLER_RETURN_MSG_ALIVE;
                            }
                        }
                        else if (response_status == GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)
                        {
                            if(!player->penalty())
                            {
                                response_status == GAMECONTROLLER_RETURN_MSG_ALIVE;
                            }
                        }
                    }
                }
            }
            if (keep_time && commInput.message().have_remote_gc())
            {
                long long diff_time = realtime_micro_time() - start_time;
                latest_data.set_secs_remaining(600 -
                                               static_cast<unsigned int>(diff_time/MICROS_PER_SECOND));
                //TODO keep track of penalty times
            }

            next = latest_data.state();
            whistleHandler(last, next);
            latest_data.set_state(next);
        }

        void GameStateModule::advanceState()
        {
            switch (latest_data.state())
            {
                case STATE_INITIAL:
                    latest_data.set_state(STATE_READY);
                    break;
                case STATE_READY:
                    latest_data.set_state(STATE_SET);
                    break;
                case STATE_SET:
                    latest_data.set_state(STATE_PLAYING);
                    keep_time = true;
                    start_time = realtime_micro_time();
                    break;
                case STATE_PLAYING:
                    manual_penalize();
                    break;
            }
        }

        void GameStateModule::manual_penalize()
        {
            for (int i = 0; i < latest_data.team_size(); ++i)
            {
                messages::TeamInfo* team = latest_data.mutable_team(i);
                if (team->team_number() == team_number)
                {
                    messages::RobotInfo* player = team->mutable_player(player_number-1);
                    if (player->penalty())
                    {
                        response_status = GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;
                        if (!commInput.message().have_remote_gc())
                            player->set_penalty(PENALTY_NONE);
                    }
                    else
                    {
                        response_status = GAMECONTROLLER_RETURN_MSG_MAN_PENALISE;
                        if (!commInput.message().have_remote_gc())
                            player->set_penalty(PENALTY_MANUAL);
                    }
                    break;
                }
            }
        }

        void GameStateModule::reset()
        {
            keep_time = false;
            latest_data.Clear();

            latest_data.set_state(STATE_INITIAL);
            latest_data.set_kick_off_team(team_number);
            latest_data.set_secondary_state(STATE2_NORMAL);
            latest_data.set_drop_in_team(team_number);

            latest_data.set_have_remote_gc(false);

            messages::TeamInfo* myTeam = latest_data.add_team();
            myTeam->set_team_number(team_number);
            myTeam->set_team_color(TEAM_BLUE);
            myTeam->set_goal_color(1); // TODO eliminate goal color
            for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
            {
                messages::RobotInfo* player = myTeam->add_player();
                player->set_penalty(PENALTY_NONE);
            }

            messages::TeamInfo* them = latest_data.add_team();
            them->set_team_number(0);
            them->set_team_color(TEAM_RED);
            them->set_goal_color(1); // TODO eliminate goal color
            for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
            {
                messages::RobotInfo* player = them->add_player();
                player->set_penalty(PENALTY_NONE);
            }

            heard_whistle = false;
        }
        void GameStateModule::switchTeam()
        {
            // Switch the order of the TeamInfo messages' indicies
            // Set the new blue team to blue, and the new red team to red
            latest_data.mutable_team()->SwapElements(0,1);
            latest_data.mutable_team(TEAM_BLUE)->set_team_color(TEAM_BLUE);
            latest_data.mutable_team(TEAM_RED)->set_team_color(TEAM_RED);
        }
        void GameStateModule::switchKickOff()
        {
            latest_data.set_kick_off_team(latest_data.kick_off_team() ? team_number : team_number+1);
        }

        const int WHISTLE_PORT = 30005;
        const char * WHISTLE_HOST = "127.0.0.1";

        void GameStateModule::whistleHandler(int last, int& next) {
            if (last == STATE_READY
                && next == STATE_SET) {
                closeSocket();
                processStartListen();
                heard_whistle = false;

                return;
            }

            if (last == STATE_SET
                && next == STATE_SET) {

                if ( processHeardWhistle() ) {
                    printf(":::: WHISTLE OVERRIDE ::::\n");
                    next = STATE_PLAYING;
                    heard_whistle = true;
                    closeSocket();

                } else if (heard_whistle) {
                    printf(":::: WHISTLE OVERRIDE ::::\n");
                    std::cout << "GameStateModule::whistleHandler() latest_data.state == STATE_SET BUT heard_whistle !!!ERROR!!!" << std::cout;
                    next = STATE_PLAYING;
                }

                return;
            }

            if ( last == STATE_SET && next == STATE_PLAYING ) {
                processEndListening();
                heard_whistle = false;

                return;
            }
        }
        
#define PERROR_AND_FAIL_IF( c , str )   \
if (c) { printf("GameStateModule::processConnect(): %s\n\n", str); return -1; }
        
        int GameStateModule::connectSocket() {
            if (processSocket > 0)
                return 0;
            else {
                struct sockaddr_in server;
                bzero(&server, sizeof(server));
                
                int sock = socket(AF_INET, SOCK_STREAM, 0);
                PERROR_AND_FAIL_IF(sock <= 0, "could not create client socket!")
                
                server.sin_family = AF_INET ;
                server.sin_port = htons(WHISTLE_PORT);
                
                struct hostent * hent = gethostbyname(WHISTLE_HOST);
                PERROR_AND_FAIL_IF(!hent, "could not get host ip")
                
                bcopy(hent->h_addr, &server.sin_addr.s_addr, hent->h_length);
                
                int ret = connect(sock, (struct sockaddr *) &server, (socklen_t) sizeof(struct sockaddr_in) );
                
                PERROR_AND_FAIL_IF(ret, "could not connect client socket!")
                processSocket = sock;
                return 0;
            }
        }
        
        void GameStateModule::closeSocket() {
            if (processSocket > 0) {
                close(processSocket);
                processSocket = 0;
            } else {
                processSocket = 0;
            }
        }
        
        enum {
            PROCESS_HEARD_WHISTLE = 0,
            PROCESS_START_LISTENING = 1,
            PROCESS_END_LISTENING = 2
        };
        
        
        
        int GameStateModule::processConnect(int _request) {
            
            connectSocket();
            
            if (processSocket > 0) {
                uint8_t request = _request, response;
                PERROR_AND_FAIL_IF( send(processSocket, &request, 1, MSG_NOSIGNAL) < 0, "send() error" );
                PERROR_AND_FAIL_IF( recv(processSocket, &response, 1, MSG_NOSIGNAL) < 0, "recv() error" );
                
                printf(":");
                
                return response;
            } else {
                return 0;
            }
        }
        
        bool GameStateModule::processHeardWhistle() {
            int ret = processConnect(PROCESS_HEARD_WHISTLE);
            if (ret < 0) closeSocket();
            return (ret == 1);
        }
        
        void GameStateModule::processStartListen() {
            int ret = processConnect(PROCESS_START_LISTENING);
            if (ret < 0) closeSocket();
        }
        
        void GameStateModule::processEndListening() {
            processConnect(PROCESS_END_LISTENING);
            closeSocket();
        }
        
    }
}
