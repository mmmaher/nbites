/**
* @file ReceivedSplStandardMessagesProvider.h
* Contains the recently received SPLStandardMessage of each teammate
* to be used in DropInChallenge
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"

MODULE(ReceivedSPLStandardMessagesProvider,
{,
  REQUIRES(FrameInfoBH),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ReceivedSPLStandardMessagesBH),
});

class ReceivedSPLStandardMessagesProvider : public ReceivedSPLStandardMessagesProviderBase
{
private:
  static PROCESS_WIDE_STORAGE(ReceivedSPLStandardMessagesProvider) theInstance;

  SPLStandardMessageWithoutData recentMessages[TeammateDataBH::numOfPlayers];
  unsigned recentMessagesTimestamps[TeammateDataBH::numOfPlayers];

public:
  ReceivedSPLStandardMessagesProvider();
  ~ReceivedSPLStandardMessagesProvider();

  void update(ReceivedSPLStandardMessagesBH& messages);

  static void addMessage(SPLStandardMessageBH& msg);
};