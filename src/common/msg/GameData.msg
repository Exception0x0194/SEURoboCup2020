int32 mode

int32 MODE_NORM = 1
int32 MODE_KICK = 2

int32 state
int32 remain_time
int32 second_time
int32 red_score
int32 blue_score

Player[2] red_players
Player[2] blue_players

int32 STATE_INIT = 0
int32 STATE_READY = 1
int32 STATE_PLAY = 2
int32 STATE_PAUSE = 3
int32 STATE_END = 4