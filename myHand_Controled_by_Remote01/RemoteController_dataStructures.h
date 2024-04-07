struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    bool menuDown;      
    bool Select; 
    bool menuUp;  
    bool toggleBottom;  
    bool toggleTop; 
    int mode;  

    bool navKeyUp    ;
    bool navKeyDown  ;
    bool navKeyLeft  ;
    bool navKeyRight ;
    bool navKeyMiddle;
    bool navKeySet   ;
    bool navKeyReset ;

    int16_t stick1_X;
    int16_t stick1_Y;
    bool    stick1_Btn;

    int16_t stick2_X;
    int16_t stick2_Y;
    bool    stick2_Btn;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

