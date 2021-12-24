 #include <ros/ros.h>
 #include <roborts_msgs/GameRobotHP.h>
 #include <roborts_msgs/GameRobotBullet.h>
 #include <roborts_msgs/GameStatus.h>
 #include <roborts_msgs/GameZoneArray.h>
 #include <roborts_msgs/RobotHeat.h>
 #include <roborts_msgs/RobotStatus.h>


enum buff_type{
    RED_HP_RECOVERY=1,
    RED_BULLET_SUPPLY=2,
    BLUE_HP_RECOVERY=3,
    BLUE_BULLET_SUPPLY=4,
    DISABLE_SHOOTING=5,
    DISABLE_MOVEMENT=6,
};


class referee_system
{
    public:
        referee_system();
        ~referee_system();
        void loss_hp(int _id ,int _loss=20 );
        void loss_bullet( int _id ,int _loss=20 );
        void refresh_hp();
        void game_start();
    private:
        // GameHP gamehp_{2000,2000,2000,2000};
        // GameBullet gamebullet_{50,0,50,0};
        int buff_active[6];
        int buff_type[6];
        roborts_msgs::GameRobotHP gamehp_;
        roborts_msgs::GameRobotBullet gamebullet_;
        roborts_msgs::GameStatus game_status_;
        ros::NodeHandle ref_nh_;
        // publisher
        ros::Publisher game_status_pub_;
        ros::Publisher hp_pub_;
        ros::Publisher bullet_pub_;
};

referee_system::referee_system()
{
    game_status_pub_ = ref_nh_.advertise<roborts_msgs::GameStatus>("game_status", 10,this);
    hp_pub_ = ref_nh_.advertise<roborts_msgs::GameRobotHP>("game_robot_hp", 30, this);
    bullet_pub_ = ref_nh_.advertise<roborts_msgs::GameRobotBullet>("game_robot_bullet",30, this);
}

referee_system::~referee_system()= default;

void referee_system::loss_hp(int _id ,int _loss){
    if(_id == 0){
        gamehp_.blue1 -= _loss;
        gamehp_.blue1 =  gamehp_.blue1 < 0 ? 0 : gamehp_.blue1;
    }
    if(_id == 1){
        gamehp_.blue2 -= _loss;
        gamehp_.blue2 = gamehp_.blue2 < 0 ? 0 : gamehp_.blue2;
    }
    if(_id == 2){
        gamehp_.red1 -= _loss;
        gamehp_.red1 =  gamehp_.red1 < 0 ? 0 : gamehp_.red1;
    }
    if(_id == 3){
        gamehp_.red2 -= _loss;
        gamehp_.red2 = gamehp_.red2 < 0 ? 0 : gamehp_.red2;
    }
    hp_pub_.publish(gamehp_);
}


void referee_system::loss_bullet(int _id ,int _loss){
    if(_id == 0){
        gamebullet_.blue1 -= _loss;
        gamebullet_.blue1 = gamebullet_.blue1 < 0 ? 0 : gamebullet_.blue1;
    }
    if(_id == 1){
        gamebullet_.blue2 -= _loss;
        gamebullet_.blue2 = gamebullet_.blue2 < 0 ? 0 : gamebullet_.blue2;
    }
    if(_id == 2){
        gamebullet_.red1 -= _loss;
        gamebullet_.red1 = gamebullet_.red1 < 0 ? 0 : gamebullet_.red1;
    }
    if(_id == 3){
        gamebullet_.red2 -= _loss;
        gamebullet_.red2 = gamebullet_.red2 < 0 ? 0 : gamebullet_.red2;
    }
    bullet_pub_.publish(gamebullet_);
}

void referee_system::refresh_hp(){
    gamehp_.blue1 = 2000;
    gamehp_.blue2 = 2000;
    gamehp_.red1 = 2000;
    gamehp_.red2 = 2000;
    hp_pub_.publish(gamehp_);
}


void referee_system::game_start(){
    game_status_.game_status = game_status_.GAME;
    game_status_pub_.publish(game_status_);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "referee_system_node");
    referee_system referee_system;
    char command = '0';
    while (command != '4') {
        std::cout << "**************************************************************************************" << std::endl;
        std::cout << "*********************************please send a command********************************" << std::endl;
        std::cout << "1: game start" << std::endl
                << "2: bullet loss 20  " << std::endl
                << "3: blue master hp loss 20 " << std::endl
                << "4: blue wing   hp loss 20 " << std::endl
                << "5: red  master hp loss 20 " << std::endl
                << "6: red  wing   hp loss 20 " << std::endl
                << "7: refresh buff zone " << std::endl
                << "8: refresh all hp and bullet  " << std::endl;
        std::cout << "**************************************************************************************" << std::endl;
        std::cout << "> ";
        std::cin >> command;
        if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7') {
        std::cout << "please inpugain!" << std::endl;
        std::cout << "> ";
        std::cin >> command;
        }

    //     switch (command) {
    //     //start thread.
    //     case '1':
    //         referee_system.game_start();
    //         break;
    //     case '2':
    //         referee_system.loss_hp();
    //         break;
    //     case '3':
    //         referee_system.loss_hp(1);
    //         break;
    //     case '4':
    //         referee_system.loss_hp(2);
    //         break;
    //     case '5':
    //         referee_system.loss_hp(3);
    //         break;
    //     case '6':
    //         break;
    //     case '7':
    //         break;
    //     default:
    //         break;
    //     }
    }
  return 0;
}