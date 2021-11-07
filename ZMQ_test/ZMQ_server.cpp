#include <zmq.hpp>
#include <string>
int main()
{
        std::string ci;
        zmq::context_t context_(1);
        zmq::socket_t Socket_(context_, ZMQ_REP);
        Socket_.bind("tcp://*:5555");
        while (true)
        {       printf("等待接收\n");
                zmq::message_t rec_message;
                Socket_.recv(&rec_message);
                printf("接收到了");
                memcpy(&ci, rec_message.data(), sizeof(ci));
                ci = "World" ;
                zmq::message_t send_message(sizeof(ci));
                memcpy(send_message.data(), &ci, sizeof(ci));
                Socket_.send(send_message);
        
        }
        

}