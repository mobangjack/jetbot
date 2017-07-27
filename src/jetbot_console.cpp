#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "asp.h"

using namespace std;

#define STARTUP_TIME 30
#define COOLING_TIME 3

typedef enum
{
    STARTUP,
    ACTIVE,
    COOLING,
    IDLE,
} WorkState_e;

WorkState_e workState = IDLE;
WorkState_e lastWorkState = IDLE;

double t_speed = 0.5;
double r_speed = t_speed * PI;

double last_t_speed = t_speed;
double last_r_speed = r_speed;

void saveSpeed()
{
    last_t_speed = t_speed;
    last_r_speed = r_speed;
}

void recoverSpeed()
{
    t_speed = last_t_speed;
    r_speed = last_r_speed;
}

geometry_msgs::Twist twist;

void setTwist(double x, double y, double z)
{
    twist.linear.x  =  x;
    twist.linear.y  =  y;
    twist.angular.z = z;
}

void cmdTwist(char cmd, geometry_msgs::Twist& twist)
{
    switch(cmd)
    {
        case 'w':
            setTwist(t_speed, 0, 0);
            printf("vx:%f\n", t_speed);
            break;
        case 's':
            setTwist(-t_speed, 0, 0);
            printf("vx:%f\n", -t_speed);
            break;
        case 'a':
            setTwist(0, t_speed, 0);
            printf("vy:%f\n", t_speed);
            break;
        case 'd':
            setTwist(0, -t_speed, 0);
            printf("vy:%f\n", -t_speed);
            break;
        case 'r':
            setTwist(0, 0, r_speed);
            printf("wz:%f\n", r_speed);
            break;
        case 'f':
            setTwist(0, 0, -r_speed);
            printf("wz:%f\n", -r_speed);
            break;
        default:
            //setTwist(0, 0, 0);
            break;

    }
}

uint32_t timer = 0;

bool isNumKey(char ch)
{
    return (ch >= '0' && ch <= '9');
}

bool isNavKey(char ch)
{
    return (ch=='w'|| ch=='s'|| ch=='a'|| ch=='d'|| ch=='r'|| ch=='f');
}

bool isHotKey(char ch)
{
    return isNumKey(ch) || isNavKey(ch);
}

void cmdSpeed(char ch)
{
    if (isNumKey(ch))
    {
        t_speed = 0.1 * (ch - '0');
        r_speed = t_speed * PI;
    }
}

void workstateMachine(char ch)
{
    lastWorkState = workState;
    switch (workState)
    {
        case STARTUP:
            if (timer < STARTUP_TIME)
            {
                timer++;
            }
            else
            {
                workState = ACTIVE;
            }
            break;
        case ACTIVE:
            if (isHotKey(ch))
            {
                timer = 0;
            }
            else
            {
                workState = COOLING;
            }
            break;
        case COOLING:
            if (isHotKey(ch))
            {
                timer = 0;
                workState = ACTIVE;
            }
            else if (timer < COOLING_TIME)
            {
                timer++;
            }
            else
            {
                saveSpeed();
                workState = IDLE;
            }
            break;
        case IDLE:
            if (isHotKey(ch))
            {
                timer = 0;
                recoverSpeed();
                workState = STARTUP;
            }
            break;
        default:
            workState = IDLE;
            break;
    }
}

void action(char ch)
{
    switch (workState)
    {
        case STARTUP:
        case ACTIVE:
            cmdTwist(ch, twist);
            break;
        case COOLING:
            break;
         case IDLE:
            setTwist(0, 0, 0);
            break;

    }
}

char kbhit()  
{  
    struct termios oldt, newt;  
    char ch;  
    int oldf;  
    tcgetattr(STDIN_FILENO, &oldt);  
    newt = oldt;  
    newt.c_lflag &= ~(ICANON | ECHO);  
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if((ch >= '0' && ch <= '9') || ch=='w'|| ch=='s'|| ch=='a'|| ch=='d'|| ch=='r'|| ch=='f')
    {
        printf("\n");
    }
    return ch;  
}

void help()
{
    const char str[] = "\nHelp:\n\t0-9: speed setup\n\twsadrf: move control\n";
    printf(str);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jetbot_console");

    int spin_rate = 50;
  
    ros::NodeHandle np("~");
    np.param<int>("spin_rate", spin_rate, 50); 

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate rate(spin_rate);

    help();

    while(n.ok())
    {
        ros::spinOnce();

        char ch = kbhit();

        if (ch == 'h')
        {
            help();
        }

        cmdSpeed(ch);


        workstateMachine(ch);
        action(ch);

        
        //cmdTwist(ch, twist);

        pub.publish(twist);

        rate.sleep();
    }
}
