#include "navigator/navigator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigator");
    Navigator navigator;
    navigator.process();
    return 0;
}
