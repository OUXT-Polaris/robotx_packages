#ifndef REMOTE_OPERATED_INTERFACE_H_INCLUDED
#define REMOTE_OPERATED_INTERFACE_H_INCLUDED

class remote_operated_interface
{
public:
    struct parameters
    {
        enum controller_type {dualshock4=0};
        int controller;
        parameters()
        {
            controller = dualshock4;
        }
    };
    remote_operated_interface();
    ~remote_operated_interface();
};

#endif  //REMOTE_OPERATED_INTERFACE_H_INCLUDED