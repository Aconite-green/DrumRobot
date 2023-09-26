

// CommandParser.hpp
class CommandParser {
public:
    virtual void parseEventCommand(struct can_frame *frame, int can_id) = 0;
    virtual void parseControlCommand(struct can_frame *frame, int can_id) = 0;
    // ...
};

class TMotorCommandParser : public CommandParser {
    void parseControlCommand(struct can_frame *frame, int can_id) override;
    void parseEventCommand(struct can_frame *frame, int can_id) override;
};



class MaxonCommandParser : public CommandParser {
    void parseControlCommand(struct can_frame *frame, int can_id) override;
    void parseEventCommand(struct can_frame *frame, int can_id) override;
};