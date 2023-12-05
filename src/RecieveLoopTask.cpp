#include "../include/RecieveLoopTask.hpp"

RecieveLoopTask::RecieveLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef)
    : systemState(systemStateRef), canUtils(canUtilsRef)
{
}

void RecieveLoopTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        if (systemState.main == Main::Perform)
            RecieveLoop(recieveBuffer);
    }
}

void RecieveLoopTask::checkUserInput()
{
    if (kbhit())
    {
        char input = getchar();
        if (input == 'q')
            systemState.runMode = RunMode::Pause;
        else if (input == 'e')
            systemState.runMode = RunMode::Stop;
        else if (input == 'r')
            systemState.runMode = RunMode::Running;
    }
}

void RecieveLoopTask::RecieveLoop(queue<can_frame> &recieveBuffer)
{
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    canUtils.set_all_sockets_timeout(0, 50000);
    canUtils.clear_all_can_buffers();

    Sensor sensor;

    while (systemState.runMode == RunMode::Running)
    {
        checkUserInput();

        if (systemState.runMode == RunMode::Pause)
        {
            continue;
        }

        if ((sensor.ReadVal() & 1) != 0)
        {
            cout << "Motors at Sensor Loacation please check!!!\n";
            systemState.runMode = RunMode::Pause;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::milliseconds elapsed_time = chrono::duration_cast<chrono::milliseconds>(internal - external);
        if (elapsed_time.count() >= TIME_THRESHOLD_MS)
        {
            external = std::chrono::system_clock::now();
            for (const auto &socket_pair : canUtils.sockets)
            {
                int socket_descriptor = socket_pair.second;

                handleSocketRead(socket_descriptor, recieveBuffer);
            }
        }
    }

    parse_and_save_to_csv("TuningData/DrumData_out.txt");
}

void RecieveLoopTask::handleSocketRead(int socket_descriptor, queue<can_frame> &recieveBuffer)
{
    struct can_frame readFrame[NUM_FRAMES];

    ssize_t bytesRead = read(socket_descriptor, &readFrame, sizeof(can_frame) * NUM_FRAMES);
    if (bytesRead == -1)
    {
        std::cerr << "Failed to read from socket." << std::endl;
        return;
    }

    int numFramesRead = bytesRead / sizeof(can_frame); // 중복 연산 최소화
    for (int i = 0; i < numFramesRead; ++i)
    {
        recieveBuffer.push(readFrame[i]);
    }
}

void RecieveLoopTask::parse_and_save_to_csv(const std::string &csv_file_name)
{
    // CSV 파일 열기. 파일이 없으면 새로 생성됩니다.
    std::ofstream ofs(csv_file_name, std::ios::app);
    int id;
    float position, speed, torque;

    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // 파일이 새로 생성되었으면 CSV 헤더를 추가
    ofs.seekp(0, std::ios::end);
    if (ofs.tellp() == 0)
    {
        ofs << "CAN_ID,p_act,tff_des,tff_act\n";
    };

    while (!recieveBuffer.empty())
    {
        can_frame frame = recieveBuffer.front();
        recieveBuffer.pop();

        for (const auto &pair : tmotors)
        {
            std::shared_ptr<TMotor> motor = pair.second;
            if (motor->nodeId == frame.data[0])
            {
                std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
                id = std::get<0>(parsedData);
                position = std::get<1>(parsedData);
                speed = std::get<2>(parsedData);
                torque = std::get<3>(parsedData);

                ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                    << std::dec
                    << position << "," << speed << "," << torque << "\n";
            }
        }
    }

    ofs.close();
}
