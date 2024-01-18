#include "../include/tasks/SendLoopTask.hpp"

SendLoopTask::SendLoopTask(SystemState &systemStateRef,
                           CanManager &canManagerRef,
                           std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef), pathManager(motorsRef)
{
}

void SendLoopTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        if (systemState.main == Main::Back)
        {
            if (CheckAllMotorsCurrentPosition())
            {
                clearBuffer();
                cout << "Get Back...\n";
                pathManager.GetArr(pathManager.backarr);
                SendReadyLoop();
                systemState.main = Main::Ideal;
            }
        }

        usleep(50000);
        while (systemState.main == Main::Perform)
        {
            usleep(50000);

            if (systemState.runMode == RunMode::Preparing)
            {
                if (CheckAllMotorsCurrentPosition())
                {
                    initializePathManager();
                    cout << "Get Ready...\n";
                    pathManager.GetArr(pathManager.standby);
                    SendReadyLoop();
                    systemState.runMode = RunMode::Ready;
                }
            }
            else if (systemState.runMode == RunMode::Running)
            {
                if (CheckAllMotorsCurrentPosition())
                {
                    SendLoop();
                    systemState.runMode = RunMode::PrePreparation;
                }
            }
        }
    }
}

void SendLoopTask::motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
{
    this->motors = motorsRef;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  PERFORM                                   */
///////////////////////////////////////////////////////////////////////////////

void SendLoopTask::SendLoop()
{

    struct can_frame frameToProcess;
    std::string maxonCanInterface;
    std::shared_ptr<GenericMotor> virtualMaxonMotor;

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
            maxonCanInterface = maxonMotor->interFaceName;
            virtualMaxonMotor = motor_pair.second;
        }
    }
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    while (systemState.runMode != RunMode::PrePreparation)
    {

        if (systemState.runMode == RunMode::Pause)
        {
            continue;
        }

        bool isAnyBufferLessThanTen = false;
        for (const auto &motor_pair : motors)
        {
            if (motor_pair.second->sendBuffer.size() < 10)
            {
                isAnyBufferLessThanTen = true;
                break;
            }
        }
        if (isAnyBufferLessThanTen)
        {
            if (pathManager.line < pathManager.total)
            {
                std::cout << "line : " << pathManager.line << ", total : " << pathManager.total << "\n";
                pathManager.PathLoopTask();
                pathManager.line++;
            }
            else if (pathManager.line == pathManager.total)
            {
                std::cout << "Turn Back\n";
                CheckAllMotorsCurrentPosition();
                pathManager.GetArr(pathManager.backarr);
                pathManager.line++;
            }
        }

        bool allBuffersEmpty = true;
        for (const auto &motor_pair : motors)
        {
            if (!motor_pair.second->sendBuffer.empty())
            {
                allBuffersEmpty = false;
                break;
            }
        }

        // 모든 모터의 sendBuffer가 비었을 때 성능 종료 로직 실행
        if (allBuffersEmpty)
        {
            std::cout << "Performance is Over\n";
            systemState.runMode = RunMode::PrePreparation;
            systemState.main = Main::Ideal;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            for (auto &motor_pair : motors)
            {
                shared_ptr<GenericMotor> motor = motor_pair.second;
                canManager.sendFromBuff(motor);
            }

            if (maxonMotorCount != 0)
            {
                maxoncmd.getSync(&frameToProcess);
                canManager.txFrame(virtualMaxonMotor, frameToProcess);
            }
        }
    }

    // CSV 파일명 설정
    std::string csvFileName = "../../READ/DrumData_in.txt";

    // input 파일 저장
    save_to_txt_inputData(csvFileName);
}

void SendLoopTask::save_to_txt_inputData(const string &csv_file_name)
{
    // CSV 파일 열기
    std::ofstream csvFile(csv_file_name);

    if (!csvFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : pathManager.p)
    {
        for (const double cell : row)
        {
            csvFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                csvFile << ","; // 쉼표로 셀 구분
            }
        }
        csvFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    csvFile.close();

    std::cout << "연주 txt_InData 파일이 생성되었습니다: " << csv_file_name << std::endl;

    std::cout << "SendLoop terminated\n";
}

void SendLoopTask::SendReadyLoop()
{
    cout << "Settig...\n";
    struct can_frame frameToProcess;
    std::string maxonCanInterface;
    std::shared_ptr<GenericMotor> virtualMaxonMotor;

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
            maxonCanInterface = maxonMotor->interFaceName;
            virtualMaxonMotor = motor_pair.second;
        }
    }
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    bool allBuffersEmpty;
    do
    {
        allBuffersEmpty = true;
        for (const auto &motor_pair : motors)
        {
            if (!motor_pair.second->sendBuffer.empty())
            {
                allBuffersEmpty = false;
                break;
            }
        }

        if (!allBuffersEmpty)
        {
            chrono::system_clock::time_point internal = std::chrono::system_clock::now();
            chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

            if (elapsed_time.count() >= 5000) // 5ms
            {
                external = std::chrono::system_clock::now();

                for (auto &motor_pair : motors)
                {
                    shared_ptr<GenericMotor> motor = motor_pair.second;
                    canManager.sendFromBuff(motor);
                }

                if (maxonMotorCount != 0)
                {
                    maxoncmd.getSync(&frameToProcess);
                    canManager.txFrame(virtualMaxonMotor, frameToProcess);
                }
            }
        }
    } while (!allBuffersEmpty);
    canManager.clearReadBuffers();
}

void SendLoopTask::initializePathManager()
{
    pathManager.motorInitialize(motors);
    pathManager.GetDrumPositoin();
    pathManager.GetMusicSheet();
}
bool SendLoopTask::CheckAllMotorsCurrentPosition()
{

    std::cout << "Checking all positions for motors [rad]\n"
              << endl;
    bool allMotorsChecked = true;
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        if (!checkMotorPosition(motor))
        {
            allMotorsChecked = false;
        }
    }
    return allMotorsChecked;
}

void SendLoopTask::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " : " << motor->currentPos << endl;
    }
}

bool SendLoopTask::checkMotorPosition(std::shared_ptr<GenericMotor> motor)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 5000 /*5ms*/);
    canManager.clearReadBuffers();

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        tmotorcmd.getControlMode(*tMotor, &frame);
        if (canManager.sendAndRecv(motor, frame))
        {
            std::tuple<int, float, float, float> parsedData = tmotorcmd.parseRecieveCommand(*tMotor, &frame);
            motor->currentPos = std::get<1>(parsedData);
        }
        else
        {
            return false;
        }
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        maxoncmd.getSync(&frame);
        canManager.txFrame(motor, frame);
        
        if (canManager.recvToBuff(motor, 2))
        {
            while (!motor->recieveBuffer.empty())
            {
                frame = motor->recieveBuffer.front();
                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    std::tuple<int, float, float> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    motor->currentPos = std::get<1>(parsedData);
                }
                motor->recieveBuffer.pop();
            }
        }
        else
        {
            return false;
        }
    }
    return true;
}



void SendLoopTask::clearBuffer()
{
    for (auto motor_pair : motors)
    {
        motor_pair.second->clearSendBuffer();
    }
}
