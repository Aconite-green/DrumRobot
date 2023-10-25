#include "../include/TuningTask.hpp"
#include <cerrno>  // errno
#include <cstring> // strerror
#include <sstream>
#include <iomanip>

TuningTask::TuningTask(
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors,
    const std::map<std::string, int> &sockets) : kp(8), kd(1), sine_t(1.0), fileName(""), tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets)
{
   std::stringstream ss;

    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_period_" << sine_t << ".csv";

    // 파일 이름 자동 설정
    std::string folderName = "data";
    std::string baseName = ss.str(); // ss.str()로 stringstream의 내용을 std::string으로 가져옵니다.
    fileName = folderName + "/" + baseName;
}

int TuningTask::set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec)
{
    struct timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = timeout_usec;

    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        perror("setsockopt failed"); // perror 함수는 실패 원인을 출력해줍니다.
        return -1;                   // 실패 시 에러 코드 반환
    }
    return 0; // 성공 시 0 반환
}

void TuningTask::operator()()
{
    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    std::stringstream ss;

    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_period_" << sine_t << ".csv";

    // 파일 이름 자동 설정
    std::string folderName = "data";
    std::string baseName = ss.str(); // ss.str()로 stringstream의 내용을 std::string으로 가져옵니다.
    fileName = folderName + "/" + baseName;

    // 'data' 폴더가 없으면 생성
    if (!std::filesystem::exists(folderName))
    {
        std::filesystem::create_directory(folderName);
    }

    // CSV 파일을 쓰기 모드로 열기
    std::ofstream csvFile(fileName);
    csvFile << "CAN_ID,p_des,p_act,tff_des,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float sample_time = 0.005;
    int cycles = 2;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float v_des = 0;
    float tff_des = 0;
    float p_act, v_act, tff_act;
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (auto &entry : tmotors)
            {

                std::shared_ptr<TMotor> &motor = entry.second;

                float local_time = std::fmod(time, sine_t);
                float p_des = (1 - cosf(2 * M_PI * local_time / sine_t)) * M_PI / 2;

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, kp, kd, tff_des);
                csvFile << "0x" << std::hex << std::setw(4) << std::setfill('0') << motor->nodeId << ',' << std::dec << p_des;

                clock_t external = clock();
                while (1)
                {
                    clock_t internal = clock();
                    double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
                    if (elapsed_time >= 5)
                    {

                        ssize_t bytesWritten = write(sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));
                        if (bytesWritten == -1)
                        {
                            std::cerr << "Failed to write to socket for interface: " << motor->interFaceName << std::endl;
                            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                        }
                        ssize_t bytesRead = read(sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));

                        if (bytesRead == -1)
                        {
                            std::cerr << "Failed to read from socket for interface: " << motor->interFaceName << std::endl;
                            return;
                        }
                        else
                        {
                            std::tuple<int, float, float, float> result = TParser.parseRecieveCommand(*motor, &frame);

                            p_act = std::get<1>(result);
                            v_act = std::get<1>(result);
                            tff_act = std::get<3>(result);
                            tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                            csvFile << ',' << std::dec << p_act << ',' << tff_des << ',' << tff_act << '\n';
                            break;
                        }
                    }
                }
            }
        }
    }
    csvFile.close();
}
