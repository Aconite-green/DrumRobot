#pragma once

enum class State {
    SystemInit,    // 시스템 시작: CAN 포트 열기 및 모터 연결 상태 확인
    Home,    // CAN 포트 열림 및 모터 연결 확인    // 모든 모터가 홈 위치에 도달함
    Tuning,        // 모터들의 튜닝 준비 상태
    Performing,    // 드럼 연주 진행 중 (드럼 로봇)
    Shutdown,      // 시스템 종료 및 모든 작업 마무리
    Error       
};

enum class HomeMode{


};
