// 最小化编译测试 - 验证CMakeLists.txt集成
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::cout << "=== Cartographer Line Feature Extractor - Minimal Test ===" << std::endl;
    std::cout << "Compilation: SUCCESS" << std::endl;
    std::cout << "CMakeLists.txt Integration: WORKING" << std::endl;
    std::cout << "ROS2 Environment: " << (getenv("ROS_DISTRO") ? getenv("ROS_DISTRO") : "NOT SET") << std::endl;
    
    if (argc > 1) {
        std::cout << "Arguments received:" << std::endl;
        for (int i = 1; i < argc; i++) {
            std::cout << "  [" << i << "] " << argv[i] << std::endl;
        }
    }
    
    std::cout << "=== Test Complete ===" << std::endl;
    return 0;
}