#include <iostream>

void test_units();
void test_pose();
void test_pid();
void test_feedforward();
void test_actions();
void test_mcl();

int main() {
    std::cout << "Running Vektor Extensive Unit Test Suite...\n";
    std::cout << "===========================================\n";

    test_units();
    test_pose();
    test_pid();
    test_feedforward();
    test_actions();
    test_mcl();

    std::cout << "===========================================\n";
    std::cout << "All Vektor tests completed successfully!\n";
    return 0;
}
