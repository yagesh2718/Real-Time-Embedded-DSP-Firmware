#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <cmath>
#include <sstream>

namespace test {

class TestRegistry {
public:
    struct TestEntry {
        std::string name;
        std::function<void()> func;
    };

    static TestRegistry& instance() {
        static TestRegistry reg;
        return reg;
    }

    void register_test(const std::string& name, std::function<void()> func) {
        tests_.push_back({name, func});
    }

    int run_all() {
        std::cout << "\n=======================================================\n";
        std::cout << "           RUNNING AUTOMATED FIRMWARE DSP TESTS        \n";
        std::cout << "=======================================================\n";

        int passed = 0;
        int failed = 0;

        for (const auto& t : tests_) {
            current_test_name_ = t.name;
            current_test_failed_ = false;
            current_test_error_.clear();

            try {
                t.func();
            } catch (const std::exception& e) {
                current_test_failed_ = true;
                current_test_error_ = std::string("Unhandled exception: ") + e.what();
            } catch (...) {
                current_test_failed_ = true;
                current_test_error_ = "Unknown exception thrown";
            }

            if (!current_test_failed_) {
                std::cout << " [PASS] " << t.name << "\n";
                passed++;
            } else {
                std::cout << " [FAIL] " << t.name << "\n";
                if (!current_test_error_.empty()) {
                    std::cout << "        Reason: " << current_test_error_ << "\n";
                }
                failed++;
            }
        }

        std::cout << "=======================================================\n";
        std::cout << " Total: " << tests_.size()
                  << " | Passed: " << passed
                  << " | Failed: " << failed << "\n";
        std::cout << "=======================================================\n\n";

        return (failed == 0) ? 0 : 1;
    }

    void fail(const std::string& msg, const char* file, int line) {
        current_test_failed_ = true;
        std::ostringstream ss;
        ss << file << ":" << line << " - " << msg;
        current_test_error_ = ss.str();
        throw std::runtime_error(current_test_error_);
    }

private:
    std::vector<TestEntry> tests_;
    std::string current_test_name_;
    bool current_test_failed_{false};
    std::string current_test_error_;
};

struct TestAutoRegistrar {
    TestAutoRegistrar(const std::string& name, std::function<void()> func) {
        TestRegistry::instance().register_test(name, func);
    }
};

} // namespace test

#define TEST_CASE(name) \
    static void test_func_##name(); \
    static ::test::TestAutoRegistrar reg_##name(#name, test_func_##name); \
    static void test_func_##name()

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            ::test::TestRegistry::instance().fail("ASSERT_TRUE failed: (" #condition ") is false", __FILE__, __LINE__); \
        } \
    } while (0)

#define ASSERT_FALSE(condition) \
    do { \
        if ((condition)) { \
            ::test::TestRegistry::instance().fail("ASSERT_FALSE failed: (" #condition ") is true", __FILE__, __LINE__); \
        } \
    } while (0)

#define ASSERT_EQ(val1, val2) \
    do { \
        if (!((val1) == (val2))) { \
            std::ostringstream ss; \
            ss << "ASSERT_EQ failed: " #val1 " (" << (val1) << ") != " #val2 " (" << (val2) << ")"; \
            ::test::TestRegistry::instance().fail(ss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

#define ASSERT_NEAR(val1, val2, tolerance) \
    do { \
        double diff = std::abs(static_cast<double>(val1) - static_cast<double>(val2)); \
        if (diff > static_cast<double>(tolerance)) { \
            std::ostringstream ss; \
            ss << "ASSERT_NEAR failed: |" #val1 " (" << (val1) << ") - " #val2 " (" << (val2) << ")| = " \
               << diff << " > tolerance (" << (tolerance) << ")"; \
            ::test::TestRegistry::instance().fail(ss.str(), __FILE__, __LINE__); \
        } \
    } while (0)
