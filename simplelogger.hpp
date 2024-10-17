#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <fstream>
#include <sstream>
#include <fstream>

#define ERROR -1 // Logger flags
#define WARN 1
#define INFO 0
#define NO_LOG_STRING 2

#ifndef SIMPLELOGGER_HPP
#define SIMPLELOGGER_HPP
namespace LogPref
{
class Flag {
public:
    int flag;
    Flag(int flag) {this->flag = flag;}
    Flag() {this->flag = INFO;}
};
}

namespace LogManip
{
std::ostream& setfill(std::ostream out, char c);
}

class SimpleLogger {

protected:
    std::ostream& log_stream;
    std::ofstream file_stream;

    std::string log_name;
    std::string log_name_prefix;
    std::string log_string;
    LogPref::Flag log_flag;
    bool console_enabled;

    std::string generateFileName();
    std::string generateLogString(int flag);

public:
    SimpleLogger(std::ostream& ls, std::string prefix, std::string filename);
    SimpleLogger(std::ostream& ls, std::string prefix);
    SimpleLogger(std::ostream& ls);
    SimpleLogger(const SimpleLogger& other);
    void initializeLogger(std::string filename, std::string prefix);
    void closeLogger();
    void enableConsoleOutput(bool enable_console);
    void setLoggerFlag(LogPref::Flag log_flag);
    ~SimpleLogger();

    SimpleLogger operator<<(LogPref::Flag flag);
    SimpleLogger operator<<(bool val);
    SimpleLogger operator<<(long val);
    SimpleLogger operator<<(unsigned long val);
    SimpleLogger operator<<(long long val);
    SimpleLogger operator<<(unsigned long long val);
    SimpleLogger operator<<(double val);
    SimpleLogger operator<<(long double val);
    SimpleLogger operator<<(const void* val);
    SimpleLogger operator<<(std::nullptr_t);
    SimpleLogger operator<<(short val);
    SimpleLogger operator<<(int val);
    SimpleLogger operator<<(unsigned short val);
    SimpleLogger operator<<(unsigned int val);
    SimpleLogger operator<<(float val);
    SimpleLogger operator<<(std::streambuf *sb);
    SimpleLogger operator<<(std::ios_base& (*func)(std::ios_base&));
    SimpleLogger operator<<(std::ostream& (*func)(std::ostream&));
    SimpleLogger operator<<(std::_Setfill<char> func);
    SimpleLogger operator<<(std::_Setw func);
    SimpleLogger operator<<(std::string val);
    SimpleLogger operator<<(const char* val);
    SimpleLogger operator<<(const signed char* val);
    SimpleLogger operator<<(const unsigned char* val);
    SimpleLogger operator<<(const char val);
    SimpleLogger operator<<(const unsigned char val);
};

#endif // SIMPLELOGGER_HPP
