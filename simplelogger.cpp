#include "simplelogger.hpp"

// Generates filename (in format prefix-log-data-time)

std::string SimpleLogger::generateFileName()
{
    std::chrono::time_point<std::chrono::system_clock> time_now = std::chrono::system_clock::now();
    std::time_t t_n = std::chrono::system_clock::to_time_t(time_now);

    char logfile_creation_time[100];
    std::strftime(logfile_creation_time, sizeof(logfile_creation_time), "%Y%m%d-%H%M%S", std::localtime(&t_n));
    std::string logname = this->log_name_prefix + "-log-" + std::string(logfile_creation_time) + ".log";

    return logname;
}

// Generates prefix (in format of [TIME] / [FLAG]) for log string

std::string SimpleLogger::generateLogString(int flag = INFO)
{
    auto time = std::chrono::system_clock::now(); // get the current time
    std::time_t t = std::chrono::system_clock::to_time_t(time);

    char log_time[100];
    std::strftime(log_time, sizeof(log_time), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    auto since_epoch = time.time_since_epoch(); // get the duration since epoch

    auto time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch);
    auto time_seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);

    unsigned long long millis = time_millis.count() - 1000*time_seconds.count();

    std::string logstring = std::string("[") + std::string(log_time) + std::string(":") + std::to_string(millis) + std::string("]");

    // Different flags
    switch(flag)
    {
    case ERROR:
        logstring = logstring + " / [ERROR]\t";
        break;
    case DEBUG_ERROR:
        logstring = logstring + " / [ERROR]\t";
        break;
    case DONE:
        logstring = logstring + " / [DONE]\t";
        break;
    case WARN:
        logstring = logstring + " / [WARN]\t";
        break;
    case INFO:
        logstring = logstring + " / [INFO]\t";
        break;
    case OK:
        logstring = logstring + " / [OK]\t";
        break;
    case DEBUG:
        logstring = logstring + " / [DEBUG]\t";
        break;
    default:
        logstring = logstring + " / [UNDEF]\t";
        break;
    }

    return logstring;
}

// Main Constructors (we need to send reference to our main log stream)

SimpleLogger::SimpleLogger(std::ostream& ls = std::cout, std::string prefix = "", std::string filename = "") : log_stream(ls)
{
    this->initializeLogger(filename, prefix);
}

SimpleLogger::SimpleLogger(std::ostream& ls = std::cout) : log_stream(ls)
{
    this->initializeLogger("", "some");
}

SimpleLogger::SimpleLogger(std::ostream& ls, std::string prefix) : log_stream(ls)
{
    this->initializeLogger("", prefix);
}

// Initialize function

void SimpleLogger::initializeLogger(std::string filename = "", std::string prefix = "")
{
    this->log_flag.flag = INFO;
    this->log_name_prefix = prefix;
    this->log_string = "";
    this->console_enabled = false;

    if(filename == "")
    {
        this->log_name = generateFileName();
    }
    else
    {
        this->log_name = prefix + filename;
    }

    this->file_stream.open(this->log_name);
    this->log_stream.rdbuf(this->file_stream.rdbuf());
}

// Copy logger constructor

SimpleLogger::SimpleLogger(const SimpleLogger& other) : log_stream(other.log_stream)
{
    this->log_flag = other.log_flag;
    this->log_name_prefix = other.log_name_prefix;
    this->log_string = other.log_string;
    this->console_enabled = other.console_enabled;
    this->log_name = other.log_name;
}

// Close logger stream

void SimpleLogger::closeLogger()
{
    this->file_stream.close();
}

// Destructor

SimpleLogger::~SimpleLogger()
{
    this->closeLogger();
}

// Setting logger flag

void SimpleLogger::setLoggerFlag(LogPref::Flag log_flag = LogPref::Flag(INFO))
{
    this->log_flag = log_flag;
}

// Setting ability to console output

void SimpleLogger::enableConsoleOutput(bool enable_console)
{
    console_enabled = enable_console;
}

// Sets flag just for one record;

SimpleLogger SimpleLogger::operator<<(LogPref::Flag flag)
{
    SimpleLogger s(*this);
    s.setLoggerFlag(flag);
    return s;
}

// Shift operators (follow standard ostream shift operators)

SimpleLogger SimpleLogger::operator<<(bool val){
    // Firstly printing in console
    if(console_enabled){
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    // Printing in file
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(long val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(unsigned long val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(long long val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(unsigned long long val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(double val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(long double val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const void* val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::nullptr_t) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << "nullptr";
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << "nullptr";
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(short val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(int val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(unsigned short val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(unsigned int val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(float val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::streambuf *sb) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << sb;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << sb;;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::ios_base& (*func)(std::ios_base&)) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::ostream& (*func)(std::ostream&)) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::_Setfill<char> func)
{
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::_Setw func)
{
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << func;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(std::string val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const char* val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const signed char* val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const unsigned char* val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const char val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}

SimpleLogger SimpleLogger::operator<<(const unsigned char val) {
    if (console_enabled) {
        std::cout << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    }
    log_stream << (this->log_flag.flag == NO_LOG_STRING ? "" : generateLogString(this->log_flag.flag)) << val;
    return *this << LogPref::Flag(NO_LOG_STRING);
}
