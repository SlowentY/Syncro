//2023.10.03 duepp program format research
//2023.10.06 +due_download_prog() file output
//2023.10.07  +int due_download_prog_save_to_file, +loop for {events}
//10.18 +int due_download_prog_save_to_file_command/data
//11.01 +serial
//13.11.2023 + test program on real data
//06.02.2023 + reading sync pulse sequence from csv file
//22.05.2024 + added out stream to function due_upload_trajectory in serial_lib_cpp.cpp
//25.05.2024 + added stream ability in serial_lib_cpp.cpp


//digital pin 33 -> RF
//digital pin 40 -> ADC
//digital pin 5 -> RS485 converter direction
//digital pin 6 -> GRU



#include <iostream>

#include<iomanip> // for setbase(), works for base 8,10 and 16 only

#include <thread>
#include <chrono>
#include <ctime>
#include "serial_lib_cpp.cpp"
#include <sstream>
#include <fstream>
#include <list>
#include <iterator>
#include <stdlib.h>
#include "serialib.h" // Serial library
#include "pugixml.hpp"

#define ERR -1 // Logger flags
#define WARN 1
#define INFO 0

using namespace std;

using namespace std::literals::chrono_literals;

//#define SERIAL_PORT "\\\\.\\COM1"
//const string SERIAL_PORT{"\\\\.\\COM19"};

const string SERIAL_PORT{"\\\\.\\COM5"};
static ofstream os_log; // Logger stream
static bool en_cout = false;

//Initialization of logger

static void OpenFileLogger(string logfile_name = "")
{
    if(logfile_name == "") {
        std::chrono::time_point<std::chrono::system_clock> time_now = std::chrono::system_clock::now();
        std::time_t t_n = std::chrono::system_clock::to_time_t(time_now);

        char logfile_creation_time[100];
        std::strftime(logfile_creation_time, sizeof(logfile_creation_time), "%Y%m%d-%H%M%S", std::localtime(&t_n));
        logfile_name = "log-" + std::string(logfile_creation_time, 100) + ".txt";
    }

    os_log.open(logfile_name);
}

static void CloseFileLogger()
{
    os_log.close();
}

static void EnableConsoleLogger(bool en)
{
    en_cout = en;
}

// Logger function

static ofstream& out_log(short flag = INFO)
{
    auto time = std::chrono::system_clock::now(); // get the current time
    std::time_t t = std::chrono::system_clock::to_time_t(time);

    char log_time[100];
    std::strftime(log_time, sizeof(log_time), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    auto since_epoch = time.time_since_epoch(); // get the duration since epoch

    auto time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch);
    auto time_seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);

    unsigned long long millis = time_millis.count() - 1000*time_seconds.count();

    os_log << "[" << log_time << ":" << std::dec << millis << "]";

    switch(flag)
    {
    case ERR:
        os_log << " / [ERROR]\t";
        break;
    case WARN:
        os_log << " / [WARN]\t";
        break;
    case INFO:
        os_log << " / [INFO]\t";
        break;
    }

    return os_log;
}

int32_t ii, jj, kk, nn, reps, maxnb;
char ch;
char buffer[255];
uint8_t b[255];

uint32_t outputs1, outputs2, ticks1, ticks2;
uint32_t outputs3, outputs4, ticks3, ticks4;
uint32_t outputs5, outputs6, ticks5, ticks6;
uint32_t outputs7, outputs8, ticks7, ticks8;
uint32_t outputs9, outputs10, ticks9, ticks10;
uint32_t outputs11, ticks11;

uint32_t check(string param){

    try{
        uint32_t result = stod(param);
        if (result > 0){
            return uint32_t(result);
    }}//try
    catch (std::invalid_argument const& ex){
        out_log() << "XML parse error std::invalid_argument:: " << ex.what() <<" error in param: "<<param<< '\n';
        return 0;
    }
    catch (std::out_of_range const& ex){
         out_log() << "XML parse error std::out_of_range::" << ex.what() <<" error in param: "<<param<< '\n';
         return 0;
    }

    }//check


bool ShowParameterText(string param, pugi::xml_node node)
{
    out_log() << "Showing parameter : " << param << std::endl;

    if (node==nullptr)
    {
        out_log() << "ERROR no such parameter : " << param << std::endl;
        return false;
    }
    if (strlen(node.text().get())==0)
    {
        out_log() << "ERROR no text for parameter : " << param << std::endl;
        return false;
    }

    //out_log() << param << " strlen text: " << strlen(node.text().get()) << std::endl;
    out_log() << param << " get text: " << node.text().get() << std::endl;
    out_log()<< param << " get as int : " << check(node.text().get())<< std::endl;

    //out_log() << param << " get as double : " << node.text().as_double() << std::endl;
    //out_log() << param << " get as bool: " << node.text().as_bool() << std::endl;

    return true;
} //function ShowParam



uint32_t GetOutputs(uint32_t RF, uint32_t SW, uint32_t ADC, uint32_t GRU){
    uint32_t outputs{0x03000000};
    switch(RF){
    case 0:
        outputs = outputs&0xFFFFFFFE;
        break;
        out_log()<<"RF 0"<<endl;

    case 1:
        outputs = outputs|0x00000001;
        out_log()<<"RF 1"<<endl;
        break;
    }//end switch RF

    out_log()<<hex<<outputs<<endl;
    switch(SW){
    case 0:
        outputs = outputs&0xFFFFFFEF;
        out_log()<<"SW 0"<<endl;
        break;

    case 1:
        outputs = outputs|0x00000010;
        out_log()<<"SW 1"<<endl;
        break;

    }//end switch SW
    out_log()<<hex<<outputs<<endl;

    switch(ADC){
    case 0:
        outputs = outputs&0xFFFFFEFF;
        out_log()<<"ADC 0"<<endl;
        break;

    case 1:
        outputs = outputs|0x00000100;
        out_log()<<"ADC 1"<<endl;
        break;

    }//end switch ADC
    out_log()<<hex<<outputs<<endl;


    switch(GRU){
    case 0:
        outputs = outputs&0xFCFFFFFF;
        break;

    case 1:
        outputs = outputs|0x03000000;
        break;

    }//end switch ADC
    //out_log()<<hex<<outputs;
    out_log()<<hex<<outputs<<endl;

    return outputs;
}

int main()
{
// Making logfile and opening output filestream

    OpenFileLogger();
    EnableConsoleLogger(true);

// READING XML FILE

    const string filepath{""};
    const string filename{"sync_v2.xml"};

    out_log() << filepath+filename << endl;

    pugi::xml_document doc;

    pugi::xml_parse_result result = doc.load_file((filepath+filename).c_str());

    out_log() << boolalpha << showpoint; //boolalpha and noboolalpha

    out_log() << "Load result: " << result.description() << std::endl;

//    ShowParameterText("noparam", doc.child("config").child("nochild") );
//    ShowParameterText("param1", doc.child("config").child("param1") );
//    ShowParameterText("param2", doc.child("config").child("param2") );
//    ShowParameterText("param3", doc.child("config").child("param3") );
//    ShowParameterText("param4", doc.child("config").child("param4") );
//    ShowParameterText("param5", doc.child("config").child("param5") );
//    ShowParameterText("param6", doc.child("config").child("param6") );
//
//    ShowParameterText("config2-param1", doc.child("config").child("config2").child("param1") );

    out_log()<<"ParamCount = ";

    uint32_t ParamCount =  check(doc.child("root").child("ParamCount").text().get());
    out_log()<<ParamCount<<std::endl;
    int *RF_array {new int [ParamCount]{}};
    int *SW_array {new int [ParamCount]{}};
    int *ADC_array {new int [ParamCount]{}};
    //int *GRU_array {new int [ParamCount]{}};
    int *CL_array {new int [ParamCount]{}};

    for (int i = 1; i <= ParamCount; i++){

        string param1 {"RF"+to_string(i)};
        string param2 {"SW"+to_string(i)};
        string param3 {"ADC"+to_string(i)};
        string param4 {"GRU"+to_string(i)};
        string param5 {"CL"+to_string(i)};
//        out_log() << param1;

        RF_array[i] = check(doc.child("root").child("RF").child(param1.c_str()).text().get());
        SW_array[i] = check(doc.child("root").child("SW").child(param2.c_str()).text().get());
        ADC_array[i] = check(doc.child("root").child("ADC").child(param3.c_str()).text().get());
        //GRU_array[i] = check(doc.child("root").child("GRU").child(param4.c_str()).text().get());
        CL_array[i] = check(doc.child("root").child("CL").child(param5.c_str()).text().get());

        ShowParameterText(param1, doc.child("root").child("RF").child(param1.c_str()));
        ShowParameterText(param2, doc.child("root").child("SW").child(param2.c_str()));
        ShowParameterText(param3, doc.child("root").child("ADC").child(param3.c_str()));
        //ShowParameterText(param4, doc.child("root").child("GRU").child(param4.c_str()));
        ShowParameterText(param5, doc.child("root").child("CL").child(param5.c_str()));
    }//for
    doc.save(out_log());

const string OutFilePath {"D:\Synchronisation\dueppwinserial"};
const string OutFileName {"0000"};
const string OutFileNameCommand {"_command"};
const string OutFileNameData {"_data"};
string OutFile;

  out_log() << "START" << endl;

    outputs1 = 0x02000000;
    ticks1 = 0x000000FF;

    outputs2 =  0x00000000;
    ticks2 =  0x000000FF;

    //reps=3;

    reps = 100;

    //test with rf
    uint32_t time_high =1000; //time in us
    uint32_t time_low =1000; //time in us
    uint32_t time_delay =50; //time in us
    uint32_t time_low_corrected =1000; //time in us

    uint32_t out_high =0x00000002; //pin outputs
    uint32_t out_low = 0x00000010; //pin outputs
    uint32_t ticks_high = time_high*1000/20; //number of 20 ns ticks when pins are high
    uint32_t ticks_low_corrected = time_low_corrected*1000/20; //number of 20 ns ticks when pins are high

    //uint8_t ticks_high = 0x2FAF080;
    uint32_t ticks_low = time_low*1000/20; //number of 20 ns ticks when pins are low
    uint32_t ticks_delay = time_delay*1000/20; //number of 20 ns ticks when pins are high
    uint32_t ticks_low_mod = ticks_low - 2*ticks_delay;



    due_prog_t program1;

    due_init_program(&program1, 0);


/*
//example
    due_add_event(&program1, outputs1, ticks1);
    due_add_event(&program1, outputs2, ticks2);
    //due_wait_for_trigger(&program1, outputs2, ticks2);

    for (int ii=0; ii<reps; ii++){
        due_add_event(&program1, outputs1, ticks1);

        due_add_event(&program1, outputs2, ticks2);
    }


*/
/*
    due_add_event(&program1, outputs2, 20);
    due_wait_for_trigger(&program1, out_high, ticks_high);


    for (int ii=0; ii<reps; ii++){
        due_add_event(&program1, out_low, ticks_low);
        due_add_event(&program1, out_high, ticks_high);
    }*/

/*
    due_start_loop(&program1, out_high, ticks_high, reps);//порядок параметров поменять
    due_end_loop(&program1, out_low, ticks_low);*/
    uint32_t XML_outputs{0};

    for(int i=0; i<ParamCount;i++){
        XML_outputs = GetOutputs(RF_array[i],SW_array[i],ADC_array[i],0);
        due_add_event(&program1, XML_outputs,CL_array[i]);
    }


  out_log() << "" << endl;
  out_log() << "Events added" << endl;

    due_exit_program(&program1);

    due_finalize_program(&program1);

   due_dump_program(&program1);

   out_log() << "" << endl;
   out_log() << "Program in HEX" << endl;

    for (int32_t ii=0;  ii<program1.dpos+0; ii++)
    {
    //out_log() << "" << std::hex << std::showbase << std::uppercase << program1.data[ii]<< endl;
    out_log() << "" << setbase(16) << std::setfill ('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< endl;
    }  //for

    /*-----------------------
    out_log() << "" << endl;
    out_log() << "trying due_download_prog_save_to_file() " << endl;

   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC);  //! creates readonly file - wrong permissions
   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   OutFile = OutFilePath + OutFileName;

   int fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   out_log() << std::dec << "file fd = " << fd << endl;

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file(fd, &program1);

    close(fd);

   //-----------------------*/
    out_log() << "" << endl;
    out_log() << "trying due_download_prog_save_to_file_command() " << endl;

    OutFile = OutFilePath + OutFileNameCommand;

   int fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   out_log() << std::dec << "file fd = " << fd << endl;

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file_command(fd, &program1);

    close(fd);


    out_log() << endl<< endl<< "creating Serial object" << endl;

     // Serial object
    serialib serial;

    out_log() << "connecting to Serial object" << endl;

    // Connection to serial port
    //char errorOpening = serial.openDevice(SERIAL_PORT, 38400);
    char errorOpening = serial.openDevice(SERIAL_PORT.c_str(), 38400);


    out_log() << "checking connection" << endl;


    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1)
    {
        printf ("NO connection to %s\n");

        return errorOpening;
    }
    else  printf ("Successful connection to %s\n",SERIAL_PORT.c_str());

  serial.flushReceiver();

 serial.writeChar('S');
//  this_thread::sleep_for(100ms);
  this_thread::sleep_for(100ms);

   ii = serial.available();
   out_log() << "S serial.available = " << ii<< endl;

    serial.readString(buffer, '\n', 254, 1000);
    out_log() << "S serial.readString = " << buffer << endl;


  serial.writeChar('Q');
//  this_thread::sleep_for(100ms);

   ii = serial.available();
   out_log() << "Q serial.available = " << ii<< endl;

    //serial.readChar(&ch, 1000);
    //printf ("ch = %c\n", ch);
    //out_log() << " serial.readChar = " <<  ch << endl;

    serial.readString(buffer, '\n', 254, 1000);
    out_log() << "Q serial.readString = " << buffer << endl;

//// send data length, low byte, high byte
//  cbyte[0] = 'D';
//  cbyte[1] = program->dpos & 0xff;
//  cbyte[2] = (program->dpos>>8)&0xff;
//  write(fd,cbyte,3);

//serial.writeChar('D');
//serial.writeChar(char(program1.dpos & 0xff));
//serial.writeChar(char((program1.dpos>>8)&0xff));
//
//b[0] = uint8_t('D');
//out_log()<<"byte0 = ";
//out_log()<<char(b[0])<<"  ";
//out_log() << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[0]) << endl;
//
//b[1] = uint8_t((program1.dpos>>0)&0xff);
////    out_log() << "" << setbase(16) << std::setfill ('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< endl;
//out_log()<<"byte1 = ";
////out_log()<<b[0]<<"  ";
//out_log() << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[1]) << endl;
////serial.writeChar((b));
////serial.writeBytes(b, 1);
//
//b[2] = uint8_t((program1.dpos>>8)&0xff);
//out_log()<<"byte2 = " ;
////out_log()<<b[0]<<"  ";
//out_log() << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[2]) << endl;
////serial.writeChar(char(b[0]));
////serial.writeBytes(b, 1);
//
//serial.writeBytes(b, 3);
int due_dl{0};
due_dl = due_upload_trajectory(serial, &program1);
//out_log() << std::dec << "" << endl;
//
//   ii = serial.available();
//    out_log() << "D serial.available = " << ii<< endl;
//
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "D serial.readString = " << buffer << endl;
//
//    serial.writeBytes(program1.data, program1.dpos*4);
////    this_thread::sleep_for(100ms);
//
//    ii = serial.available();
//    out_log() << "data serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "data serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   out_log() << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "S serial.readString = " << buffer << endl;
//
////   serial.writeChar('E');
//////   serial.writeChar('e');
//////  this_thread::sleep_for(100ms);
////   ii = serial.available();
////   out_log() << "e serial.available = " << ii<< endl;
////    serial.readString(buffer, '\n', 254, 1000);
////    out_log() << "e serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   out_log() << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "S serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   out_log() << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "S serial.readString = " << buffer << endl;
//
// serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   out_log() << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "S serial.readString = " << buffer << endl;
//
//    out_log() << "" << endl;
//    out_log() << "" << endl;
//    this_thread::sleep_for(1000ms);
//
//
//
//       serial.writeChar('E');
//////   serial.writeChar('e');
//
//   ii = serial.available();
//   out_log() << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "e serial.readString = " << buffer << endl;
//
//
//        this_thread::sleep_for(1000ms);
//
//
//
//       serial.writeChar('E');
//////   serial.writeChar('e');
//
//   ii = serial.available();
//   out_log() << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "e serial.readString = " << buffer << endl;
//
//       ii = serial.available();
//   out_log() << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "e serial.readString = " << buffer << endl;
//
//
//       ii = serial.available();
//   out_log() << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    out_log() << "e serial.readString = " << buffer << endl;


        //this_thread::sleep_for(30000ms);

    out_log() << "" << endl;
    out_log() << "DONE!" << endl;
    return 0;
}
