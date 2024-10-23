//2023.10.03 duepp program format research
//2023.10.06 +due_download_prog() file output
//2023.10.07  +int due_download_prog_save_to_file, +loop for {events}
//10.18 +int due_download_prog_save_to_file_command/data
//11.01 +serial
//13.11.2023 + test program on real data
//06.02.2023 + reading sync pulse sequence from csv file
//22.05.2024 + added out stream to function due_upload_trajectory in serial_lib_cpp.cpp
//25.05.2024 + added stream ability in serial_lib_cpp.cpp
#include <iostream>
#include<iomanip> // for setbase(), works for base 8,10 and 16 only
#include <thread>
#include <chrono>
#include <sstream>
#include <fstream>
#include <list>
#include <iterator>
#include <filesystem>
#include <stdlib.h>
#include "serialib.h" // Serial library
#include "pugixml.hpp"
#include "serial_lib_cpp.cpp"

#define filename "gate_RF_SWITCH_ADC_GRU_CYCLES(1).csv"

using namespace std::literals::chrono_literals;

//#define SERIAL_PORT "\\\\.\\COM1"
//const string SERIAL_PORT{"\\\\.\\COM19"};
const std::string SERIAL_PORT{"\\\\.\\COM5"};

// Util functions for work with xml
namespace utilityFunctions{
    uint32_t check(std::string param){
            uint32_t result = stoi(param);
            std::cout << result << " ";
            if (result >= 0){
                return uint32_t(result);
            }
            else{
                throw std::invalid_argument("Result isn't integer type\n");
            }
    }

    bool ShowParameterText(std::string param, pugi::xml_node node){
        std::cout << "Showing parameter : " << param << '\n';
        if (node==nullptr){
            std::cout << "ERROR no such parameter : " << param << '\n';
            return false;
        }
        if (strlen(node.text().get())==0){
            std::cout << "ERROR no text for parameter : " << param << '\n';
            return false;
        }
        return true;
    }
}

namespace handler{
    // xml handler
    void evalXml(int argc, char ** argv){
        // Handle input xml
    try {
        if (argc < 2) {
            throw std::invalid_argument("Incorrect number of arguments\n");
        }

        std::cout << "Try to load file...\n";
        pugi::xml_parse_result result = doc.load_file(argv[1]);

        if (!result) {
             throw std::invalid_argument("Error in loading file:\n");
        }
    }
    catch(const std::invalid_argument& e) {
        std::cerr << e.what();
        return 6;
    }

    std::cout << "External structure is ok\n";

    std::cout << "ParamCount = ";

    // Check, is this file stucture fits us
    uint32_t ParamCount =  utilityFunctions::check(doc.child("root").child("ParamCount").text().get());
    std::cout << ParamCount << '\n';
    uint32_t *RF_array {new uint32_t [ParamCount]{}};
    uint32_t *SW_array {new uint32_t [ParamCount]{}};
    uint32_t *ADC_array {new uint32_t [ParamCount]{}};
    uint32_t *GX_array {new uint32_t [ParamCount]{}};
    uint32_t *GY_array {new uint32_t [ParamCount]{}};
    uint32_t *GZ_array {new uint32_t [ParamCount]{}};
    uint32_t *CL_array {new uint32_t [ParamCount]{}};
    try {
        for (size_t i = 1; i <= ParamCount; i++){
            std::string param1 = "RF"+std::to_string(i);
            std::string param2 = "SW"+std::to_string(i);
            std::string param3 = "ADC"+std::to_string(i);
            std::string param4 = "GX"+std::to_string(i);
            std::string param5 = "GY"+std::to_string(i);
            std::string param6 = "GZ"+std::to_string(i);
            std::string param7 = "CL"+std::to_string(i);

            // Find necessary tag and put variable in array
            RF_array[i] = utilityFunctions::check(doc.child("root").child("RF").child(param1.c_str()).text().get());
            SW_array[i] = utilityFunctions::check(doc.child("root").child("SW").child(param2.c_str()).text().get());
            ADC_array[i] = utilityFunctions::check(doc.child("root").child("ADC").child(param3.c_str()).text().get());
            GX_array[i] = utilityFunctions::check(doc.child("root").child("GX").child(param4.c_str()).text().get());
            GY_array[i] = utilityFunctions::check(doc.child("root").child("GY").child(param5.c_str()).text().get());
            GZ_array[i] = utilityFunctions::check(doc.child("root").child("GZ").child(param6.c_str()).text().get());
            CL_array[i] = utilityFunctions::check(doc.child("root").child("CL").child(param7.c_str()).text().get());

            // Condition for correctness
            if(!utilityFunctions::ShowParameterText(param1, doc.child("root").child("RF").child(param1.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param2, doc.child("root").child("SW").child(param2.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param3, doc.child("root").child("ADC").child(param3.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param4, doc.child("root").child("GX").child(param4.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param5, doc.child("root").child("GY").child(param5.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param6, doc.child("root").child("GZ").child(param6.c_str())) &&
                                  !utilityFunctions::ShowParameterText(param7, doc.child("root").child("CL").child(param7.c_str()))){
               throw std::invalid_argument("Incorrect xml structure\n");
            }
        }
    }
    catch(const std::invalid_argument& e){
        std::cerr << e.what() << '\n';
        return 5;
    }
    std::cout << "Correct internal structure of xml\n";
    doc.save(std::cout);
    }

    // csv handler
    void evalCsv(int argc, char** argv){
        std::string filename = argv[1];
       std::ifstream work_file(filename);
    std::string line;
    char delimiter = ',';
    uint32_t csv_row_counter = 0;
    uint32_t ticks{0};

    std::list <uint32_t> RF_list;
    std::list <uint32_t> TR_SW_list;
    std::list <uint32_t> ADC_list;
    std::list <uint32_t> GRAD_list;
    std::list <uint32_t> CYCLES_NUM_list;

    while (getline(work_file, line))
    {
        std::stringstream stream(line);
        std::string RF, TR_SW, ADC, GRAD, CYCLES_NUM, del_;

        std::getline(stream, RF, delimiter);
        std::getline(stream, TR_SW, delimiter);
        std::getline(stream, ADC, delimiter);
        std::getline(stream, GRAD, delimiter);
        std::getline(stream, CYCLES_NUM, delimiter);
        RF_list.push_back(stoi(RF));
        TR_SW_list.push_back(stoi(TR_SW));
        ADC_list.push_back(stoi(ADC));
        GRAD_list.push_back(stoi(GRAD));
        CYCLES_NUM_list.push_back(stoi(CYCLES_NUM));

        std::cout << "==================" << '\n';
        std::cout << "RF: " << RF << '\n';
        std::cout << "TR_SW: " << TR_SW << '\n';
        std::cout << "ADC: " << ADC << '\n';
        std::cout << "GRAD: " << GRAD << '\n';
        std::cout << "CYCLES_NUM: " << CYCLES_NUM<< '\n';
        csv_row_counter += 1;
        ticks += stoi(CYCLES_NUM);

    }
    work_file.close();
    auto iter_rf = RF_list.begin();
    auto iter_tr_sw = TR_SW_list.begin();
    auto iter_adc = ADC_list.begin();
    auto iter_grad = GRAD_list.begin();
    auto iter_cycles = CYCLES_NUM_list.begin();

//    for (int i=0; i<csv_row_counter;i++){
//        cout << *iter_rf;
//        iter_rf++;
//    }
    //end of reading pulse sequence from csv file
    }
}

int main(int argc, char** argv)
{
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
    pugi::xml_document doc;
    std::cout << "Start...\n";
     std::string fileName = argv[1];
        std::cout << "File: " << fileName << "\n";

        size_t fileNameSize = fileName.size();
        if (fileNameSize <= 4 || fileName.substr(fileNameSize - 3, 3) != "xml" ||
            fileName.substr(fileNameSize - 3, 3) != "csv") {
            throw std::invalid_argument("Incorrect file name or size\n");
        }
        else {
            if(fileName.substr(fileNameSize - 3, 3) == "xml") evalXml(argc, argv);
            else evalCsv(argc, argv);
        }

const std::string OutFilePath {"D:\Synchronisation\dueppwinserial"};
const std::string OutFileName {"0000"};
const std::string OutFileNameCommand {"_command"};
const std::string OutFileNameData {"_data"};
std::string OutFile;

  std::cout << "START" << '\n';

    outputs1 = 0x00FFFFFF;
    ticks1 = 0x000000FF;

    outputs2 =  0x00000000;
    ticks2 =  0x000000FF;

    reps=3;

    //test with rf
    uint32_t time_high =500000*50; //time in us
    uint32_t time_low =1000*50; //time in us
    uint32_t out_high =0x00FFFFFF; //pin outputs
    uint32_t out_low = 0x00000000; //pin outputs
    uint32_t ticks_high = time_high*1000/20; //number of 20 ns ticks when pins are high
    //uint8_t ticks_high = 0x2FAF080;
    uint32_t ticks_low = time_low*1000/20; //number of 20 ns ticks when pins are low




    due_prog_t program1;

    due_init_program(&program1, 0);

//    uint32_t byte_active = *iter_rf+2*(*iter_tr_sw)+4*(*iter_adc)+8*(*iter_grad);


//    due_add_event(&program1, 0x0000FFFFF, 20);
//    due_add_event(&program1, stoi("0x00"+to_string(byte_active)+ "FFFFF"), *iter_cycles);

//    due_wait_for_trigger(&program1, 0x000FFFF+byte_active, *iter_cycles);
//    reps = 32;
//    due_wait_for_trigger(&program1, 0x002FFFFF, *iter_cycles);

//    iter_cycles++;
//    iter_tr_sw++;
//    iter_adc++;
//    iter_grad++;
//    iter_rf++;
//    byte_active = 0;

    due_add_event(&program1, out_low, time_low);
    //due_wait_for_trigger(&program1, out_low, ticks_low);
//    due_wait_for_trigger(&program1, out_high , time_high - 0.4*time_low);
    due_wait_for_trigger(&program1, out_high , time_high );

    due_add_event(&program1, out_low , time_low);

    for (int i=0; i<99;i++){
        due_add_event(&program1, out_high, time_high);
        due_add_event(&program1, out_low, time_low);
    }
    //end test with rf

//example
//    due_add_event(&program1, outputs1, ticks1);
//    //due_wait_for_trigger(&program1, outputs2, ticks2);
//    due_wait_for_trigger(&program1, outputs2, ticks2);
//
//    for (int ii=0; ii<reps; ii++){
//        due_add_event(&program1, outputs1, ticks1);
//
//        due_add_event(&program1, outputs2, ticks2);
//    }



//due_start_loop(&program1, reps ,0x00FFFFF, ticks);
//
//    for (jj = 0; jj < csv_row_counter-1; jj++){
//        byte_active = *iter_rf+2*(*iter_tr_sw)+4*(*iter_adc)+8*(*iter_grad);
//        byte_active = *iter_rf+2*(*iter_tr_sw)+4*(*iter_adc)+8*(*iter_grad);
//
//        cout <<"0x"<<hex<< "0x00"+to_string(byte_active)+ "FFFFF"<<"\n";
//        due_add_event(&program1, stoi("0x00"+to_string(byte_active)+ "FFFFF"), *iter_cycles);
//        due_add_event(&program1, 0x00FFFFFF, *iter_cycles);
//
//        iter_rf++;
//        iter_cycles++;
//        iter_tr_sw++;
//        iter_adc++;
//        iter_grad++;
//        byte_active = 0;
//    }//end for
// due_end_loop(&program1, 0x000FFFFF, ticks);
//


   //due_add_event(&program1, 2, 63);
   //due_wait_for_trigger(&program1, 3, 127);
    //due_write_dacs(&program1, 255, 255, 4, 32);

  std::cout << "" << '\n';
  std::cout << "Events added" << '\n';

    due_exit_program(&program1);

    due_finalize_program(&program1);

   due_dump_program(&program1);

   std::cout << "" << '\n';
   std::cout << "Program in HEX" << '\n';

    for (int32_t ii=0;  ii<program1.dpos+0; ii++)
    {
    //cout << "" << std::hex << std::showbase << std::uppercase << program1.data[ii]<< endl;
    std::cout << "" << std::setbase(16) << std::setfill ('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< '\n';
    }  //for

    //-----------------------
    std::cout << "" << '\n';
    std::cout << "trying due_download_prog_save_to_file() " << '\n';

   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC);  //! creates readonly file - wrong permissions
   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   OutFile = OutFilePath + OutFileName;

   int fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   std::cout << std::dec << "file fd = " << fd << '\n';

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file(fd, &program1);

    close(fd);

   //-----------------------
    std::cout << "" << '\n';
    std::cout << "trying due_download_prog_save_to_file_command() " << '\n';

    OutFile = OutFilePath + OutFileNameCommand;

   fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   std::cout << std::dec << "file fd = " << fd << '\n';

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file_command(fd, &program1);

    close(fd);

//-----------------------
    std::cout << "" << '\n';
    std::cout << "trying due_download_prog_save_to_file_data() " << '\n';

    OutFile = OutFilePath + OutFileNameData;

   fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   std::cout << std::dec << "file fd = " << fd << '\n';

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file_data(fd, &program1);

    close(fd);

//-----------------------


    std::cout << '\n\n' << "creating Serial object" << '\n';

     // Serial object
    serialib serial;

    std::cout << "connecting to Serial object" << '\n';

    // Connection to serial port
    //char errorOpening = serial.openDevice(SERIAL_PORT, 38400);
    char errorOpening = serial.openDevice(SERIAL_PORT.c_str(), 38400);


    std::cout << "checking connection" << '\n';


    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1)
    {
        printf ("NO connection to %s\n",SERIAL_PORT.c_str());
        return errorOpening;
    }
    else  printf ("Successful connection to %s\n",SERIAL_PORT.c_str());

    serial.flushReceiver();
    serial.writeChar('S');
//  this_thread::sleep_for(100ms);

    std::this_thread::sleep_for(100ms);
    ii = serial.available();
    std::cout << "S serial.available = " << ii << '\n';

    serial.readString(buffer, '\n', 254, 1000);
    std::cout << "S serial.readString = " << buffer << '\n';


  serial.writeChar('Q');
//  this_thread::sleep_for(100ms);

   ii = serial.available();
   std::cout << "Q serial.available = " << ii<< '\n';

    //serial.readChar(&ch, 1000);
    //printf ("ch = %c\n", ch);
    //cout << " serial.readChar = " <<  ch << endl;

    serial.readString(buffer, '\n', 254, 1000);
    std::cout << "Q serial.readString = " << buffer << '\n';

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
//cout<<"byte0 = ";
//cout<<char(b[0])<<"  ";
//cout << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[0]) << endl;
//
//b[1] = uint8_t((program1.dpos>>0)&0xff);
////    cout << "" << setbase(16) << std::setfill ('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< endl;
//cout<<"byte1 = ";
////cout<<b[0]<<"  ";
//cout << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[1]) << endl;
////serial.writeChar((b));
////serial.writeBytes(b, 1);
//
//b[2] = uint8_t((program1.dpos>>8)&0xff);
//cout<<"byte2 = " ;
////cout<<b[0]<<"  ";
//cout << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[2]) << endl;
////serial.writeChar(char(b[0]));
////serial.writeBytes(b, 1);
//
//serial.writeBytes(b, 3);
int due_dl{0};
due_dl = due_upload_trajectory(serial, &program1);
//cout << std::dec << "" << endl;
//
//   ii = serial.available();
//    cout << "D serial.available = " << ii<< endl;
//
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "D serial.readString = " << buffer << endl;
//
//    serial.writeBytes(program1.data, program1.dpos*4);
////    this_thread::sleep_for(100ms);
//
//    ii = serial.available();
//    cout << "data serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "data serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   cout << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "S serial.readString = " << buffer << endl;
//
////   serial.writeChar('E');
//////   serial.writeChar('e');
//////  this_thread::sleep_for(100ms);
////   ii = serial.available();
////   cout << "e serial.available = " << ii<< endl;
////    serial.readString(buffer, '\n', 254, 1000);
////    cout << "e serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   cout << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "S serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   cout << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "S serial.readString = " << buffer << endl;
//
// serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   cout << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "S serial.readString = " << buffer << endl;
//
//    cout << "" << endl;
//    cout << "" << endl;
//    this_thread::sleep_for(1000ms);
//
//
//
//       serial.writeChar('E');
//////   serial.writeChar('e');
//
//   ii = serial.available();
//   cout << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "e serial.readString = " << buffer << endl;
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
//   cout << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "e serial.readString = " << buffer << endl;
//
//       ii = serial.available();
//   cout << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "e serial.readString = " << buffer << endl;
//
//
//       ii = serial.available();
//   cout << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    cout << "e serial.readString = " << buffer << endl;


        //this_thread::sleep_for(30000ms);

    std::cout << "" << '\n';
    std::cout << "DONE!" << '\n';
    return 0;
}
